from Ethercat_Communication import Epos4_definition as E_D
import pysoem
from collections import namedtuple
import threading
import yaml
from Ethercat_Communication import common
import sys
import struct
import time
import os
dirname = os.path.dirname(__file__)
filename2 = os.path.join(dirname, '../Config/Ethercat_config.yaml')
class _get_driver_config_data:
    def __init__(self):

        """read config files"""
        with open(filename2) as descrip_file:
            self.data = yaml.load(descrip_file, Loader=yaml.FullLoader)
    

        """interface name""" 
        self.Interface_name                     = self.data['port']


        self.EPOS4_Product_Code                 = self.data['driver_info']['EPOS4']['product_code']

        self.EPOS4_Vendor_ID                    = self.data['driver_info']['EPOS4']['vendor_id']

        self.number_of_slave                    = self.data['network']['slaves_count']


        print('interface name: ', self.Interface_name)

        
    

class EC_MAIN_CONTROLLER(_get_driver_config_data, threading.Thread):

    def __init__(self):
        

        _get_driver_config_data.__init__(self)
        threading.Thread.__init__(self)

        self._stop_thread                       = False
        self._ifname = self.Interface_name
        # self._pd_thread_stop_event                          = threading.Event()
        # self._ch_thread_stop_event                          = threading.Event()
        self._actual_wkc = 0

        
        """Master Instance"""
        self._master = pysoem.Master()
        self._master.in_op = False
        self._master.do_check_state = False
        SlaveSet = namedtuple('SlaveSet', 'name product_code config_func joint_name vendor_id')
        self._expected_slave_mapping = {}

        """Update expected slave mapping"""
        for i in range(self.number_of_slave):
            if( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] == 'GOLD_DRIVE' ):    
                self._expected_slave_mapping[i] =  SlaveSet( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'],
                                                             self.data['driver_info'][ self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ]['product_code'] ,
                                                             self._Gold_drive_config_function, 
                                                             self.data['network']['slaves_info']['slave_'+str(i)]['joint_name'],
                                                             self.data['driver_info'][ self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ]['vendor_id'] )
        
            if( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] == 'EPOS4' ):    
                self._expected_slave_mapping[i] =  SlaveSet( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'],
                                                             self.data['driver_info'][ self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ]['product_code'] ,
                                                             self._config_function, 
                                                             self.data['network']['slaves_info']['slave_'+str(i)]['joint_name'],
                                                             self.data['driver_info'][ self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ]['vendor_id'] )
        
            if( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] == 'MC5004_DRIVER' ):    
                self._expected_slave_mapping[i] =  SlaveSet( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'],
                                                             self.data['driver_info'][ self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ]['product_code'] ,
                                                             self._MC_drive_config_function, 
                                                             self.data['network']['slaves_info']['slave_'+str(i)]['joint_name'],
                                                             self.data['driver_info'][ self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ]['vendor_id'] )
        
            if( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] == 'MAXPOS' ):    
                self._expected_slave_mapping[i] =  SlaveSet( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'],
                                                             self.data['driver_info'][ self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ]['product_code'] ,
                                                             self._Maxpos_driver_config_function, 
                                                             self.data['network']['slaves_info']['slave_'+str(i)]['joint_name'],
                                                             self.data['driver_info'][ self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ]['vendor_id'] )
        
        print('vendor_id',self._expected_slave_mapping[i].vendor_id)
        print('productcode',self._expected_slave_mapping[i].product_code)
        self._master.open(self._ifname)

        # config_init returns the number of slaves found
        if self._master.config_init() > 0:

            print("{} slaves found and configured".format(
                len(self._master.slaves)))

            for i, slave in enumerate(self._master.slaves):
                assert(slave.man == self._expected_slave_mapping[i].vendor_id )
                assert(
                    slave.id == self._expected_slave_mapping[i].product_code)
                slave.config_func = self._expected_slave_mapping[i].config_func
                print('current_state_____' ,self._master.state_check(pysoem.SAFEOP_STATE, 50000) )
                #this is only used for Faulhaber driver to let it into operation mode
                if( slave.id == 0x3139 ):
                    # PREOP_STATE to SAFEOP_STATE request - each slave's config_func is called            
                    self.SDO_write( i , [0x1c33, 0x01, 'UNSIGNED16'] , 0x22 )
                    self.SDO_write( i , [0x1c32, 0x01, 'UNSIGNED16'] , 0x01 )



            self._master.config_map()

            # wait 50 ms for all slaves to reach SAFE_OP state
            if self._master.state_check(pysoem.SAFEOP_STATE, 50000*10) != pysoem.SAFEOP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.SAFEOP_STATE:
                        # print('{} did not reach SAFEOP state'.format(slave.name()))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                
                print('send driver back to Pre-Operation')

                #back to Init State
                self._master.state = pysoem.INIT_STATE                
                self._master.write_state()
                #waiting for device get Init State
                print( '>>>current state: ', self._master.state_check(pysoem.INIT_STATE, 50000) )

                #back to Pre-Operation State
                self._master.state = pysoem.PREOP_STATE                
                self._master.write_state()
                #waiting for device get Init State
                print( '>>>current state: ', self._master.state_check(pysoem.PREOP_STATE, 50000) )

            else:
                self._master.state = pysoem.OP_STATE #pysoem.PREOP_STATE

                self._master.send_processdata()
                self._master.receive_processdata()
                
                self._master.write_state()
                print('>>>preop: ',pysoem.PREOP_STATE )
                print('>>>OP_STATE: ',pysoem.OP_STATE )

                print( '>>>current state: ', self._master.state_check(pysoem.OP_STATE, 50000) )

                checking_count = 200

                while( ( checking_count != 0 ) and  
                        ( self._master.state_check(pysoem.OP_STATE, 50000) != pysoem.OP_STATE)  ):

                    checking_count = checking_count - 1
                    self._master.send_processdata()
                    self._master.receive_processdata()
                
                time.sleep(0.01)
                
                self.start_synchronization()


    def start_synchronization(self):
        self.start()


    def _config_function(self,slave_pos):
        """This function is used to config PDO map while device is in safe-operation mode"""
        slave = self._master.slaves[slave_pos]

       
        slave.sdo_write(0x1A00, 0, struct.pack('<B',0) )

        slave.sdo_write(0x1A00, 0x01,struct.pack('<I',0x60640020) )
        slave.sdo_write(0x1A00, 0x02,struct.pack('<I',0x60FD0020) )        
        slave.sdo_write(0x1A00, 0x03,struct.pack('<I',0x60410010) )
        
        slave.sdo_write(0x1A00, 0, struct.pack('<B',3))


        slave.sdo_write(0x1600, 0, struct.pack('<B',0) )

        slave.sdo_write(0x1600, 0x01,struct.pack('<I',0x60400010) )

        slave.sdo_write(0x1600, 0x02,struct.pack('<I',0x60600008) )
    
        slave.sdo_write(0x1600, 0x03,struct.pack('<I',0x60830020) )
    
        slave.sdo_write(0x1600, 0x04,struct.pack('<I',0x60840020) )
        
        slave.sdo_write(0x1600, 0x05,struct.pack('<I',0x607A0020) )

        slave.sdo_write(0x1600, 0x06,struct.pack('<I',0x60810020) )

        slave.sdo_write(0x1600, 0, struct.pack('<B',6) )

       
        print("config PDO of slave ",slave_pos," done!")

    def _Maxpos_driver_config_function(self,slave_pos):
        """This function is used to config PDO map while device is in safe-operation mode"""
        slave = self._master.slaves[slave_pos]

       
        slave.sdo_write(0x1A00, 0, struct.pack('<B',0) )

        slave.sdo_write(0x1A00, 0x01,struct.pack('<I',0x60640020) )
        slave.sdo_write(0x1A00, 0x02,struct.pack('<I',0x60FD0020) )        
        slave.sdo_write(0x1A00, 0x03,struct.pack('<I',0x60410010) )
        
        slave.sdo_write(0x1A00, 0, struct.pack('<B',3))


        slave.sdo_write(0x1600, 0, struct.pack('<B',0) )

        slave.sdo_write(0x1600, 0x01,struct.pack('<I',0x60400010) )

        slave.sdo_write(0x1600, 0x02,struct.pack('<I',0x60600008) )
    
        slave.sdo_write(0x1600, 0x03,struct.pack('<I',0x60830020) )
    
        slave.sdo_write(0x1600, 0x04,struct.pack('<I',0x60840020) )
        
        slave.sdo_write(0x1600, 0x05,struct.pack('<I',0x607A0020) )

        slave.sdo_write(0x1600, 0x06,struct.pack('<I',0x60810020) )

        slave.sdo_write(0x1600, 0, struct.pack('<B',6) )

        print("config PDO of slave ",slave_pos," done!")

    
    def _Gold_drive_config_function(self,slave_pos):
        slave = self._master.slaves[slave_pos]
        #disable RX PDO assignments object
        slave.sdo_write(0x1C12, 0x00, struct.pack('<H',0) )
        #write to assign RX PDO object
        slave.sdo_write(0x1C12, 0x01, struct.pack('<H',0x1607) )
        #enable RX PDO assignments object
        slave.sdo_write(0x1C12, 0x00, struct.pack('<H',1) )
        
        #disable TX PDO assignments object
        slave.sdo_write(0x1C13, 0x00, struct.pack('<H',0) )

        slave.sdo_write(0x1C13, 0x01, struct.pack('<H',0x1A07) )

        #enable TX PDO assignments object
        slave.sdo_write(0x1C13, 0x00, struct.pack('<H',1) )   


       #disable Receive PDO Mapping
        slave.sdo_write(0x1A07, 0x00, struct.pack('<B',0)            )

        slave.sdo_write(0x1A07, 0x01,struct.pack('<I',0x60640020) )
        slave.sdo_write(0x1A07, 0x02,struct.pack('<I',0x60FD0020) )        
        slave.sdo_write(0x1A07, 0x03,struct.pack('<I',0x60410010) )
        # #mapping actual velocity
        slave.sdo_write(0x1A07, 0x04,struct.pack('<I',0x606C0020) )
        
        #enable Receive PDO Mapping
        slave.sdo_write(0x1A07, 0x00, struct.pack('<B',4)            )
    



        #disable Receive PDO Mapping
        slave.sdo_write(0x1607, 0, struct.pack('<B',0) )

        #mapping target position
        slave.sdo_write(0x1607, 0x01,struct.pack('<I',0x607A0020) )
        
        #mapping digital output
        slave.sdo_write(0x1607, 0x02,struct.pack('<I',0x60FE0120) )
        
        #mapping control word
        slave.sdo_write(0x1607, 0x03,struct.pack('<I',0x60400010) )

        #mapping profile accelerate
        slave.sdo_write(0x1607, 0x04,struct.pack('<I',0x60830020) )
        
        #mapping profile deaccelerate
        slave.sdo_write(0x1607, 0x05,struct.pack('<I',0x60840020) )

        #mapping profile velocity
        slave.sdo_write(0x1607, 0x06,struct.pack('<I',0x60810020) )

        #mapping target position
        slave.sdo_write(0x1607, 0x07,struct.pack('<I',0x60600008) )

        #enable Receive PDO Mapping
        slave.sdo_write(0x1607, 0, struct.pack('<B',7)            )


        
 


    def _MC_drive_config_function(self,slave_pos):
        """This function is used to config PDO map while device is in safe-operation mode"""
        slave = self._master.slaves[slave_pos]
        #disable RX PDO assignments object
        slave.sdo_write(0x1C12, 0x00, struct.pack('<B',0) )
        #write to assign RX PDO object
        slave.sdo_write(0x1C12, 0x01, struct.pack('<H',0x1600) )
        slave.sdo_write(0x1C12, 0x02, struct.pack('<H',0x1601) )
        #enable RX PDO assignments object
        slave.sdo_write(0x1C12, 0x00, struct.pack('<B',2) )
        
        #disable TX PDO assignments object
        slave.sdo_write(0x1C13, 0x00, struct.pack('<B',0) )
        #enable TX PDO assignments object
        slave.sdo_write(0x1C13, 0x00, struct.pack('<B',1) )   



        #remap sending PDO Mapping
        slave.sdo_write(0x1A00, 0, struct.pack('<B',0) )

        #Position Actual Value
        slave.sdo_write(0x1A00, 0x01,struct.pack('<I',0x60640020) )
        #status word
        slave.sdo_write(0x1A00, 0x02,struct.pack('<I',0x60410010) )
        
        slave.sdo_write(0x1A00, 0, struct.pack('<B',2))

    
        #disable Receive PDO Mapping    
        slave.sdo_write(0x1600, 0, struct.pack('<B',0) )

        #mapping target position
        slave.sdo_write(0x1600, 0x01,struct.pack('<I',0x607A0020) )
        
        #mapping digital output
        # slave.sdo_write(0x1600, 0x02,struct.pack('<I',0x60FE0120) )
        
        #mapping control word
        slave.sdo_write(0x1600, 0x02,struct.pack('<I',0x60400010) )

        #mapping profile accelerate
        slave.sdo_write(0x1600, 0x03,struct.pack('<I',0x60830020) )
        
        #enable Receive PDO Mapping
        slave.sdo_write(0x1600, 0, struct.pack('<B',3) )


        #disable Receive PDO Mapping
        slave.sdo_write(0x1601, 0, struct.pack('<B',0) )

        #mapping profile deaccelerate
        slave.sdo_write(0x1601, 0x01,struct.pack('<I',0x60840020) )

        #mapping profile velocity
        slave.sdo_write(0x1601, 0x02,struct.pack('<I',0x60810020) )

        #mapping target position
        slave.sdo_write(0x1601, 0x03,struct.pack('<I',0x60600008) )

        #enable Receive PDO Mapping
        slave.sdo_write(0x1601, 0, struct.pack('<B',3)  )
      
    def SDO_read(self,slave_name, object ):
        """ This """
        """get slave's key in slave mapping"""
        # _slave_pos = list(self._expected_slave_mapping.keys())[list(self._expected_slave_mapping.values()).index(slave_name)]

        _slave_pos                              = slave_name

        _index                                  = object[0]

        _subindex                               = object[1]

        _data_size                              = common.c_type_size[ object[2] ]

        print('data size',_data_size)

        _response_data                          = self._master.slaves[_slave_pos].sdo_read( _index,
                                                                                            _subindex, 
                                                                                            _data_size  )
        print(_response_data)
        return struct.unpack( common.c_type[ object[2] ], _response_data  )[0] 


    def SDO_write(self,slave_name, object , value ):
        """ This """
        """get slave's key in slave mapping"""
        # _slave_pos = list(self._expected_slave_mapping.keys())[list(self._expected_slave_mapping.values()).index(slave_name)]

        _slave_pos                              = slave_name

        _index                                  = object[0]

        _subindex                               = object[1]

        _data_size                              = common.c_type[ object[2] ]

        _packing_data                           = struct.pack(_data_size,value )        


        _response_data                          = self._master.slaves[_slave_pos].sdo_write( _index,
                                                                                             _subindex, 
                                                                                             _packing_data  )

        # return struct.unpack( common.c_type[ object[2] ], _response_data  )[0] 

    def read_eeprom(self,slave_name, address):
        print( self._master.slaves[slave_name].eeprom_read( address) )

    def write_eeprom(self,slave_name, address, data):
        self._master.slaves[slave_name].eeprom_write( address, data )

    def get_slave_node_ID(self, slave_index):

        return self.SDO_read(slave_index,E_D.NODE_ID)


    def set_output_data(self,slave_ID, parsing_info, *control_word_data):

        _packed_data                            = struct.pack( parsing_info , *control_word_data)

        self._master.slaves[slave_ID].output    = _packed_data    

    def set_output_data_test(self,slave_ID, parsing_info, *control_word_data):

        print(  struct.pack( parsing_info , *control_word_data) )

    def get_output_data(self,slave_ID, parsing_info ):
        """this fucntion is used to get the data which is transmitted to slave
           
           @parameter:

                slave_ID: the id of slave count from 0
                parsing_info: struct format data that is used for parsing data from output 
            
            @return:
                tupple of data in output """
        print('output len', self._master.slaves[slave_ID].output )
        return struct.unpack( parsing_info , self._master.slaves[slave_ID].output )

    def get_input_data(self,slave_ID, parsing_info ):
        """this fucntion is used to get the data which has received from slave
           
           @parameter:

                slave_ID: the id of slave count from 0
                parsing_info: struct format data that is used for parsing data from output 
            
            @return:
                tupple of data is sent from slave """

        data_unpack                             = struct.unpack( parsing_info , self._master.slaves[slave_ID].input )
        
        return data_unpack      


    def run(self):

        print('---output len', len( self._master.slaves[0].output ))
        
        while( self._stop_thread != True ):
            
            self._master.send_processdata()
            self._master.receive_processdata()
            time.sleep(4*10**(-4))
            # time.sleep(0.0001)


    def stop_driver_communicate(self):
        self._stop_thread                       = True
        time.sleep(0.5)
        self.join()


            




        





