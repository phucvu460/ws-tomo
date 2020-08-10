from Ethercat_Communication import EC_controller as EC_Ctr
from dataclasses import dataclass
from collections import namedtuple
from threading import Thread
import threading
import time
import queue
from Ethercat_Communication import Gold_drive_definition as GD_def
from Ethercat_Communication import Epos4_definition as EP_def
from Ethercat_Communication import MC_definition as MC_def
from Ethercat_Communication import Maxpos_definition as MP_def
import struct
import math 
from Ethercat_Communication import common
import yaml
import os
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../Config/Ethercat_config.yaml')
# _EC_Controller                                  = EC_Ctr.EC_MAIN_CONTROLLER()


#define
TARGET_ARRAY_POS                                = 0
SPEED_ARRAY_POS                                 = 1
ACCE_ARRAY_POS                                  = 2
DEACCE_ARRAY_POS                                = 3

@dataclass
class SlaveInfo:
    ID:int                                   
    DRIVER_TYPE:str
    BELONG_TO:str
    PARSING_INFO:str  
    JOINT_NAME:str
    GEAR_RATIO:int                              = 1
    ENCODER_RES:int                             = 1
    TARGET_DEACCE:int                           = 5000
    TARGET_ACCE:int                             = 5000
    TARGET_VELOCITY:int                         = 5000
    ACTUAL_TORQUE:int                           = 0
    TARGET_POS:int                              = 0
    TARGET_REACH:bool                           = False
    CONTROL_WORD:int                            = 0
    STATUS_WORD:int                             = 0
    MODE_OF_OPERATION:int                       = 1
    DIGITAL_OUTPUT:int                          = 0
    ACTUAL_POS:int                              = 0
    DIGITAL_INPUT:int                           = 0
    ACTUAL_VELOCITY:int                         = 0 
    DRIVE_DEF:'typing.Any'                      = EP_def                          

class MAIL_BOX(Thread):
    def __init__(self):
        #get slave info
        """read config files"""
        with open(filename) as descrip_file:
            self.data = yaml.load(descrip_file, Loader=yaml.FullLoader)
    

        """interface name""" 
        self.number_of_slave                    = self.data['network']['slaves_count']

        SlaveSet = namedtuple('SlaveSet', 'name joint_name ratio encoder_res belong_to')
        self._expected_slave_mapping = {}
        self._lock                              = threading.Lock()
        self._slave_set                         = {}
        self.control_word                       = 0

        self._temppp                            = 0
        self.access                             = 0
        for i in range(self.number_of_slave):
            self._expected_slave_mapping[i]     =  SlaveSet( self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'],
                                                             self.data['network']['slaves_info']['slave_'+str(i)]['joint_name']  ,
                                                             self.data['network']['slaves_info']['slave_'+str(i)]['ratio']       ,
                                                             self.data['network']['slaves_info']['slave_'+str(i)]['encoder_res'] ,
                                                             self.data['network']['slaves_info']['slave_'+str(i)]['belong_to']      )
            
            self.mail_box_register( i , self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand'] ,                                    
                                    self.data['network']['slaves_info']['slave_'+str(i)]['joint_name']       ,
                                    self.data['network']['slaves_info']['slave_'+str(i)]['ratio']            ,
                                    self.data['network']['slaves_info']['slave_'+str(i)]['encoder_res'] ,
                                    self.data['network']['slaves_info']['slave_'+str(i)]['belong_to']        )
            # self.mail_box_register( i , self.data['network']['slaves_info']['slave_'+str(i)]['driver_brand']  )
        

        self._EC_Controller                                  = EC_Ctr.EC_MAIN_CONTROLLER()

        Thread.__init__(self)


        self.start()





    def mail_box_register(self, id, driver_type,joint_name,ratio,encoder_res,belong_to):

        self._lock.acquire()      

        print(joint_name)
        if( driver_type == 'GOLD_DRIVE' ):
        
            self._slave_set[id]                 = SlaveInfo( id ,
                                                             driver_type ,
                                                             belong_to,
                                                             '<iIHIIIb'  ,
                                                             joint_name  ,
                                                             ratio       ,
                                                             encoder_res  )      
 
            self._slave_set[id].DRIVE_DEF       = GD_def
        elif( driver_type == 'EPOS4' ):
        
            self._slave_set[id]                 = SlaveInfo( id ,
                                                             driver_type ,
                                                             belong_to,
                                                             '<HBIIiI'    ,
                                                             joint_name  ,
                                                             ratio       ,
                                                             encoder_res  ) 
            
            self._slave_set[id].DRIVE_DEF       = EP_def

        elif( driver_type == 'MC5004_DRIVER' ):
        
            self._slave_set[id]                 = SlaveInfo( id ,
                                                             driver_type ,
                                                             belong_to,
                                                             '<iHIIIb'   ,
                                                             joint_name  ,
                                                             ratio       ,
                                                             encoder_res  ) 
            
            self._slave_set[id].DRIVE_DEF       = MC_def

        elif( driver_type == 'MAXPOS' ):
        
            self._slave_set[id]                 = SlaveInfo( id ,
                                                             driver_type ,
                                                             belong_to,
                                                             '<HBIIiI'    ,
                                                             joint_name  ,
                                                             ratio       ,
                                                             encoder_res  ) 
            
            self._slave_set[id].DRIVE_DEF       = MP_def


        print(self._slave_set[id])

        self._lock.release()   
    
    def servo_on(self,id):
        self._slave_set[id].CONTROL_WORD        = 0x00
        time.sleep(0.05)

        self._slave_set[id].CONTROL_WORD        = 0x80
        time.sleep(0.05)

        self._slave_set[id].CONTROL_WORD        = 0b110
        time.sleep(0.05)

        self._slave_set[id].CONTROL_WORD        = 0b10111
        time.sleep(0.05)

        self._slave_set[id].CONTROL_WORD        = 0b11111
        time.sleep(0.05)


    def servo_off(self,id):
        self._slave_set[id].CONTROL_WORD        = 0x00
        time.sleep(0.05)

        self._slave_set[id].CONTROL_WORD        = 0x80
        time.sleep(0.05)

        self._slave_set[id].CONTROL_WORD        = 0b110
        time.sleep(0.05)
  
    def run(self):


        while(True):    
            
            if( self._lock == True ):
                pass
            
            else:
                for slave in self._slave_set.items():

                    if(slave[1].DRIVER_TYPE == 'EPOS4'):

                        self._EC_Controller.set_output_data( slave[1].ID , 
                                                        slave[1].PARSING_INFO, 
                                                        slave[1].CONTROL_WORD,
                                                        slave[1].MODE_OF_OPERATION,
                                                        slave[1].TARGET_ACCE,
                                                        slave[1].TARGET_DEACCE,
                                                        slave[1].TARGET_POS,
                                                        slave[1].TARGET_VELOCITY  )
                        
                        
                    elif(slave[1].DRIVER_TYPE == 'GOLD_DRIVE'):
                        #set PDO output data
                        self._EC_Controller.set_output_data( slave[1].ID , 
                                                        slave[1].PARSING_INFO,
                                                        slave[1].TARGET_POS,
                                                        slave[1].DIGITAL_OUTPUT,
                                                        slave[1].CONTROL_WORD ,
                                                        slave[1].TARGET_ACCE ,
                                                        slave[1].TARGET_DEACCE,
                                                        slave[1].TARGET_VELOCITY,
                                                        slave[1].MODE_OF_OPERATION  )
                    
                    elif( slave[1].DRIVER_TYPE == 'MC5004_DRIVER' ):
                        #set PDO output data
                        self._EC_Controller.set_output_data( slave[1].ID , 
                                                        slave[1].PARSING_INFO,
                                                        slave[1].TARGET_POS,
                                                        slave[1].CONTROL_WORD ,
                                                        slave[1].TARGET_ACCE ,
                                                        slave[1].TARGET_DEACCE,
                                                        slave[1].TARGET_VELOCITY,
                                                        slave[1].MODE_OF_OPERATION  )

                    elif(slave[1].DRIVER_TYPE == 'MAXPOS'):

                        self._EC_Controller.set_output_data( slave[1].ID , 
                                                        slave[1].PARSING_INFO, 
                                                        slave[1].CONTROL_WORD,
                                                        slave[1].MODE_OF_OPERATION,
                                                        slave[1].TARGET_DEACCE,
                                                        slave[1].TARGET_ACCE,
                                                        slave[1].TARGET_POS,
                                                        slave[1].TARGET_VELOCITY  )
                    
                    #get PDO input data                  
                    self.get_input_data( slave[1].ID )
                    
                    #update reach target postion status
                    self._update_target_reach_status(slave[1].ID)
                    
            time.sleep(0.001)

    def get_input_data(self,id ):
        if( self._slave_set[id].DRIVER_TYPE == 'GOLD_DRIVE' ):

            data_obtain_tuple                       = self._EC_Controller.get_input_data(id,'<iIHi')
            self._slave_set[id].ACTUAL_POS          = data_obtain_tuple[0]
            self._slave_set[id].DIGITAL_INPUT       = data_obtain_tuple[1]
            self._slave_set[id].STATUS_WORD         = data_obtain_tuple[2]
            self._slave_set[id].ACTUAL_VELOCITY     = data_obtain_tuple[3]

        elif( self._slave_set[id].DRIVER_TYPE == 'EPOS4' ):

            data_obtain_tuple                       = self._EC_Controller.get_input_data(id,'<iIH')
            self._slave_set[id].ACTUAL_POS          = data_obtain_tuple[0]
            self._slave_set[id].DIGITAL_INPUT       = data_obtain_tuple[1]
            self._slave_set[id].STATUS_WORD         = data_obtain_tuple[2]
        
        elif( self._slave_set[id].DRIVER_TYPE == 'MC5004_DRIVER' ):

            data_obtain_tuple                       = self._EC_Controller.get_input_data(id,'<iH')
            self._slave_set[id].ACTUAL_POS          = data_obtain_tuple[0]
            # self._slave_set[id].DIGITAL_INPUT       = data_obtain_tuple[1]
            self._slave_set[id].STATUS_WORD         = data_obtain_tuple[1]
        
        elif( self._slave_set[id].DRIVER_TYPE == 'MAXPOS' ):

            data_obtain_tuple                       = self._EC_Controller.get_input_data(id,'<iIH')
            self._slave_set[id].ACTUAL_POS          = data_obtain_tuple[0]
            self._slave_set[id].DIGITAL_INPUT       = data_obtain_tuple[1]
            self._slave_set[id].STATUS_WORD         = data_obtain_tuple[2]

 
    def get_EPOS_input_data(self,id ):
        data_obtain_tuple                       = self._EC_Controller.get_input_data(id,'iIH')
        self._slave_set[id].ACTUAL_POS          = data_obtain_tuple[0]
        self._slave_set[id].DIGITAL_INPUT       = data_obtain_tuple[1]
        self._slave_set[id].STATUS_WORD         = data_obtain_tuple[2]
        print(self._slave_set[id].ACTUAL_POS,self._slave_set[id].STATUS_WORD)

    def test(self,id):
        print( self._slave_set[id].ACTUAL_POS )
        print( self._slave_set[id].DIGITAL_INPUT )
        print(self._slave_set[id].STATUS_WORD )
        self._EC_Controller.set_output_data_test(self._slave_set[id].ID , 
                                            '<HBIIi', 
                                            self._slave_set[id].CONTROL_WORD,
                                            self._slave_set[id].MODE_OF_OPERATION,
                                            self._slave_set[id].TARGET_DEACCE,
                                            self._slave_set[id].TARGET_ACCE,
                                            self._slave_set[id].TARGET_POS      )
        print( struct.pack( '<HBIIi', 
                            self._slave_set[id].CONTROL_WORD,
                            self._slave_set[id].MODE_OF_OPERATION,
                            self._slave_set[id].TARGET_DEACCE,
                            self._slave_set[id].TARGET_ACCE,
                            self._slave_set[id].TARGET_POS  ))
        a = struct.pack( '<HBIIi', 
                            self._slave_set[id].CONTROL_WORD,
                            self._slave_set[id].MODE_OF_OPERATION,
                            self._slave_set[id].TARGET_DEACCE,
                            self._slave_set[id].TARGET_ACCE,
                            self._slave_set[id].TARGET_POS  )
        print(struct.unpack('<HBIIi', a) )

    def get_output_data(self,id):
        print( 'parsing info',self._slave_set[id].PARSING_INFO )
        print( self._EC_Controller.get_output_data(id,self._slave_set[id].PARSING_INFO) )


    def actual_position_in_rad(self,id):
        _slave_type                             = self._slave_set[id].DRIVER_TYPE
        if(_slave_type == 'GOLD_DRIVE' ):
            return  self._slave_set[id].ACTUAL_POS * 2*math.pi /                                                            \
                    (self._slave_set[id].GEAR_RATIO * self._slave_set[id].ENCODER_RES)
        if(_slave_type == 'EPOS4' ):
            return  self._slave_set[id].ACTUAL_POS * 2*math.pi /                                                            \
                    (self._slave_set[id].GEAR_RATIO * 4 * self._slave_set[id].ENCODER_RES)
        if(_slave_type == 'MAXPOS' ):
            return  self._slave_set[id].ACTUAL_POS * 2*math.pi /                                                            \
                    (self._slave_set[id].GEAR_RATIO * 4* self._slave_set[id].ENCODER_RES)
        if(_slave_type == 'MC5004_DRIVER' ):
            return  self._slave_set[id].ACTUAL_POS * 2*math.pi /                                                            \
                    (self._slave_set[id].GEAR_RATIO * 4* self._slave_set[id].ENCODER_RES)
        # return  self._slave_set[id].ACTUAL_POS

    def actual_velocity_in_rad(self,id):
        _slave_type                             = self._slave_set[id].DRIVER_TYPE
        if(_slave_type == 'GOLD_DRIVE' ):
            return  self._slave_set[id].ACTUAL_VELOCITY * 2*math.pi /                                                            \
                    (self._slave_set[id].GEAR_RATIO * self._slave_set[id].ENCODER_RES)


    def set_target_position(self,id,target_pos ):
        self._slave_set[id].TARGET_POS          = target_pos
        #enable new set point
        _slave_type                             = self._slave_set[id].DRIVER_TYPE
        if(_slave_type == 'GOLD_DRIVE' ):
            self._set_control_word_enable_motion(id)
        if(_slave_type != 'GOLD_DRIVE' ):
            self._set_control_word_enable_motion_head(id)

    def set_target_acce(self,id, target_acce):
        self._slave_set[id].TARGET_ACCE         = target_acce
    
    def set_target_deacce(self, id, target_deacce ):
        self._slave_set[id].TARGET_DEACCE       = target_deacce
    
    def set_target_velocity(self, id , target_velocity):
        self._slave_set[id].TARGET_VELOCITY     = target_velocity

    def _set_control_word_enable_motion_head(self,id):
        self._slave_set[id].CONTROL_WORD        = self._slave_set[id].CONTROL_WORD & 0xFFEF
        time.sleep(0.003)
        self._slave_set[id].CONTROL_WORD        = self._slave_set[id].CONTROL_WORD | 0x10
        time.sleep(0.003)
        # self._slave_set[id].CONTROL_WORD        = self._slave_set[id].CONTROL_WORD | 0x10          

    def _set_control_word_enable_motion(self,id):


        self._slave_set[id].CONTROL_WORD        = ( self._slave_set[id].CONTROL_WORD & 0xFECF ) | 0x200 #Set bit 9 to 1
        time.sleep(0.003)
        self._slave_set[id].CONTROL_WORD        = self._slave_set[id].CONTROL_WORD | 0x10 #Set bit 4 to 1
        time.sleep(0.003)
        self._slave_set[id].CONTROL_WORD        = self._slave_set[id].CONTROL_WORD & 0x20F   ###Reset bit 4 to 0


    def _enable_move_test(self,id):
        self._slave_set[id].CONTROL_WORD        = ( self._slave_set[id].CONTROL_WORD & 0xFECF ) | 0x200
        time.sleep(0.009)
        self._slave_set[id].CONTROL_WORD        = self._slave_set[id].CONTROL_WORD | 0x10
        time.sleep(0.009)

    def _update_target_reach_status(self,id):
        """this function is used to update the state of reach target position of motor"""
        if( self._slave_set[id].STATUS_WORD & 0x400 ):

            self._slave_set[id].TARGET_REACH    = True
        else:
            self._slave_set[id].TARGET_REACH    = False


        if( ( self._slave_set[id].STATUS_WORD & 0x1000) ):
            # print('bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb')
            self._temppp = 1 
        else :
            self._temppp = 2
    def get_temp(self,id):
        print(self._temppp,"\n")
        print(  bin( self._slave_set[id].STATUS_WORD) )

    def is_motor_reach_target(self,id):
        """this function return whether motor reaches the last set point or not"""
        return self._slave_set[id].TARGET_REACH         

    
    
    def set_brake(self,id, state):
        self._slave_set[id].DIGITAL_OUTPUT      = state

    def get_accelerate(self,id):
        print( self._EC_Controller.SDO_read( id, self._slave_set[id].DRIVE_DEF.ACCELERATE ) )

    def set_SDO_accelerate(self,id,acce):
        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.ACCELERATE , acce )

    def set_SDO_deaccelerate(self,id,deacce):
        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.DEACCELERATE ,deacce )

    def set_SDO_velocity(self,id,velocity):
        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.DEACCELERATE ,velocity )

    def get_status_word(self,id):
        return self._slave_set[id].STATUS_WORD

    def get_input_data_test(self, id):
        data_obtain_tuple                       = self._EC_Controller.get_input_data(id,'H')
        print(data_obtain_tuple)

    def read_sdo_test(self, id, object):
        print( self._EC_Controller.SDO_read(id , object ))


    def homing(self,id):
        if self._slave_set[id].BELONG_TO == 'hand':
            self._slave_set[id].MODE_OF_OPERATION   = self._slave_set[id].DRIVE_DEF.SERVO_MODE.homing_mode

            # print(EPOS4_CONTROL.HMM.COMMAND_PARAM.HOMING_METHOD)
            if (self._slave_set[id].ID == 13): #Thumb_2_Left

                self._EC_Controller.SDO_write(id,self._slave_set[id].DRIVE_DEF.HOMING_METHOD,27)  
                self._EC_Controller.SDO_write(id,self._slave_set[id].DRIVE_DEF.HOMING_METHOD_SPEED_SWITCH,100)
                self._EC_Controller.SDO_write(id,self._slave_set[id].DRIVE_DEF.HOMING_METHOD_SPEED_ZERO,100)
                self._EC_Controller.SDO_write(id,self._slave_set[id].DRIVE_DEF.HOMING_ACCE,1000)
                # self.sdo_write(EPOS4_CONTROL.HMM.COMMAND_PARAM.HOMING_SWITCH_SEARCH_SPEED)
                # self.sdo_write(EPOS4_CONTROL.HMM.COMMAND_PARAM.HOMING_ZERO_SEARCH_SPEED)
                # self.sdo_write(EPOS4_CONTROL.HMM.COMMAND_PARAM.HOMING_ACCELERATION)
            else:
                self._EC_Controller.SDO_write(id,self._slave_set[id].DRIVE_DEF.HOMING_METHOD,23)
                self._EC_Controller.SDO_write(id,self._slave_set[id].DRIVE_DEF.HOMING_METHOD_SPEED_SWITCH,1000)
                self._EC_Controller.SDO_write(id,self._slave_set[id].DRIVE_DEF.HOMING_METHOD_SPEED_ZERO,100)
                self._EC_Controller.SDO_write(id,self._slave_set[id].DRIVE_DEF.HOMING_ACCE,5000)
                # self.sdo_write(EPOS4_CONTROL.HMM.COMMAND_PARAM.HOMING_SWITCH_SEARCH_SPEED)
                # self.sdo_write(EPOS4_CONTROL.HMM.COMMAND_PARAM.HOMING_ZERO_SEARCH_SPEED)
                # self.sdo_write(EPOS4_CONTROL.HMM.COMMAND_PARAM.HOMING_ACCELERATION)

            self._slave_set[id].CONTROL_WORD        = 0b101111
            time.sleep(0.05)
            self._slave_set[id].CONTROL_WORD        = 0b111111
           
        elif self._slave_set[id].BELONG_TO == 'arm':
           self.move_to_position( id, 0 , 10 )

    def change_mode_position(self,id):
        self._slave_set[id].MODE_OF_OPERATION   = self._slave_set[id].DRIVE_DEF.SERVO_MODE.profile_position
        self._slave_set[id].CONTROL_WORD        = 0b101111
        time.sleep(0.05)
        self._slave_set[id].CONTROL_WORD        = 0b111111


    

#--------------SDO control-------------------

    def SDO_servo_on(self,id):
        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.CONTROL_WORD , 0x00)

        self._slave_set[id].CONTROL_WORD        = 0x00

        time.sleep(0.006)
        
        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.CONTROL_WORD , 0x80)

        self._slave_set[id].CONTROL_WORD        = 0x80
        time.sleep(0.006)

        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.CONTROL_WORD , 0b110)

        self._slave_set[id].CONTROL_WORD        = 0b110
        time.sleep(0.006)
        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.CONTROL_WORD , 0b10111)

        self._slave_set[id].CONTROL_WORD        = 0b10111
        time.sleep(0.006)

        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.CONTROL_WORD , 0b11111)

        self._slave_set[id].CONTROL_WORD        = 0b11111
        time.sleep(0.006)

        self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.MODE_OF_OPERATION , 1 )

        time.sleep(0.006)
        self._EC_Controller.SDO_write (id,[0x60C5,0X00,'UNSIGNED32'],32000)

        time.sleep(0.006)
        self._EC_Controller.SDO_write (id,[0x60C6,0X00,'UNSIGNED32'],32000)
        
        time.sleep(0.006)
        self._EC_Controller.SDO_write (id,[0x6086, 0x00, 'INTEGER16'],0)
        time.sleep(0.006)
        self._EC_Controller.SDO_write (id,[0x6080, 0x00, 'UNSIGNED32'],5800)
    
    def step_to_rad(self,step):
        _rad                                   = step*2*math.pi/( 4*512)
        return _rad

    def calculate_reach_time(self,id,start_time):
        while( self.is_motor_reach_target(id)!=True ):
            continue
        print(time.time() - start_time)


    def move_to_position(self,id,rad,time_to_reach):
        _current_pos                            = self.actual_position_in_rad(id)
        _current_velocity                       = self.actual_velocity_in_rad(id)
        if( abs(rad - _current_pos) < 0.00001 ):
            return 0
        if( (self._slave_set[id].DRIVER_TYPE == 'EPOS4')& (self._slave_set[id].GEAR_RATIO == 22)) or                \
            (self._slave_set[id].DRIVER_TYPE == 'MAXPOS'):
            
            target_position_degree = rad*180/math.pi
            target_position_round = (target_position_degree*5.6385/10)
            target_position_rad = target_position_round*math.pi*2
            _movement_profile                       = common.MovementProfileCalculator( _current_pos, 
                                                                                        target_position_rad ,time_to_reach,
                                                                                        self._slave_set[id].GEAR_RATIO,
                                                                                        self._slave_set[id].ENCODER_RES)
        else : 

            _movement_profile                       = common.MovementProfileCalculator( _current_pos, 
                                                                                        rad ,time_to_reach,
                                                                                        self._slave_set[id].GEAR_RATIO,
                                                                                        self._slave_set[id].ENCODER_RES)

        
        if( self._slave_set[id].DRIVER_TYPE == 'GOLD_DRIVE' ):
            _move_array                         = _movement_profile.get_movement_profile_in_encoder_counts()


        elif( self._slave_set[id].DRIVER_TYPE == 'EPOS4' ):
           _move_array                         = _movement_profile.get_movement_profile_in_encoder_pulse()

        elif( self._slave_set[id].DRIVER_TYPE == 'MAXPOS' ):
           _move_array                         = _movement_profile.MAXPOS_get_movement_profile_in_encoder_pulse()
            
        elif( self._slave_set[id].DRIVER_TYPE == 'MC5004_DRIVER' ):
           _move_array                         = _movement_profile.MC_get_movement_profile()
        


        print('current_pos: ', _current_pos)
        print('current velocity: ', _current_velocity)
        print('move_array: ', _move_array)
        self.set_target_acce(id, _move_array[2])

        self.set_target_deacce(id,_move_array[3] )

        self.set_target_velocity(id,_move_array[1] )

        #calculate time 
        # start_time                              = time.time()

        self.set_target_position(id,_move_array[0] )

        #print reach time
        # self.calculate_reach_time(id,start_time)
        # print('enddddddddddddddddddd')


    def move_to_position_blended(self,id,rad,time_to_reach):
        _current_pos                            = self.actual_position_in_rad(id)
        _current_velocity                       = self.actual_velocity_in_rad(id)

        if( abs(rad - _current_pos) < 0.000001 ):
            return 0
        if( (self._slave_set[id].DRIVER_TYPE == 'EPOS4')& (self._slave_set[id].GEAR_RATIO == 22)) or                \
            (self._slave_set[id].DRIVER_TYPE == 'MAXPOS'):
            
            target_position_degree = rad*180/math.pi
            target_position_round = (target_position_degree*5.6385/10)
            target_position_rad = target_position_round*math.pi*2
            _movement_profile                       = common.MovementProfileCalculator( _current_pos, 
                                                                                        target_position_rad ,time_to_reach,
                                                                                        self._slave_set[id].GEAR_RATIO,
                                                                                        self._slave_set[id].ENCODER_RES)
        else : 

            _movement_profile                       = common.MovementProfileCalculator( _current_pos, 
                                                                                        rad ,time_to_reach,
                                                                                        self._slave_set[id].GEAR_RATIO,
                                                                                        self._slave_set[id].ENCODER_RES,
                                                                                        )

        
        if( self._slave_set[id].DRIVER_TYPE == 'GOLD_DRIVE' ):
            _move_array                         = _movement_profile.get_movement_profile_in_encoder_counts()

        elif( self._slave_set[id].DRIVER_TYPE == 'EPOS4' ):
           _move_array                         = _movement_profile.get_movement_profile_in_encoder_pulse()

        elif( self._slave_set[id].DRIVER_TYPE == 'MAXPOS' ):
           _move_array                         = _movement_profile.MAXPOS_get_movement_profile_in_encoder_pulse()

        elif( self._slave_set[id].DRIVER_TYPE == 'MC5004_DRIVER' ):
           _move_array                         = _movement_profile.MC_get_movement_profile()
        

        print('current_pos: ', _current_pos)
        print('current velocity: ', _current_velocity)
        print('move_array: ', _move_array)
        # self.set_target_acce(id, _move_array[2])

        # self.set_target_deacce(id,_move_array[3] )

        self.set_target_acce(id, 10**9 )

        self.set_target_deacce(id,10**9 )

        self.set_target_velocity(id,400000 )

        #calculate time 
        # start_time                              = time.time()

        self.set_target_position(id,_move_array[0] )

        #print reach time
        # self.calculate_reach_time(id,start_time)
        # print('enddddddddddddddddddd')

    def move_to_position_test(self,id,speed_current,speed_final,rad,time_to_reach, start_pos_in_rad,acce_ratio,
                                                                        deacce_ratio,
                                                                        constant_ratio):
        _current_pos                            = start_pos_in_rad

        if( abs(rad - _current_pos) < 0.000001 ):
            return 0
    
        _movement_profile                       = common.MovementProfile_TEST( _current_pos, speed_current, speed_final,
                                                                        rad ,time_to_reach,
                                                                        self._slave_set[id].GEAR_RATIO,
                                                                        self._slave_set[id].ENCODER_RES,
                                                                        acce_ratio,
                                                                        deacce_ratio,
                                                                        constant_ratio
                                                                                            )

                                                                                            
        if( self._slave_set[id].DRIVER_TYPE == 'GOLD_DRIVE' ):
            _move_array                         = _movement_profile.get_movement_profile_in_encoder_counts()

        

        # print('current_pos: ', _current_pos)
        # print('current velocity: ', _current_velocity)
        # print('move_array: ', _move_array)
        # self.set_target_acce(id, _move_array[2])

        # self.set_target_deacce(id,_move_array[3] )

        print('pos',_move_array[0])
        print('vel',_move_array[1])
        print('acc',_move_array[2])
        print('deacc',_move_array[3])



        self.set_target_acce(id, _move_array[2] )

        self.set_target_deacce(id,_move_array[3] )

        self.set_target_velocity(id,_move_array[1] )

        #calculate time 
        # start_time                              = time.time()

        self.set_target_position(id,_move_array[0] )

        #print reach time
        # self.calculate_reach_time(id,start_time)
        # print('enddddddddddddddddddd')

    def move_to_position_blended_test_edge(self,id,current_pos,target_position,speed_current,speed_final,time_to_reach):
        _current_pos                            = current_pos
        self.access                             = 1
        # if(orientation == 1):
        # speed_final = 2*abs(target_position - current_pos)/time_to_reach - speed_current 
        # elif(orientation == 1)
        if speed_final < 0:
            print("speed final is negative")
        else:
            if id == 0 or id == 1:
                if(abs(target_position - _current_pos) < 0.0000096 ):
                    return 0
            if id == 2 or id == 3:
                if(abs(target_position - _current_pos) < 0.000012 ):
                    return 0
            if id == 4 or id == 5:
                if(abs(target_position - _current_pos) < 0.0000192 ):
                    return 0

                # target_position = _current_pos
            
            # print(">>>>>cur_pos,cur_speed,fin_speed,target_pos: ",_current_pos,", ",speed_current,", ",speed_final,", ",target_position)
            _movement_profile                       = common.MovementProfile_test_edge( _current_pos,
                                                                                        speed_current,
                                                                                        speed_final,
                                                                                        target_position,
                                                                                        self._slave_set[id].GEAR_RATIO,
                                                                                        self._slave_set[id].ENCODER_RES,
                                                                                        )


            # print('_current_pos: ', _current_pos)
            # print('speed_current: ', speed_current)
            # print('speed_final: ', speed_final)
            # print('target_position: ', target_position)
            # print('gear_ratio: ',self._slave_set[id].GEAR_RATIO)
            # print('res: ',self._slave_set[id].ENCODER_RES)
        
                        #     current_position,
                        #    speed_current,
                        #    speed_final,
                        #    target_position,
                        #    gear_ratio, 
                        #    motor_res 
            
            if( self._slave_set[id].DRIVER_TYPE == 'GOLD_DRIVE' ):
                _move_array                         = _movement_profile.get_movement_profile_in_encoder_counts()

            elif( self._slave_set[id].DRIVER_TYPE == 'EPOS4' ):
                    _move_array                         = _movement_profile.get_movement_profile_in_encoder_pulse()

            elif( self._slave_set[id].DRIVER_TYPE == 'MAXPOS' ):
                _move_array                         = _movement_profile.MAXPOS_get_movement_profile_in_encoder_pulse()

            elif( self._slave_set[id].DRIVER_TYPE == 'MC5004_DRIVER' ):
                _move_array                         = _movement_profile.MC_get_movement_profile()
            

            # print('current_pos: ', _current_pos)
            print('move_array: ', _move_array)


            self.set_target_acce(id, _move_array[2] )

            self.set_target_deacce(id,_move_array[3] )

            self.set_target_velocity(id,_move_array[1] )


            self.set_target_position(id,_move_array[0] )

            self.access = 0
            #print reach time
            # self.calculate_reach_time(id,start_time)
            # print('enddddddddddddddddddd')

    def move_to_position_ROS(self,id,rad,velocity,accelerate,deaccelerate):
        _current_pos                            = self.actual_position_in_rad(id)
        _current_velocity                       = self.actual_velocity_in_rad(id)
        time_to_reach                           = 0
        if( abs(rad - _current_pos) <= 0.000000001 ):
            return 0
        if( (self._slave_set[id].DRIVER_TYPE == 'EPOS4')& (self._slave_set[id].GEAR_RATIO == 22)) or                \
            (self._slave_set[id].DRIVER_TYPE == 'MAXPOS'):
            
            target_position_degree = rad*180/math.pi
            target_position_round = (target_position_degree*5.6385/10)
            target_position_rad = target_position_round*math.pi*2
            _movement_profile                       = common.MovementProfileCalculator( _current_pos, 
                                                                                        target_position_rad ,time_to_reach,
                                                                                        self._slave_set[id].GEAR_RATIO,
                                                                                        self._slave_set[id].ENCODER_RES)
        else : 

            _movement_profile                       = common.MovementProfileCalculator_ROS( _current_pos, 
                                                                                        rad ,time_to_reach,
                                                                                        self._slave_set[id].GEAR_RATIO,
                                                                                        self._slave_set[id].ENCODER_RES,
                                                                                        velocity,
                                                                                        accelerate,
                                                                                        deaccelerate
                                                                                        )

        
        if( self._slave_set[id].DRIVER_TYPE == 'GOLD_DRIVE' ):
            _move_array                         = _movement_profile.get_movement_profile_in_encoder_counts()

        elif( self._slave_set[id].DRIVER_TYPE == 'EPOS4' ):
            _move_array                         = _movement_profile.get_movement_profile_in_encoder_pulse()

        elif( self._slave_set[id].DRIVER_TYPE == 'MAXPOS' ):
            _move_array                         = _movement_profile.MAXPOS_get_movement_profile_in_encoder_pulse()

        elif( self._slave_set[id].DRIVER_TYPE == 'MC5004_DRIVER' ):
            _move_array                         = _movement_profile.MC_get_movement_profile()
        

        print('current_pos: ', _current_pos)
        print('current velocity: ', _current_velocity)
        print('move_array: ', _move_array)
        # self.set_target_acce(id, _move_array[2])

        # self.set_target_deacce(id,_move_array[3] )

        self.set_target_acce(id, _move_array[2] )
        print('acce: ', _move_array[2])
        self.set_target_deacce(id,_move_array[3] )
        print('deacce: ',_move_array[3])
        self.set_target_velocity(id,_move_array[1] )
        print('velocity: ', _move_array[1])
        #calculate time 
        # start_time                              = time.time()

        self.set_target_position(id,_move_array[0] )

        #print reach time
        # self.calculate_reach_time(id,start_time)
        # print('enddddddddddddddddddd')
        

    def SDO_go_to_pos( self,id,rad,time_to_reach ):
 
        if( id == 0 ):
            _target_pos                         = int( round(rad * 3.2 * 23 * 4 * 512 /(2* math.pi)  ) )
        elif( id == 1 ):
            _target_pos                         = int( round(rad * 40 * 23 * 4 * 512 /(2* math.pi) ) )


        if( id == 0 ):
            _target_pos_rad                     = rad * 3.2 * 23
        elif( id == 1 ):
            _target_pos_rad                     = rad * 40 * 23 

        # print('target_pos_rad',_target_pos_rad)

        _current_pos                            = self._EC_Controller.SDO_read( id, self._slave_set[id].DRIVE_DEF.POSITION_ACTUAL_VALUE )
        # print('_current_pos',_current_pos)
        _current_pos                            = self.step_to_rad(_current_pos)
        # print('_current_pos_rad',_current_pos)

        _movement_profile                       = common.MovementProfileCalculator(_current_pos, _target_pos_rad ,time_to_reach,10)

        _move_array                             = _movement_profile.MAXPOS_get_movement_profile_in_encoder_pulse()

        print(_move_array)
        print(int(_move_array[TARGET_ARRAY_POS]))
        print(int(_move_array[SPEED_ARRAY_POS]))
        print(int(_move_array[ACCE_ARRAY_POS]))
        print(int(_move_array[DEACCE_ARRAY_POS]))

        self._EC_Controller.SDO_write (id,[0x6086, 0x00, 'INTEGER16'],0)
        
        print('acce',int(_move_array[ACCE_ARRAY_POS]*60/(2*math.pi) ) )

        print('speed',int(_move_array[SPEED_ARRAY_POS]*60/(2*math.pi) ) )

        self.set_target_acce(id,int(_move_array[ACCE_ARRAY_POS]*60/(2*math.pi) ) )

        self.set_target_deacce(id,int(_move_array[DEACCE_ARRAY_POS]*60/(2*math.pi) ) )

        self.set_target_velocity(id,int(_move_array[SPEED_ARRAY_POS]*60/(2*math.pi) ) )

        self.set_target_position(id,_target_pos )


        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.TARGET_POSITION , _target_pos  )
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.ACCELERATE ,  int(_move_array[ACCE_ARRAY_POS]*30/(2*math.pi) ) )
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.DEACCELERATE ,int(_move_array[DEACCE_ARRAY_POS]*30/(2*math.pi) ) )
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.PROFILE_VELOCITY,int(_move_array[SPEED_ARRAY_POS]*60/(2*math.pi)) )        

        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.ACCELERATE ,  100 )
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.DEACCELERATE , 100 )
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.PROFILE_VELOCITY,5000 )        
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.PROFILE_VELOCITY,50 )        


        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.TARGET_POSITION , int(_move_array[TARGET_ARRAY_POS])  )
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.ACCELERATE ,  14019 )
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.DEACCELERATE ,14019 )
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.PROFILE_VELOCITY,168228 )  
        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.CONTROL_WORD , self._slave_set[id].CONTROL_WORD & 0xFFEF )

        # self._EC_Controller.SDO_write( id, self._slave_set[id].DRIVE_DEF.CONTROL_WORD , self._slave_set[id].CONTROL_WORD | 0x10 )

    def SDO_get_data(self,id):
        _current_pos                            = self._EC_Controller.SDO_read( id, self._slave_set[id].DRIVE_DEF.POSITION_ACTUAL_VALUE )
        print(_current_pos)


    def test_conculation(self):
        
        _movement_profile                       = common.MovementProfileCalculator(0, 10000 ,2)

        _movement_profile.get_movement_profile()


    def read_eeprom(self,id,eepromadd):        
        self._EC_Controller.read_eeprom(id,eepromadd)

    def write_eeprom(self,id,eepromadd, *data):
        self._EC_Controller.write_eeprom(id,eepromadd,struct.pack('<BB',*data) )


    def kill_thread(self):
        self._EC_Controller.stop_driver_communicate()
        self.join()

#elmo 
#position counts
#velocity count per sec
#accelerate count per sec ^2



#epos
#position counts
#velocity rpm
#accelerate rpm/s

#faulhaber
#position       
#velocity rpm
    def test_blend_set_point(self,id,p_1,p_2,p_3,p_4, time_1,time_2, time_3, time_4):

        self.move_to_position_blended(id,p_1,time_1 )
        self.move_to_position_blended(id,p_2,time_2 )
        self.move_to_position_blended(id,p_3,time_3 )
        self.move_to_position_blended(id,p_4,time_4 )


        print('Control Word: ',f'{self._slave_set[id].CONTROL_WORD:0>17b}')

