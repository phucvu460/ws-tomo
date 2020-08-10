import serial 
import serial.rs485
import yaml
import math
from threading import Thread
from collections import namedtuple
import time

_PRINT_LOG                                      = False



class SERIAL_MAIN_CONTROLLER:
    def __init__(self, ser_port='/dev/ttyUSB1' ):
        # """read config files"""
        # with open('./Config/Serial_config.yaml') as descrip_file:
        #     self.data = yaml.load(descrip_file, Loader=yaml.FullLoader)
        
        # #get serial port and baudrate
        # _serial_port                            = self.data["port"]
        # _serial_baudrate                        = self.data["baudrate"]
        # _slaves_count                           = self.data["slaves_info"]

        # print("Serial port: ", _serial_port)          
        # print("Serial baudrate: ", _serial_baudrate)

        # SlaveSet = namedtuple('SlaveSet', 'name joint_name ratio encoder_res belong_to')
        # self._expected_slave_mapping = {}

        # """Update expected slave mapping"""
        # for i in range(self.number_of_slave):
                
        #     self._expected_slave_mapping[i]     =  SlaveSet( self.data['slaves_info']['slave_'+str(i)]['driver_brand'],
        #                                                      self.data['slaves_info']['slave_'+str(i)]['joint_name']  ,
        #                                                      self.data['slaves_info']['slave_'+str(i)]['ratio']       ,
        #                                                      self.data['slaves_info']['slave_'+str(i)]['encoder_res'] ,
        #                                                      self.data['slaves_info']['slave_'+str(i)]['belong_to']      )

        #     if(_PRINT_LOG == True):
        #         print(  self._expected_slave_mapping[i]  )


        
        self.ser                                =serial.rs485.RS485(port=ser_port, baudrate=57600)
        self.ser.rs485_mode                          = serial.rs485.RS485Settings()
        self.ser.write('HELLO'.encode('utf-8'))
        self.ser.write('Y1N0p363.82d1.24x0.17i7.22Ks#'.encode('utf-8'))
        time.sleep(0.1)
        self.ser.write('Y1N0p363.82d1.24x0.17i7.22Ks#'.encode('utf-8'))
        time.sleep(0.1)
        self.ser.write('Y1N0p363.82d1.24x0.17i7.22Ks#'.encode('utf-8'))
        time.sleep(0.1)

        print("Serial port is initialized!")



    def process_sending_data(self,CMD,motor_id,position=0x00, velocity=0x00, accelerate=0x00):
        """ This function is used to parsing sending data into the protocol which refer at
         https://docs.google.com/spreadsheets/d/1fo4xZucQrdholPErPqckLzpDAIB9GCyrwqWlOyRX5dc/edit?ts=5e607897#gid=0 """

         
        _CMD                                    = CMD
        _address                                = motor_id
        #position parsing
        _position_byte_0_LSB                    = (position >> 0 ) & 0x000000ff
        _position_byte_1                        = (position >> 8 ) & 0x000000ff
        _position_byte_2                        = (position >> 16) & 0x000000ff
        _position_byte_3_MSB                    = (position >> 24) & 0x000000ff
        #velocity
        _velocity_byte_0_LSB                    = (velocity >> 0 ) & 0x00ff
        _velocity_byte_1_MSB                    = (velocity >> 8 ) & 0x00ff
        #accelerate
        _accelerate_byte_0_LSB                  = (accelerate >> 0 ) & 0x00ff
        _accelerate_byte_1_MSB                  = (accelerate >> 8 ) & 0x00ff

        #check sum
        _check_sum                              = _CMD + _accelerate_byte_1_MSB
        _check_sum_byte_0_LSB                   = (_check_sum >> 0 ) & 0x00ff
        _check_sum_byte_0_MSB                   = (_check_sum >> 8 ) & 0x00ff

        self._Serial.write([ _CMD                   ,
                             _address               ,
                             _position_byte_0_LSB   ,
                             _position_byte_1       ,
                             _position_byte_2       ,
                             _position_byte_3_MSB   ,
                             _velocity_byte_0_LSB   ,
                             _velocity_byte_1_MSB   ,
                             _accelerate_byte_0_LSB ,
                             _accelerate_byte_1_MSB ,
                             _check_sum_byte_0_LSB  ,
                             _check_sum_byte_0_MSB   ])
        if(_PRINT_LOG == True):
                print(  [    _CMD                   ,
                             _address               ,
                             _position_byte_0_LSB   ,
                             _position_byte_1       ,
                             _position_byte_2       ,
                             _position_byte_3_MSB   ,
                             _velocity_byte_0_LSB   ,
                             _velocity_byte_1_MSB   ,
                             _accelerate_byte_0_LSB ,
                             _accelerate_byte_1_MSB ,
                             _check_sum_byte_0_LSB  ,
                             _check_sum_byte_0_MSB   ]  )

    def get_input_data(self):
        pass

    def set_output_data(self):
        pass






    def process_serial_output_data(self,motor,pos,vel,acc,syns = False ):
        if(syns == True):
            output                              = 'Y1'+'N'+str(motor)+'P'+str(pos)+'V'+str(vel)+'A'+str(acc)+'s#'
        else:
            output                              = 'Y1'+'N'+str(motor)+'P'+str(pos)+'V'+str(vel)+'A'+str(acc)+'#'
        return output


    def set_motor_data(self,motor,pos,vel,acc,syn = False):
        _ser_data                               = self.process_serial_output_data(motor,pos,vel,acc,syn)
        if(_PRINT_LOG == True):
            print(_ser_data)
            print(_ser_data.encode('utf-8'))
        self.ser.write( _ser_data.encode('utf-8') )
        time.sleep(0.001)

    def servo_run(self):
        self.ser.write('Y1N0s#'.encode('utf-8'))
        time.sleep(0.001)


    def hand_run_temp(self):
        self.set_motor_data(3, 0   ,1000,3000)
        self.set_motor_data(4,-4200,1000,3000)
        self.set_motor_data(5,-4200,1000,3000)
        self.set_motor_data(6,-4200,1000,3000)
        self.servo_run()

        time.sleep(7)


        self.set_motor_data(3,-4200,1000,3000)
        self.set_motor_data(4, 0   ,1000,3000)
        self.set_motor_data(5,-4200,1000,3000)
        self.set_motor_data(6,-4200,1000,3000)
        self.servo_run()

        time.sleep(7)

        self.set_motor_data(3,-4200,1000,3000)
        self.set_motor_data(4,-4200,1000,3000)
        self.set_motor_data(5, 0   ,1000,3000)
        self.set_motor_data(6,-4200,1000,3000)
        self.servo_run()

        time.sleep(7)


        self.set_motor_data(3,-4200,1000,3000)
        self.set_motor_data(4,-4200,1000,3000)
        self.set_motor_data(5,-4200,1000,3000)
        self.set_motor_data(6, 0   ,1000,3000)
        self.servo_run()


        time.sleep(7)



        self.set_motor_data(3,0,1000,3000)
        self.set_motor_data(4,0,1000,3000)
        self.set_motor_data(5,0,1000,3000)
        self.set_motor_data(6,0,1000,3000)
        self.servo_run()


    