import numpy as np
from math import pi
import cstruct
import struct
import yaml

ETHERCAT_CONFIG_FILE_NAME = 'etherCAT_communication.yaml'
MECHANIC_CONFIG_FILE_NAME = 'mechanical_description.yaml'
CONTROL_PROFILE_FILE_NAME = 'control_profile.yaml'

'''C-type Definition'''
c_type = {
    'INTEGER8': '<b',
    'UNSIGNED8': '<B',
    'INTEGER16': '<h',
    'UNSIGNED16': '<H',
    'INTEGER32': '<i',
    'UNSIGNED32': '<I',
    'STRING':'<s'
}

c_type_size = {
    'INTEGER8': 1,
    'UNSIGNED8': 1,
    'INTEGER16': 2,
    'UNSIGNED16': 2,
    'INTEGER32': 4,
    'UNSIGNED32': 4,
    'STRING': 0
}


control_word_bit                                = { 'bit_0'  : 0x0001 , #Switch on
                                                    'bit_1'  : 0x0002 , #Enable voltage
                                                    'bit_2'  : 0x0004 , #Quick stop
                                                    'bit_3'  : 0x0008 , #Enable operation
                                                    
                                                    'bit_4'  : 0x0001 , 
                                                    'bit_5'  : 0x0002 , #Fault reset
                                                    'bit_6'  : 0x0004 , #abs/rel
                                                    'bit_7'  : 0x0008 ,
                                                    
                                                    'bit_8'  : 0x0001 , 
                                                    'bit_9'  : 0x0002 , 
                                                    'bit_10' : 0x0004 ,
                                                    'bit_11' : 0x0008 ,
                                                    
                                                    'bit_12' : 0x0001 , 
                                                    'bit_13' : 0x0002 ,
                                                    'bit_14' : 0x0004 , 
                                                    'bit_15' : 0x0008    }

status_word_bit                                 = { 'bit_0'  : 0x0001 , #Ready to Switch on
                                                    'bit_1'  : 0x0002 , #Switched on
                                                    'bit_2'  : 0x0004 , #operation enabled
                                                    'bit_3'  : 0x0008 , #fault
                                                    
                                                    'bit_4'  : 0x0010 , #voltage enable
                                                    'bit_5'  : 0x0020 , #quick stop
                                                    'bit_6'  : 0x0040 , #Switch on disable
                                                    'bit_7'  : 0x0080 , #warning
                                                    
                                                    'bit_8'  : 0x0100 , 
                                                    'bit_9'  : 0x0200 , #operation specific 
                                                    'bit_10' : 0x0400 , #internal limit active
                                                    'bit_11' : 0x0800 , #Operating mode-specific
                                                    
                                                    'bit_12' : 0x1000 , #Operating mode-specific 
                                                    'bit_13' : 0x2000 , #Operating mode-specific
                                                    'bit_14' : 0x4000 , #Operating mode-specifics
                                                    'bit_15' : 0x8000    } 


def c_type_convert(type):
    return c_type[type]


def byte_number_from_ctype(type):
    return c_type_size[type]


"""Bitwise Operators"""


def bit_is_set(val, bit):
    return True if((val >> bit) & 1) else False


def bit_is_clear(val, bit):
    return False if((val >> bit) & 1) else True


def bv(bit):
    return 1 << bit


def set_bit(val, bit):
    return val | bv(bit)


def clear_bit(val, bit):
    return val & ~bv(bit)


class BIT:
    SET = 1
    CLEAR = 0


class ACCESS_TYPE:
    """SLAVE PDO ACCESS TYPE"""
    ONE_TO_ALL = -1
    ONE_TO_ONE = -2


class SDO_ADDR:
    """ADDRESS STRUCTURE"""
    INDEX = 0
    SUB_INDEX = 1
    DATA_TYPE = 2
    VALUE = 3

class MOVE_PROFILE:
    """MOVEMENT PROFILE STRUCTURE"""
    TARGET_POS = 0
    SPEED = 1
    ACCEL = 2
    DECEL = 3


class MECHANICAL_CONVERTER:
    """Mechanical Converter"""

    def __init__(self, part, joint_name):

        """Reading from config file from slave_position"""
        with open('./config/' + MECHANIC_CONFIG_FILE_NAME) as desc_file:
            mechanical_desc = yaml.load(desc_file.read())
        # , Loader = yaml.FullLoader
        self.joint_part = part
        self.joint_name = joint_name

        part_desc = mechanical_desc[self.joint_part]
        joint_desc = part_desc[self.joint_name]

        self.gear_ratio = joint_desc['gear_ratio']
        self.pulse_per_revolution = joint_desc['pulse_per_revolution']

        self.pulse_per_2rad = self.pulse_per_revolution/(2*pi)

        self.rads_to_rpm_ratio = 30/pi

    def joint_rads_to_motor_rpm(self, value):

        after_gear = value

        before_gear = after_gear * self.gear_ratio

        motor_rpm = self.rads_to_rpm(before_gear)
        return int(motor_rpm)

    def rads_to_rpm(self, value):
        return value * self.rads_to_rpm_ratio

    def rpm_to_rads(self, value):
        return value / self.rads_to_rpm_ratio

    def pulse_to_rad(self, pulse):
        return pulse / self.pulse_per_2rad

    def rad_to_pulse(self, degree):
        return int(degree * self.pulse_per_2rad)

    # def get_current_position(self):
    #     return self.pulse_to_rad(self.in_param.PositionActualValue)
    #
    # def set_current_position(self):

class MovementProfileCalculator:
    """Calculator Function"""

    def __init__(self, current_position, target_position, _travel_time,gear_ratio, motor_res):
        self._encoder_res                       = motor_res
        self._gear_ratio                        = gear_ratio
        self._current_position                  = np.array(current_position*self._gear_ratio)
        self._target_position                   = np.array(target_position*self._gear_ratio)
        self._travel_time                       = _travel_time
        self._distance                          = abs(self._target_position - self._current_position)

        self._speed                             = 0  # rad/s
        self._accelerate                        = 0  # rad/s^2
        self._deaccelerate                      = 0  # rad/s^2

        self._angleWhenAcce                     = 0
        self._angleWhenConstantSpeed            = 0
        self._angleWhenDeacce                   = 0

        self._timeAcce                          = 0
        self._timeDeacce                        = 0
        self._timeConstantSpeed                 = 0
        # you can adjust the ratio of each period here
        self._timeAcceRatio                     = 0.30  # of _timeReachTarget
        self._timeDeacceRatio                   = 0.30  # of _timeReachTarget
        self._timeConstantSpeedRatio            = 0.40  # of _timeReachTarget

    def _calculate_time_period(self):
        self._timeAcce                          = self._timeAcceRatio * self._travel_time

        self._timeDeacce                        = self._timeDeacceRatio * self._travel_time

        self._timeConstantSpeed                 = self._timeConstantSpeedRatio * self._travel_time

    def _calculate_speed(self):
        self._speed                             = (1 / 2 * self._timeAcce +
                                                   self._timeConstantSpeed +
                                                   self._timeDeacce)

        self._speed                             = self._speed * (self._travel_time -
                                                                 self._timeConstantSpeed -
                                                                 self._timeAcce)

        self._speed                             = self._speed - 1 / 2 * self._timeDeacce ** 2

        self._speed                             = self._distance / self._speed

        self._speed                             = self._speed * (self._travel_time -
                                                                 self._timeConstantSpeed -
                                                                 self._timeAcce)

    def _calculate_angle_of_each_period(self):
        # calculating the angle when rotation in Acceleration period
        self._angleWhenAcce = 1 / 2 * self._speed * self._timeAcce
        # calculating the angle when rotation in constant speed period
        self._angleWhenConstantSpeed = self._speed * self._timeConstantSpeed
        # calculating angle when rotation in deaccelation period
        self._angleWhenDeacce = self._speed * self._timeDeacce

        self._angleWhenDeacce = -1 / 2 * self._speed * (self._timeDeacce ** 2)

        self._angleWhenDeacce = self._angleWhenDeacce / \
                                (self._travel_time -
                                 self._timeConstantSpeed -
                                 self._timeAcce)

        self._angleWhenDeacce = self._angleWhenDeacce + \
                                (self._speed * self._timeDeacce)

    def _calculate_acce_dece(self):
        """
            Calculate acceleration and deceleration
        """
        self._accelerate = 2 * self._angleWhenAcce / (self._timeAcce ** 2)
        self._deaccelerate = abs(self._angleWhenDeacce - self._speed *
                                 self._timeDeacce)
        self._deaccelerate = 2 * self._deaccelerate / (self._timeDeacce ** 2)

    def get_movement_profile_in_rad(self):
        """@brief: This function will return speed, acceleration and deaccelerate calculate from
                    the input data
           @param:
                   - targetPosition: The expected position of the joint in radian
                   - time          : The expected period (in second) the the joint gets
                                     its expected position

           @return:
                    - speed        : The speed in radian/sec
                    - accelerate   : The accelerate in radian/sec^2
                    - deaccelerate : The deaccelerate in radian/sec^2 """

        # calculating the time of each period of rotation (start -> reach set speed -> stop)
        self._calculate_time_period()

        self._calculate_speed()

        self._calculate_angle_of_each_period()

        self._calculate_acce_dece()

        # return np.transpose(
        #     np.vstack([self._target_position,
        #                self._speed,
        #                self._accelerate,
        #                self._deaccelerate])).tolist()
        return np.array([self._target_position,self._speed,self._accelerate,self._deaccelerate])
        
        # return 

    def get_movement_profile_in_encoder_counts(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( self._target_position * self._encoder_res / (2*pi) )
        speed_in_counts                         = int( self._speed * self._encoder_res / (2*pi) )
        accelerate_in_counts                    = int( self._accelerate * self._encoder_res / (2*pi) )
        deaccelerate_in_counts                  = int( self._deaccelerate * self._encoder_res / (2*pi) )

        return np.array([target_position_in_counts,speed_in_counts,accelerate_in_counts,deaccelerate_in_counts])

    def get_movement_profile_in_encoder_pulse(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position * self._encoder_res*4)/ (2*pi) )
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( (self._accelerate)*60/ ((2*pi)) )
        deaccelerate_in_rpm_per_s               = int( (self._deaccelerate)*60 / ((2*pi)) )


        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])

    def MAXPOS_get_movement_profile_in_encoder_pulse(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position* self._encoder_res*4)/ (2*pi))
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( (self._accelerate)*60/ ((2*pi)) )
        deaccelerate_in_rpm_per_s               = int( (self._deaccelerate)*60 / ((2*pi)) )

        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])
    
    def MC_get_movement_profile(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position* self._encoder_res*4)/ (2*pi))
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( self._accelerate * self._encoder_res *4 / (2*pi) )
        deaccelerate_in_rpm_per_s               = int( self._deaccelerate * self._encoder_res *4 / (2*pi) )

        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])

    def _rad_to_inc(self,rad):
        return rad * self._encoder_res / (2*pi)

    def _rad_per_Sec_to_RPM(self,rad_per_Sec):
        return rad_per_Sec * 60 /(2*pi)

        

# class data_parsing:

class MovementProfileCalculator_ROS:
    """Calculator Function"""

    def __init__(self, current_position, target_position, _travel_time,gear_ratio, motor_res,velocity,accelerate,deaccelerate):
        self._encoder_res                       = motor_res
        self._gear_ratio                        = gear_ratio
        self._current_position                  = np.array(current_position*self._gear_ratio)
        self._target_position                   = np.array(target_position*self._gear_ratio)
        self._travel_time                       = _travel_time
        self._distance                          = abs(self._target_position - self._current_position)

        self._speed                             = velocity*self._gear_ratio  # rad/s
        self._accelerate                        = accelerate*self._gear_ratio  # rad/s^2
        self._deaccelerate                      = deaccelerate*self._gear_ratio  # rad/s^2

        # self._angleWhenAcce                     = 0
        # self._angleWhenConstantSpeed            = 0
        # self._angleWhenDeacce                   = 0

        # self._timeAcce                          = 0
        # self._timeDeacce                        = 0
        # self._timeConstantSpeed                 = 0
        # you can adjust the ratio of each period here
        # self._timeAcceRatio                     = 0.30  # of _timeReachTarget
        # self._timeDeacceRatio                   = 0.30  # of _timeReachTarget
        # self._timeConstantSpeedRatio            = 0.40  # of _timeReachTarget

    # def _calculate_time_period(self):
    #     self._timeAcce                          = self._timeAcceRatio * self._travel_time

    #     self._timeDeacce                        = self._timeDeacceRatio * self._travel_time

    #     self._timeConstantSpeed                 = self._timeConstantSpeedRatio * self._travel_time

    # def _calculate_speed(self):
    #     self._speed                             = (1 / 2 * self._timeAcce +
    #                                                self._timeConstantSpeed +
    #                                                self._timeDeacce)

    #     self._speed                             = self._speed * (self._travel_time -
    #                                                              self._timeConstantSpeed -
    #                                                              self._timeAcce)

    #     self._speed                             = self._speed - 1 / 2 * self._timeDeacce ** 2

    #     self._speed                             = self._distance / self._speed

    #     self._speed                             = self._speed * (self._travel_time -
    #                                                              self._timeConstantSpeed -
    #                                                              self._timeAcce)

    # def _calculate_angle_of_each_period(self):
    #     # calculating the angle when rotation in Acceleration period
    #     self._angleWhenAcce = 1 / 2 * self._speed * self._timeAcce
    #     # calculating the angle when rotation in constant speed period
    #     self._angleWhenConstantSpeed = self._speed * self._timeConstantSpeed
    #     # calculating angle when rotation in deaccelation period
    #     self._angleWhenDeacce = self._speed * self._timeDeacce

    #     self._angleWhenDeacce = -1 / 2 * self._speed * (self._timeDeacce ** 2)

    #     self._angleWhenDeacce = self._angleWhenDeacce / \
    #                             (self._travel_time -
    #                              self._timeConstantSpeed -
    #                              self._timeAcce)

    #     self._angleWhenDeacce = self._angleWhenDeacce + \
    #                             (self._speed * self._timeDeacce)

    # def _calculate_acce_dece(self):
    #     """
    #         Calculate acceleration and deceleration
    #     """
    #     self._accelerate = 2 * self._angleWhenAcce / (self._timeAcce ** 2)
    #     self._deaccelerate = abs(self._angleWhenDeacce - self._speed *
    #                              self._timeDeacce)
    #     self._deaccelerate = 2 * self._deaccelerate / (self._timeDeacce ** 2)

    def get_movement_profile_in_rad(self):
        """@brief: This function will return speed, acceleration and deaccelerate calculate from
                    the input data
           @param:
                   - targetPosition: The expected position of the joint in radian
                   - time          : The expected period (in second) the the joint gets
                                     its expected position

           @return:
                    - speed        : The speed in radian/sec
                    - accelerate   : The accelerate in radian/sec^2
                    - deaccelerate : The deaccelerate in radian/sec^2 """

        # calculating the time of each period of rotation (start -> reach set speed -> stop)
        # self._calculate_time_period()

        # self._calculate_speed()

        # self._calculate_angle_of_each_period()

        # self._calculate_acce_dece()

        # return np.transpose(
        #     np.vstack([self._target_position,
        #                self._speed,
        #                self._accelerate,
        #                self._deaccelerate])).tolist()
        return np.array([self._target_position,self._speed,self._accelerate,self._deaccelerate])
        
        # return 

    def get_movement_profile_in_encoder_counts(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( self._target_position * self._encoder_res / (2*pi) )
        speed_in_counts                         = int( self._speed * self._encoder_res / (2*pi) )
        accelerate_in_counts                    = int( self._accelerate * self._encoder_res / (2*pi) )
        deaccelerate_in_counts                  = int( self._deaccelerate * self._encoder_res / (2*pi) )

        return np.array([target_position_in_counts,speed_in_counts,accelerate_in_counts,deaccelerate_in_counts])

    def get_movement_profile_in_encoder_pulse(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position * self._encoder_res*4)/ (2*pi) )
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( (self._accelerate)*60/ ((2*pi)) )
        deaccelerate_in_rpm_per_s               = int( (self._deaccelerate)*60 / ((2*pi)) )


        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])

    def MAXPOS_get_movement_profile_in_encoder_pulse(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position* self._encoder_res*4)/ (2*pi))
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( (self._accelerate)*60/ ((2*pi)) )
        deaccelerate_in_rpm_per_s               = int( (self._deaccelerate)*60 / ((2*pi)) )

        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])
    
    def MC_get_movement_profile(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position* self._encoder_res*4)/ (2*pi))
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( self._accelerate * self._encoder_res *4 / (2*pi) )
        deaccelerate_in_rpm_per_s               = int( self._deaccelerate * self._encoder_res *4 / (2*pi) )

        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])

    def _rad_to_inc(self,rad):
        return rad * self._encoder_res / (2*pi)

    def _rad_per_Sec_to_RPM(self,rad_per_Sec):
        return rad_per_Sec * 60 /(2*pi)

        

# class data_parsing:
















#testttt
class MovementProfile_TEST:
    """Calculator Function"""

    def __init__(self, current_position,
                       speed_current,
                       speed_final,
                       target_position,
                       _travel_time,
                       gear_ratio, 
                       motor_res,
                       acce_ratio,
                       deacce_ratio,
                       constant_ratio            ):

        self._encoder_res                       = motor_res
        self._gear_ratio                        = gear_ratio
        self._current_position                  = np.array(current_position*self._gear_ratio)
        self._target_position                   = np.array(target_position*self._gear_ratio)
        self._travel_time                       = _travel_time
        self._distance                          = abs(self._target_position - self._current_position)
        self._speed_current                     = speed_current
        self._speed_final                       = speed_final

        self._speed                             = 0  # rad/s
        self._accelerate                        = 0  # rad/s^2
        self._deaccelerate                      = 0  # rad/s^2

        self._angleWhenAcce                     = 0
        self._angleWhenConstantSpeed            = 0
        self._angleWhenDeacce                   = 0

        self._timeAcce                          = 0
        self._timeDeacce                        = 0
        self._timeConstantSpeed                 = 0
        # you can adjust the ratio of each period here
        self._timeAcceRatio                     = acce_ratio  # of _timeReachTarget
        self._timeDeacceRatio                   = deacce_ratio  # of _timeReachTarget
        self._timeConstantSpeedRatio            = constant_ratio  # of _timeReachTarget

    def _calculate_time_period(self):
        self._timeAcce                          = self._timeAcceRatio * self._travel_time

        self._timeDeacce                        = self._timeDeacceRatio * self._travel_time

        self._timeConstantSpeed                 = self._timeConstantSpeedRatio * self._travel_time

    def _calculate_speed(self):
        self._speed                             = self._speed_current + self._timeAcce * self._accelerate

    # def _calculate_angle_of_each_period(self):
    #     # calculating the angle when rotation in Acceleration period
    #     self._angleWhenAcce = 1 / 2  self._speed  self._timeAcce
    #     # calculating the angle when rotation in constant speed period
    #     self._angleWhenConstantSpeed = self._speed * self._timeConstantSpeed
    #     # calculating angle when rotation in deaccelation period
    #     self._angleWhenDeacce = self._speed * self._timeDeacce

    #     self._angleWhenDeacce = -1 / 2  self._speed  (self._timeDeacce ** 2)

    #     self._angleWhenDeacce = self._angleWhenDeacce / \
    #                             (self._travel_time -
    #                              self._timeConstantSpeed -
    #                              self._timeAcce)

    #     self._angleWhenDeacce = self._angleWhenDeacce + \
    #                             (self._speed * self._timeDeacce)

    def _calculate_acce(self):
        """
            Calculate acceleration 
        """
        self._accelerate = self._distance
        self._accelerate = self._accelerate - self._speed_current*( self._travel_time - 1/2*self._timeDeacce)
        self._accelerate = self._accelerate - 1/2*self._speed_final*self._timeDeacce
        
        denominator      =  1/2*self._timeAcce**2 + self._timeAcce*self._timeConstantSpeed + \
                            1/2*self._timeAcce*self._timeDeacce

        self._accelerate = self._accelerate / denominator


    def _calculate_deacce(self):
        """
            Calculate deceleration
        """
        if(self._timeDeacce != 0):
            self._deaccelerate = ( self._speed - self._speed_final )/self._timeDeacce
        else:
            self._deaccelerate = 10**9
    def get_movement_profile_in_rad(self):
        """@brief: This function will return speed, acceleration and deaccelerate calculate from
                    the input data
           @param:
                   - targetPosition: The expected position of the joint in radian
                   - time          : The expected period (in second) the the joint gets
                                     its expected position

           @return:
                    - speed        : The speed in radian/sec
                    - accelerate   : The accelerate in radian/sec^2
                    - deaccelerate : The deaccelerate in radian/sec^2 """

        # calculating the time of each period of rotation (start -> reach set speed -> stop)
        self._calculate_time_period()

        self._calculate_acce()

        self._calculate_speed()

        self._calculate_deacce()

        print(self._target_position,self._speed,self._accelerate,self._deaccelerate)

        # return np.transpose(
        #     np.vstack([self._target_position,
        #                self._speed,
        #                self._accelerate,
        #                self._deaccelerate])).tolist()
        return np.array([self._target_position,self._speed,self._accelerate,self._deaccelerate])
        
        # return 

    def get_movement_profile_in_encoder_counts(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( self._target_position * self._encoder_res / (2*pi) )
        speed_in_counts                         = int( self._speed * self._encoder_res / (2*pi) )
        accelerate_in_counts                    = int( self._accelerate * self._encoder_res / (2*pi) )
        deaccelerate_in_counts                  = int( self._deaccelerate * self._encoder_res / (2*pi) )

        return np.array([target_position_in_counts,speed_in_counts,accelerate_in_counts,deaccelerate_in_counts])

    def get_movement_profile_in_encoder_pulse(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position * self._encoder_res*4)/ (2*pi) )
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( (self._accelerate)*60/ ((2*pi)) )
        deaccelerate_in_rpm_per_s               = int( (self._deaccelerate)*60 / ((2*pi)) )


        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])

    def MAXPOS_get_movement_profile_in_encoder_pulse(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position* self._encoder_res*4)/ (2*pi))
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( (self._accelerate)*60/ ((2*pi)) )
        deaccelerate_in_rpm_per_s               = int( (self._deaccelerate)*60 / ((2*pi)) )

        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])
    
    def MC_get_movement_profile(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position* self._encoder_res*4)/ (2*pi))
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( self._accelerate*self._encoder_res*4) / (2*pi) 
        deaccelerate_in_rpm_per_s               = int( self._deaccelerate*self._encoder_res*4) / (2*pi) 

        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])

    def _rad_to_inc(self,rad):
        return rad * self._encoder_res / (2*pi)

    def _rad_per_Sec_to_RPM(self,rad_per_Sec):
        return rad_per_SecMovementProfile_TEST * 60 /(2*pi)

class MovementProfile_test_edge:
    """Calculator Function"""

    def __init__(self, current_position,
                    speed_current,
                    speed_final,
                    target_position,
                    gear_ratio, 
                    motor_res            ):
        self._encoder_res                       = motor_res
        self._gear_ratio                        = gear_ratio
        self._current_position                  = np.array(current_position*self._gear_ratio)
        self._target_position                   = np.array(target_position*self._gear_ratio)
        self._travel_time                       = 0
        self._distance                          = abs(self._target_position - self._current_position)
        self._speed_current                     = speed_current*self._gear_ratio
        self._speed_final                       = speed_final*self._gear_ratio

        self._speed                             = 0  # rad/s
        self._accelerate                        = 0  # rad/s^2
        self._deaccelerate                      = 0  # rad/s^2



        self._timeAcce                          = 0
        self._timeDeacce                        = 0
        self._timeConstantSpeed                 = 0
        # you can adjust the ratio of each period here

    def _calculate_time_period(self):
        self._travel_time               = 2 * self._distance /(self._speed_current + self._speed_final) 
        # print("time to reach: ",self._travel_time)
        

    def _calculate_speed_output(self):
        if(self._accelerate <= 10**(-4) and 
        self._accelerate >= -10**(-4)        ):
            self._speed                 = self._speed_current
            self._accelerate            = 10**9
            self._deaccelerate          = 10**9
        elif(self._accelerate > 10**(-4) ):
            self._speed                 = self._speed_final
            self._accelerate            = abs(self._accelerate)
            self._deaccelerate          = 10**9
        else:
            self._speed                 = self._speed_current/2
            self._deaccelerate          = abs(self._accelerate)
            self._accelerate            = 10**9
        
    # def _calculate_angle_of_each_period(self):
    #     # calculating the angle when rotation in Acceleration period
    #     self._angleWhenAcce = 1 / 2 * self._speed * self._timeAcce
    #     # calculating the angle when rotation in constant speed period
    #     self._angleWhenConstantSpeed = self._speed * self._timeConstantSpeed
    #     # calculating angle when rotation in deaccelation period
    #     self._angleWhenDeacce = self._speed * self._timeDeacce

    #     self._angleWhenDeacce = -1 / 2 * self._speed * (self._timeDeacce ** 2)

    #     self._angleWhenDeacce = self._angleWhenDeacce / \
    #                             (self._travel_time -
    #                              self._timeConstantSpeed -
    #                              self._timeAcce)

    #     self._angleWhenDeacce = self._angleWhenDeacce + \
    #                             (self._speed * self._timeDeacce)

    def _calculate_acce(self):
        """
            Calculate acceleration 
        """
        self._accelerate                = ( self._speed_final - self._speed_current )/ self._travel_time
        # print("accleeeeeee; ",self._accelerate)
    def _calculate_deacce(self):
        """
            Calculate deceleration
        """
        pass
        # self._deaccelerate = ( self._speed - self._speed_final )/self._timeDeacce

    def get_movement_profile_in_rad(self):
        """@brief: This function will return speed, acceleration and deaccelerate calculate from
                    the input data
        @param:
                - targetPosition: The expected position of the joint in radian
                - time          : The expected period (in second) the the joint gets
                                    its expected position

        @return:
                    - speed        : The speed in radian/sec
                    - accelerate   : The accelerate in radian/sec^2
                    - deaccelerate : The deaccelerate in radian/sec^2 """

        # calculating the time of each period of rotation (start -> reach set speed -> stop)
        self._calculate_time_period()

        self._calculate_acce()

        self._calculate_speed_output()





        # return np.transpose(
        #     np.vstack([self._target_position,
        #                self._speed,
        #                self._accelerate,
        #                self._deaccelerate])).tolist()
        return np.array([self._target_position,self._speed,self._accelerate,self._deaccelerate])
        
        # return 

    def get_movement_profile_in_encoder_counts(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( self._target_position * self._encoder_res / (2*pi) )
        speed_in_counts                         = int( self._speed * self._encoder_res / (2*pi) )
        
        if(self._accelerate < 10000000):
            accelerate_in_counts                = int( self._accelerate * self._encoder_res / (2*pi) )
        else:
            accelerate_in_counts                = 2**31 - 1 #10**9

        if(self._deaccelerate < 10000000):
            deaccelerate_in_counts              = int( self._deaccelerate * self._encoder_res / (2*pi) )
        else:
            deaccelerate_in_counts              = 2**31 -1 #10**9

        return np.array([target_position_in_counts,speed_in_counts,accelerate_in_counts,deaccelerate_in_counts])

    def get_movement_profile_in_encoder_pulse(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position * self._encoder_res*4)/ (2*pi) )
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( (self._accelerate)*60/ ((2*pi)) )
        deaccelerate_in_rpm_per_s               = int( (self._deaccelerate)*60 / ((2*pi)) )


        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])

    def MAXPOS_get_movement_profile_in_encoder_pulse(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position* self._encoder_res*4)/ (2*pi))
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( (self._accelerate)*60/ ((2*pi)) )
        deaccelerate_in_rpm_per_s               = int( (self._deaccelerate)*60 / ((2*pi)) )

        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])
    
    def MC_get_movement_profile(self):
        self.get_movement_profile_in_rad()
        target_position_in_counts               = int( (self._target_position* self._encoder_res*4)/ (2*pi))
        speed_in_rmp                            = int( (self._speed)*60 / ((2*pi )))
        accelerate_in_rpm_per_s                 = int( self._accelerate * self._encoder_res *4 / (2*pi) )
        deaccelerate_in_rpm_per_s               = int( self._deaccelerate * self._encoder_res *4 / (2*pi) )

        return np.array([target_position_in_counts,speed_in_rmp,accelerate_in_rpm_per_s,deaccelerate_in_rpm_per_s])

    def _rad_to_inc(self,rad):
        return rad * self._encoder_res / (2*pi)

    def _rad_per_Sec_to_RPM(self,rad_per_Sec):
        return rad_per_Sec * 60 /(2*pi)