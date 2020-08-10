from Ethercat_Communication import common

                
CONTROL_WORD                                    = [0x6040, 0x00, 'UNSIGNED16']
            
STATUS_WORD                                     = [0x6041, 0x00, 'UNSIGNED16']
            
TARGET_POSITION                                 = [0x607A, 0x00, 'INTEGER32']
            
POSITION_OFFSET                                 = [0x60B0, 0x00, 'INTEGER32']
            
VELOCITY_OFFSET                                 = [0x60B1, 0x00, 'INTEGER32']
            
TORQUE_OFFSET                                   = [0x60B2, 0x00, 'INTEGER16']
            
MODE_OF_OPERATION                               = [0x6060, 0x00, 'INTEGER8']
            
PHYSICAL_OUTPUT                                 = [0x60FE, 0x01, 'INTEGER32']
            
TOUCH_PROBE_FUNCTION                            = [0x60B8, 0x00, 'INTEGER16']
            
PROFILE_VELOCITY                                = [0x6081, 0x00, 'UNSIGNED32']
            
PROFILE_ACCEL                                   = [0x6083, 0x00, 'UNSIGNED32']
            
PROFILE_DECEL                                   = [0x6084, 0x00, 'UNSIGNED32']
            
ABORT_CONNECTION_OPTION_CODE                    = [0x6007, 0x00, 'INTEGER16']        
            
POSITION_ACTUAL_VALUE                           = [0x6064, 0x00, 'INTEGER32']
            
VELOCITY_ACTUAL_VALUE                           = [0x606C, 0x00, 'INTEGER32']
            
TORQUE_ACTUAL_VALUE                             = [0x6077, 0x00, 'INTEGER16']
            
MODE_OF_OPERATION_DISPLAY                       = [0x6061, 0x00, 'INTEGER8']
            
DIGITAL_INPUT                                   = [0x60FD, 0x00, 'UNSIGNED32']
            
TOUCH_PROBE_STATUS                              = [0x60B9, 0x00, 'UNSIGNED16']
            
TOUCH_PROBE_POSITION1                           = [0x60BA, 0x00, 'INTEGER32']
            
TOUCH_PROBE_POSITION2                           = [0x60BB, 0x00, 'INTEGER32']
            
ERROR_CODE                                      = [0x603F, 0x00, 'UNSIGNED16']
        
NODE_ID                                         = [0x2000, 0x00, 'UNSIGNED8']

ACCELERATE                                      = [0x6083, 0x00, 'UNSIGNED32']

DEACCELERATE                                    = [0x6084, 0x00, 'UNSIGNED32']

HOMING_METHOD                                   = [0x6098, 0x00, 'INTEGER8']

HOMING_METHOD_SPEED_SWITCH                      = [0x6099, 0x01, 'UNSIGNED32']

HOMING_METHOD_SPEED_ZERO                      = [0x6099, 0x02, 'UNSIGNED32']

HOMING_ACCE                                     = [0x609A, 0x00, 'UNSIGNED32']

class SERVO_MODE:
    profile_position                            = 1
    profile_velocity                            = 3
    homing_mode                                 = 6
    cyclic_synchronous_position_mode            = 8
    cyclic_synchronous_Velocity_mode            = 9
    cyclic_synchronous_torque_mode              = 10    


# MANUFACTURE_DEVICE_NAME                         = [0x1008, 0x00, 'STRING' ]
                    
# class EPOS4_DEFINE:       

#     class OBJECT_DEFINE:
        
#         CONTROL_WORD                                        = [0x6040, 0x00, 'UNSIGNED16']
        
#         STATUS_WORD                                         = [0x6041, 0x00, 'UNSIGNED16']
        
#         TARGET_POSITION                                     = [0x607A, 0x00, 'INTEGER32']

#         POSITION_OFFSET                                     = [0x60B0, 0x00, 'INTEGER32']

#         VELOCITY_OFFSET                                     = [0x60B1, 0x00, 'INTEGER32']

#         TORQUE_OFFSET                                       = [0x60B2, 0x00, 'INTEGER16']
        
#         MODE_OF_OPERATION                                   = [0x6060, 0x00, 'INTEGER8']
        
#         PHYSICAL_OUTPUT                                     = [0x60FE, 0x01, 'INTEGER32']
        
#         TOUCH_PROBE_FUNCTION                                = [0x60B8, 0x00, 'INTEGER16']
        
#         PROFILE_VELOCITY                                    = [0x6081, 0x00, 'UNSIGNED32']
        
#         PROFILE_ACCEL                                       = [0x6083, 0x00, 'UNSIGNED32']
        
#         PROFILE_DECEL                                       = [0x6084, 0x00, 'UNSIGNED32']

#         ABORT_CONNECTION_OPTION_CODE                        = [0x6007, 0x00, 'INTEGER16']        
        
#         POSITION_ACTUAL_VALUE                               = [0x6064, 0x00, 'INTEGER32']
        
#         VELOCITY_ACTUAL_VALUE                               = [0x606C, 0x00, 'INTEGER32']
        
#         TORQUE_ACTUAL_VALUE                                 = [0x6077, 0x00, 'INTEGER16']
        
#         MODE_OF_OPERATION_DISPLAY                           = [0x6061, 0x00, 'INTEGER8']
        
#         DIGITAL_INPUT                                       = [0x60FD, 0x00, 'UNSIGNED32']
        
#         TOUCH_PROBE_STATUS                                  = [0x60B9, 0x00, 'UNSIGNED16']
        
#         TOUCH_PROBE_POSITION1                               = [0x60BA, 0x00, 'INTEGER32']
        
#         TOUCH_PROBE_POSITION2                               = [0x60BB, 0x00, 'INTEGER32']
        
#         ERROR_CODE                                          = [0x603F, 0x00, 'UNSIGNED16']

#         NODE_ID                                             = [0x2000, 0x00, 'UNSIGNED8']



        