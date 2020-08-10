from Serial_Communication import Serial_controller
import time
ser_left = Serial_controller.SERIAL_MAIN_CONTROLLER('/dev/ttyUSB0')
time.sleep(2)

ser_left.set_motor_data(3,0,700,500,True)
ser_left.set_motor_data(1,0,300,500,True)
