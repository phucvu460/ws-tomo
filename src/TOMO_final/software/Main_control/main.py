from Mailbox import mail_box
import yaml
import math
import time


if __name__ == '__main__':

    #get slave info
    """read config files"""
    with open('./Config/Ethercat_config.yaml') as descrip_file:
        data = yaml.load(descrip_file, Loader=yaml.FullLoader)

    """interface name""" 
    number_of_slave                    = data['network']['slaves_count']

    _mail                               = mail_box.MAIL_BOX()

    #switch on all motor
    for i in range(number_of_slave):
        _mail.servo_on(i)

    