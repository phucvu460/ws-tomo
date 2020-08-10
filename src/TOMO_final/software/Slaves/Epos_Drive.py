

class EPOS:
    def __init__(self,mail_box,ID):

        self._current_torque                    = 0
        self._ID                                = ID
        self._velocity                          = 0
        self._deacce                            = 0
        self._acceleration                      = 0
        self._actual_position                   = 0
        self._target_reach                      = True

        #register with the mail box
        self._mail_box                          = mail_box
        self._mail_box.register( self._ID )



        