import math
import numpy as np 


JOINT_1                                         = 0
JOINT_2                                         = 1
JOINT_3                                         = 2
JOINT_4                                         = 3
JOINT_5                                         = 4
JOINT_6                                         = 5



class kinematic:
    def __init__(self):
        self.theta_array                        = np.full([6],0)
        self.alpha_array                        = np.full([6],0)
        self.r_array                            = np.full([6],0)
        self.d_array                            = np.full([6],0)
        # init value
        self.theta_array[JOINT_1]               = math.pi
        self.theta_array[JOINT_2]               = math.pi/2
        self.theta_array[JOINT_3]               = math.pi/2
        self.theta_array[JOINT_4]               = math.pi/2
        self.theta_array[JOINT_5]               = 0
        self.theta_array[JOINT_6]               = math.pi/2  #constant

        self.alpha_array[JOINT_1]               = -math.pi/2 #constant
        self.alpha_array[JOINT_2]               = math.pi/2  #constant
        self.alpha_array[JOINT_3]               = -math.pi/2 #constant
        self.alpha_array[JOINT_4]               = math.pi/2  #constant
        self.alpha_array[JOINT_5]               = -math.pi/2 #constant
        self.alpha_array[JOINT_6]               = 0          #constant

        self.r_array[JOINT_1]                   = 0          #constant
        self.r_array[JOINT_2]                   = 0          #constant
        self.r_array[JOINT_3]                   = 0          #constant
        self.r_array[JOINT_4]                   = 0          #constant
        self.r_array[JOINT_5]                   = 0          #constant
        self.r_array[JOINT_6]                   = 22         #constant

        self.d_array[JOINT_1]                   = 259        #constant
        self.d_array[JOINT_2]                   = 0          #constant
        self.d_array[JOINT_3]                   = 262        #constant
        self.d_array[JOINT_4]                   = 0          #constant
        self.d_array[JOINT_5]                   = 321.5      #constant
        self.d_array[JOINT_6]                   = 0          #constant




    
    def homogeneous_translation_func(self, theta , alpha , r , d):
        """This function return homogeneous transtaion matrix from frame n-1 to fram n
            parameter: theta, alpha are gotten in DH table with radian unit
                       r, d are gotten in DH table in mm unit"""

        H_array                                 = np.full([4,4],0)

        H_array[0][0]                           = math.cos(theta)
        H_array[0][1]                           = -math.sin(theta) * math.cos(alpha)
        H_array[0][2]                           = math.sin(theta) * math.cos(alpha)
        H_array[0][3]                           = r * math.cos(theta)


        H_array[1][0]                           = math.sin(theta)
        H_array[1][1]                           = math.cos(theta) * math.cos(alpha)
        H_array[1][2]                           = -math.cos(theta) * math.sin(alpha)
        H_array[1][3]                           = r * math.sin(theta)


        H_array[2][0]                           = 0
        H_array[2][1]                           = math.sin(alpha)
        H_array[2][2]                           = math.cos(alpha)
        H_array[2][3]                           = d

        
        H_array[3][0]                           = 0
        H_array[3][1]                           = 0
        H_array[3][2]                           = 0
        H_array[3][3]                           = 1

        return H_array


    def forward_kinematic(self, theta_array):
        
        homo_array                              = np.full(6,0)
        translation_result                      = 1
        for i in range(6):
            homo_array[i]                       = self.homogeneous_translation_func( theta_array[i],
                                                                                     self.alpha_array[i],
                                                                                     self.r_array[i],
                                                                                     self.d_array[i] )

            translation_result                  = translation_result * homo_array[i] 

        return translation_result