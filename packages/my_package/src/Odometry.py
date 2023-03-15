#Kutsub välja ratastelt saadud info, et anda välja meetrites ja radiaanides läbitud vahemaad ja nurgad

#!/usr/bin/env python3

import math
import numpy as np 
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

rate = rospy.Rate(1)

class OdometryNode(DTROS):

    def __init__(self, node_name):

        self.prev_tick_left = 0
        self.prev_tick_right = 0
        self.ticksL = 0
        self.ticksR = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.seqLeft = rospy.Subscriber('/ejvirkus/left_wheel_encoder_node/tick', WheelEncoderStamped, self.ticks_leftwheel)
        self.seqRight = rospy.Subscriber('/ejvirkus/right_wheel_encoder_node/tick', WheelEncoderStamped, self.ticks_rightwheel)
        self.pub = rospy.Publisher('ejvirkus/wheels_driver_node/wheels_cmd',WheelsCmdStamped , queue_size=10)

    def ticks_leftwheel(self, data):    #Odomeetria jaoks robotilt väärtuste kutsumine
        self.ticksL = data.data

    def ticks_rightwheel(self, data):
        self.ticksR = data.data

    def OdometryNode(self):
        
        N_tot = 135 #Tickide arv ratta täispöördel
        alpha = 2*np.pi/N_tot
        self.delta_ticksL = self.ticksL-self.prev_tick_left # delta ticks of left wheel 
        self.delta_ticksR = self.ticksR-self.prev_tick_right # delta ticks of right wheel 
        self.rotation_wheel_left = alpha * self.delta_ticks_left # total rotation of left wheel 
        self.rotation_wheel_right = alpha * self.delta_ticks_right # total rotation of right wheel 

        R = 0.0335 #Ratta raadius

        d_left = R * self.rotation_wheel_left
        d_right = R * self.rotation_wheel_right

        d_A = (d_left + d_right)/2 #Roboti läbitud distanss kaadris meetrites

        baseline_wheel2wheel = 0.1 #Roboti rataste vahe meetrites

        Delta_Theta = (d_right-d_left)/baseline_wheel2wheel


        self.prev_tick_right = self.ticksR
        self.prev_tick_left = self.ticksL
        

        print(f"The robot has rotated: {np.rad2deg(Delta_Theta)} degrees")
        print(f"The robot has travelled: {d_A} meters")
        print(f"_________________________________________________________________________\n")
        rate.sleep()

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    rospy.spin() #Hoiab node käimas