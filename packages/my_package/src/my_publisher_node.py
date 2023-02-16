#!/usr/bin/env python3

import math
import sys
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):

        self.e = 0
        self.theta_ref = 0
        self.theta_hat = 0
        self.e_int = 0
        self.e_der = 0
        self.prev_e = 0
        self.delta_t = 0
        
        self.prev_tick_left = 0
        self.prev_tick_right = 0
        self.ticksL = 0
        self.ticksR = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.seqLeft = rospy.Subscriber('/ejvirkus/left_wheel_encoder_node/tick', WheelEncoderStamped, self.ticks_leftwheel)
        self.seqRight = rospy.Subscriber('/ejvirkus/right_wheel_encoder_node/tick', WheelEncoderStamped, self.ticks_rightwheel)
        self.pub = rospy.Publisher('ejvirkus/wheels_driver_node/wheels_cmd',WheelsCmdStamped , queue_size=10)


    def on_shutdown(self):

        speed.vel_right = 0
        speed.vel_left = 0

        self.pub.publish(speed)

    def ticks_leftwheel(self, data):
        self.ticksL = data.data

    def ticks_rightwheel(self, data):
        self.ticksR = data.data

    def run(self):
        self.prev_tick_left = self.ticksL
        self.prev_tick_right = self.ticksR

        bus = SMBus(1)

        self.temp = bus.read_byte_data(62, 17)

        self.theta_ref = bin(temp)[2:].zfill(8)

        self.line_values = []
        for i, value in ennumerate(self.theta_ref):
            if value == '1':
                self.line_values.append(i + 1)
        print(self.line_values)

        self.theta_hat = sum(self.line_values)
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            def PIDController(v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t): #add theta_ref as input
                """
                Args:
                    v_0 (:double:) linear Duckiebot speed (given).
                    theta_ref (:double:) reference heading pose
                    theta_hat (:double:) the current estiamted theta.
                    prev_e (:double:) tracking error at previous iteration.
                    prev_int (:double:) previous integral error term.
                    delta_t (:double:) time interval since last call.
                returns:
                    v_0 (:double:) linear velocity of the Duckiebot 
                    omega (:double:) angular velocity of the Duckiebot
                    e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
                    e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
                """
                
                # Tracking error
                self.e = self.theta_ref - self.theta_hat

                # integral of the error
                self.e_int = self.prev_int + self.e*self.delta_t

                # anti-windup - preventing the integral error from growing too much
                self.e_int = max(min(self.e_int,2),-2)

                # derivative of the error
                self.e_der = (self.e - self.prev_e)/self.delta_t

                # controller coefficients
                Kp = 1
                Ki = 0
                Kd = 0

                # PID controller for omega
                omega = Kp*self.e + Ki*self.e_int + Kd*self.e_der
                
                #print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
                
                return [v_0, omega], self.e, self.e_int

                
            N_tot = 135
            alpha = 2*np.pi/N_tot
            self.delta_ticksL = self.ticksL-self.prev_tick_left # delta ticks of left wheel 
            self.delta_ticksR = self.ticksR-self.prev_tick_right # delta ticks of right wheel 
            self.rotation_wheel_left = alpha * self.delta_ticks_left # total rotation of left wheel 
            self.rotation_wheel_right = alpha * self.delta_ticks_right # total rotation of right wheel 

            R = 0.0335 # insert value measured by ruler, in *meters*

            d_left = R * self.rotation_wheel_left
            d_right = R * self.rotation_wheel_right

            d_A = (d_left + d_right)/2 # robot distance travelled in robot frame [meters]

            baseline_wheel2wheel = 0.1 #  Take a ruler and measure the distance between the center of t

            Delta_Theta = (d_right-d_left)/baseline_wheel2wheel # [radians]


            self.prev_tick_right = self.ticksR
            self.prev_tick_left = self.ticksL
            

            print(f"The robot has rotated: {np.rad2deg(Delta_Theta)} degrees")
            print(f"The robot has travelled: {d_A} meters")
            print(f"_________________________________________________________________________\n")
            rate.sleep()



if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()