#!/usr/bin/env python3

import numpy as np
import rospy
from Odometry import OdometryNode
from Movement import Move_Node
from PID_controller import PID_Controller
from duckietown.dtros import DTROS, NodeType
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped
speed = WheelsCmdStamped()
rospy_rate = 40
error = 0 
last_error = 0


class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):

        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.seqLeft = rospy.Subscriber('/ejvirkus/left_wheel_encoder_node/tick', WheelEncoderStamped, self.ticks_leftwheel)
        self.seqRight = rospy.Subscriber('/ejvirkus/right_wheel_encoder_node/tick', WheelEncoderStamped, self.ticks_rightwheel)
        self.pub = rospy.Publisher('ejvirkus/wheels_driver_node/wheels_cmd',WheelsCmdStamped , queue_size=10)

    def on_shutdown(self):  #Seismajäämine juhul kui robot välja lülitub

        speed.vel_right = 0
        speed.vel_left = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        self.prev_tick_left = self.ticksL
        self.prev_tick_right = self.ticksR

        bus = SMBus(1)

        self.temp = bus.read_byte_data(62, 17)

        self.line_values = []
        for i, value in ennumerate(self.theta_ref):
            if value == '1':
                self.line_values.append(i + 1)
        print(self.line_values)

        self.theta_hat = sum(self.line_values)
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            OdometryNode(self)


if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.run()
    rospy.spin()