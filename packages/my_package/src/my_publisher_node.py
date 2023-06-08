#!/usr/bin/env python3

import rospy
import time
from default_movement import default_movement
from Odometry import OdometryNode
import PID_controller
from sensor_msgs.msg import Range
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from smbus2 import SMBus

target_sensor_position = 4.5
vehicle_speed = float(rospy.get_param("/maxvel"))#0.25

speed = WheelsCmdStamped()
rospy_rate = 15

class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):
        self.bus = SMBus(1)
        self.theta_ref = str(0)
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/ejvirkus/left_wheel_encoder_node/tick', WheelEncoderStamped, self.ticks_leftwheel)
        rospy.Subscriber('/ejvirkus/right_wheel_encoder_node/tick', WheelEncoderStamped, self.ticks_rightwheel)
        self.pub = rospy.Publisher("/ejvirkus/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.tof = rospy.Subscriber("/ejvirkus/front_center_tof_driver_node/range", Range, self.callback)

        self.prev_e = 0

        self.ticksR = 0
        self.ticksL = 0

        self.range = 1
        self.ticksR = 0
        self.ticksL = 0

        self.rightvalues = [[0, 1, 1, 1, 1, 1, 1, 1], [0, 0, 1, 1, 1, 1, 1, 1], [0, 0, 0, 1, 1, 1, 1, 1], [0, 0, 0, 0, 1, 1, 1, 1], [0, 0, 0, 0, 0, 1, 1, 1], [0, 0, 0, 0, 0, 0, 1, 1], [0, 0, 0, 0, 0, 0, 0, 1]]
        self.leftvalues = [[1, 1, 1, 1, 1, 1, 1, 0], [1, 1, 1, 1, 1, 1, 0, 0], [1, 1, 1, 1, 1, 0, 0, 0], [1, 1, 1, 1, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0, 0]]

    def callback(self, data):
        self.range = data.range

    def ticks_rightwheel(self, data):
        self.ticksR = data.data
    
    def ticks_leftwheel(self, data):
        self.ticksL = data.data

    def on_shutdown(self):  #Seismajäämine juhul kui robot välja lülitub
        rospy.on_shutdown(self.shutdown)

    def obstacle_avoidance(self):
        speed.vel_right = 0
        speed.vel_left = 0
        self.pub.publish(speed)
        time.sleep(1)
        # Turn 90 degrees right in 0.65 second
        speed.vel_right = -0.02
        speed.vel_left = 0.3
        self.pub.publish(speed)
        time.sleep(0.6)
        
        speed.vel_right = 0
        speed.vel_left = 0
        self.pub.publish(speed)
        time.sleep(1)
        # Go straight for 2.2 meters
        speed.vel_right = 0.3
        speed.vel_left = 0.15
        self.pub.publish(speed)
        time.sleep(3)
        #peed.vel_right = 0
        #peed.vel_left = 0
        #elf.pub.publish(speed)
        #ime.sleep(0.3)
        # Turn 90 degrees left in 0.65 second
        #peed.vel_right = 0.3
        #peed.vel_left = -0.02
        #elf.pub.publish(speed)
        #ime.sleep(0.3)
        #peed.vel_right = 0
        #peed.vel_left = 0
        #elf.pub.publish(speed)
        #ime.sleep(0.3)
        # Go straight for 2.2 meters
        #peed.vel_right = 0.3
        #peed.vel_left = 0.3
        #elf.pub.publish(speed)

    def sharp_right(self):
        print("starting to turn right")
        speed.vel_right = -0.1
        speed.vel_left = 0.2
        self.pub.publish(speed)
        while sum(self.errorlist) == 0 or self.errorlist in self.rightvalues:
            #print("Sharp right!")
            #print("R:", self.theta_ref)
            temp = self.bus.read_byte_data(62, 17)
            self.theta_ref = bin(temp)[2:].zfill(8)
            self.errorlist = list(map(int, self.theta_ref))
            speed.vel_right = 0.2
            speed.vel_left = -0.1
            self.pub.publish(speed)
            #print("Right: ", self.theta_ref)
            self.lastturn = 1

    def sharp_left(self):
        print("starting to turn left")
        speed.vel_right = -0.1
        speed.vel_left = 0.2
        self.pub.publish(speed)
        while sum(self.errorlist) == 0 or self.errorlist in self.leftvalues:
            #print("Sharp left!")
            #print("L:", self.theta_ref)
            temp = self.bus.read_byte_data(62, 17)
            self.theta_ref = bin(temp)[2:].zfill(8)
            self.errorlist = list(map(int, self.theta_ref))
            speed.vel_right = -0.2
            speed.vel_left = 0.1
            self.pub.publish(speed)
            #print("Left: ", self.theta_ref)
            self.lastturn = 2

    def shutdown(self):
        speed.vel_right = 0
        speed.vel_left = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        #t0 = time.time()
        self.line_values = []
        for i, value in enumerate(self.theta_ref):
            if value == '1':
                self.line_values.append(i + 1)
        print(self.line_values)

        self.theta_hat = sum(self.line_values)
        print("theta_hat is: ", self.theta_hat)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.range > 0.25:
                print(self.range)
                temp = self.bus.read_byte_data(62, 17)
                self.theta_ref = bin(temp)[2:].zfill(8)
                self.errorlist = list(map(int, self.theta_ref))
                print(self.errorlist)
                if self.errorlist in self.rightvalues:  #Äkiline parem põõre
                    self.sharp_right()
                if self.errorlist in self.leftvalues:    #Äkiline vasak põõre
                    self.sharp_left()
                OdometryNode(self)
                #t1 = time.time()
                e, omega = PID_controller.PIDController.apply_controller(self, self.prev_e)
                self.prev_e = e
                speed.vel_left = vehicle_speed  + omega
                speed.vel_right = vehicle_speed - omega
                #print("omega is: ", PID_controller.apply_controller())
                self.pub.publish(speed)
            else:
                self.obstacle_avoidance()

if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.run()
    rospy.spin()