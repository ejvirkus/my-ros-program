#!/usr/bin/env python3

import rospy
from time import sleep
from default_movement import default_movement
from Odometry import OdometryNode
import PID_controller
from sensor_msgs.msg import Range
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped

target_sensor_position = 4.5
vehicle_speed = float(rospy.get_param("/maxvel"))#0.25

speed = WheelsCmdStamped()
rospy_rate = 15

class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):
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

    def callback(self, data):
        self.range = data.range

    def ticks_rightwheel(self, data):
        self.ticksR = data.data
    
    def ticks_leftwheel(self, data):
        self.ticksL = data.data

    def on_shutdown(self):  #Seismajäämine juhul kui robot välja lülitub
        rospy.on_shutdown(self.shutdown)

    def obstacle_avoidance(self):

        right_initial = self.ticksR
        left_initial = self.ticksL

        right = self.ticksR - right_initial
        print(right)
        left = self.ticksL - left_initial
        print(left)
        sleep(1)
         
        while left <= 60:
            speed.vel_left = 0.2
            speed.vel_right = 0.0
            self.pub.publish(speed)
            sleep(1)
            left = self.ticksL - left_initial
        if left > 60:
            speed.vel_left = 0.0
            speed.vel_right = 0.0
            self.pub.publish(speed)
            while right < 200:
                    speed.vel_left = 0.2
                    speed.vel_right = 0.2
                    self.pub.publish(speed)
                    right = self.ticksR - right_initial
            if right >= 200:
                    right = self.ticksR - right_initial
                    speed.vel_left = 0.0
                    speed.vel_right = 0.0
                    self.pub.publish(speed)
                    sleep(1)
                    while right < 350:
                            right = self.ticksR - right_initial
                            speed.vel_right = 0.2
                            speed.vel_left = 0.0
                            self.pub.publish(speed)
                            sleep(1)
                    while right >= 350 and right < 600:
                            right = self.ticksR - right_initial
                            speed.vel_right = 0.4
                            speed.vel_left = 0.4
                            self.pub.publish(speed)
                            sleep(1)
                    if right >= 600:
                            right = self.ticksR - right_initial
                            speed.vel_right = 0.25
                            speed.vel_left = 0.25
                            self.pub.publish(speed)

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
            if self.range > 0.3:
                print(self.range)
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