#!/usr/bin/env python3

import rospy
from default_movement import default_movement
from Odometry import OdometryNode
import Obstacle_Avoidance
import PID_controller
from sensor_msgs.msg import Range
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

target_sensor_position = 4.5
vehicle_speed = float(rospy.get_param("/maxvel"))#0.25

speed = WheelsCmdStamped()
rospy_rate = 15
obstacle_distance = Range()

turn_left_at_fork = False
turn_right_at_fork = False
roadsign_detected = False
roadsign_confirmed = False


class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):
        self.theta_ref = str(0)
        print("theta.ref is: ", self.theta_ref)
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher("/ejvirkus/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)

        self.prev_e = 0

    def on_shutdown(self):  #Seismajäämine juhul kui robot välja lülitub
        rospy.on_shutdown(self.shutdown)

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
            if obstacle_distance > Range(2):
                OdometryNode(self)
                #t1 = time.time()
                e, omega = PID_controller.PIDController.apply_controller(self, self.prev_e)
                self.prev_e = e
                speed.vel_left = vehicle_speed  + omega
                speed.vel_right = vehicle_speed - omega
                #print("omega is: ", PID_controller.apply_controller())
                self.pub.publish(speed)
            else:
                Obstacle_Avoidance.obstacleavoidance()

if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.run()
    rospy.spin()