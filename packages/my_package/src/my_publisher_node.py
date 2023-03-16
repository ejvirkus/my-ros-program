#!/usr/bin/env python3

import rospy
from default_movement import default_movement
from Odometry import OdometryNode
from Movement import Move_Node
from PID_controller import PID_Controller
from duckietown.dtros import DTROS, NodeType
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

target_sensor_position = 4.5
vehicle_speed = 0.5

speed = WheelsCmdStamped()
rospy_rate = 40
error = 0 
last_error = 0

Kp = 0
Ki = 0
Kd = 0
I = 0

turn_left_at_fork = False
turn_right_at_fork = False
roadsign_detected = False
roadsign_confirmed = False

car = Move_Node(vehicle_speed)
pid_controller = PID_Controller(Kp, Ki, Kd, I, rospy_rate)


class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):
        self.theta_ref = str(0)
        print("theta.ref is: ", self.theta_ref)
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher("/ejvirkus/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)

    def on_shutdown(self):  #Seismajäämine juhul kui robot välja lülitub
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        Move_Node.move_stop
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        bus = SMBus(1)
        print("SMBus is: ", bus)

        self.current_sensor_position = bus.read_byte_data(62, 17)
        print("current position is: ", self.current_sensor_position)

        self.line_values = []
        for i, value in enumerate(self.theta_ref):
            if value == '1':
                self.line_values.append(i + 1)
        print(self.line_values)

        self.theta_hat = sum(self.line_values)
        print("theta_hat is: ", self.theta_hat)
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            OdometryNode(self)
            PID_Controller.apply_controller()


if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.run()
    rospy.spin()