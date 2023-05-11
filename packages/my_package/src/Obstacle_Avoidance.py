import rospy
from time import sleep
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
rospy_rate = 15
speed = WheelsCmdStamped()

class ObstacleAvoidance(DTROS):

        def __init__(self, node_name):

                self.ticksR = 0
                self.ticksL = 0

                self.right = self.ticksR
                self.left = self.ticksL

                self.right_initial = self.right
                self.left_initial = self.left
                self.newright = self.right - self.right_initial
                self.newleft = self.left - self.left_initial

        def ticksLeft(self, data):
                self.ticksL = data.data

        def ticksRight(self, data):
                self.ticksR= data.data

        def obstacleavoidance(self):


                while self.left <= 30:
                        speed.vel_left = 0.2
                        speed.vel_right = 0.0
                        speed.vel.publish(speed)
                        self.left = self.left - self.left_initial
                if self.left > 30:
                        speed.vel_left = 0.0
                        speed.vel_right = 0.0
                        speed.vel.publish(speed)
                        sleep
                        while self.right < 100:
                                speed.vel_left = 0.2
                                speed.vel_right = 0.2
                                speed.vel.publish(speed)
                                self.right = self.right - self.right_initial
                        if self.right >= 100:
                                self.right = self.right - self.right_initial
                                speed.vel_left = 0.0
                                speed.vel_right = 0.0
                                speed.vel.publish(speed)
                                sleep
                                while self.right < 200:
                                        self.right = self.right - self.right_initial
                                        speed.vel_right = 0.2
                                        speed.vel_left = 0.0
                                        self.pub.publish(speed)
                                        sleep
                                while self.right >= 300:
                                        self.right = self.right - self.right_initial
                                        speed.vel_right = 0.4
                                        speed.vel_left = 0.4
                                        self.pub.publish(speed)
                                        sleep
                                if self.right >= 400:
                                        self.right = self.right - self.right_initial
                                        speed.vel_right = 0.25
                                        speed.vel_left = 0.25
                                        self.pub.publish(speed)

if __name__ == '__main__':
        node = ObstacleAvoidance(node_name='ObstacleAvoidance')
        node.run()
        rospy.spin()