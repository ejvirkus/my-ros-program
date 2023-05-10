import rospy
from duckietown_msgs.msg import WheelsCmdStamped
#rate = rospy.Rate(15)
speed = WheelsCmdStamped()
sleep = rospy.sleep(0.5)
def obstacleavoidance(self):
        right_initial = self.right
        left_initial = self.left
        right = self.right - right_initial
        left = self.left - left_initial
        while left <= 30:
                speed.vel_left = 0.2
                speed.vel_right = 0.0
                speed.vel.publish(speed)
                left = self.left - left_initial
        if left > 30:
                speed.vel_left = 0.0
                speed.vel_right = 0.0
                speed.vel.publish(speed)
                sleep
                while right < 100:
                        speed.vel_left = 0.2
                        speed.vel_right = 0.2
                        speed.vel.publish(speed)
                        right = self.right - right_initial
                if right >= 100:
                        right = self.right - right_initial
                        speed.vel_left = 0.0
                        speed.vel_right = 0.0
                        speed.vel.publish(speed)
                        sleep
                        while right < 200:
                                right = self.right - right_initial
                                speed.vel_right = 0.2
                                speed.vel_left = 0.0
                                self.pub.publish(speed)
                                sleep
                        while right >= 300:
                                right = self.right - right_initial
                                speed.vel_right = 0.4
                                speed.vel_left = 0.4
                                self.pub.publish(speed)
                                sleep
                        if right >= 400:
                                right = self.right - right_initial
                                speed.vel_right = 0.25
                                speed.vel_left = 0.25
                                self.pub.publish(speed)