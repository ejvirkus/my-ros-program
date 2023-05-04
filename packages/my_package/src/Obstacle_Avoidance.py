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
                speed(0.5)
                while right < 100:
                        speed.vel_left = 0.2
                        speed.vel_right = 0.2
                        speed.vel.publish(speed)