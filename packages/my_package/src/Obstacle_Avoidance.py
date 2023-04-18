import rospy
from sensor_msgs.msg import range
from duckietown_msgs.msg import WheelsCmdStamped

rate = rospy.rate(15)
obstacle_distance = range()
speed = WheelsCmdStamped()

def obstacleavoidance(self):
    if obstacle_distance < 2:
        speed.vel_left = 0          #keerab ennast vasakule
        speed.vel_right = 0.5
        self.pub.publish(speed) 
        rospy.sleep(2)

        speed.vel_left = 0.75       #sõidab otse, eeldatavasti takistusest mööda
        speed.vel_right = 0.75
        self.pub.publish(speed) 
        rospy.sleep(4)

        speed.vel_left = 0.5        #keerab ennast tagasi suunaga rajale
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.sleep(4)

        speed.vel_left = 0.75       #sõidab tagasi rajale
        speed.vel_right = 0.75
        self.pub.publish(speed)
        rospy.sleep(4)

        speed.vel_left = 0          #keerab ennast tagasi algsele suunale
        speed.vel_right = 0.5
        self.pub.publish(speed)
        rospy.sleep(2)