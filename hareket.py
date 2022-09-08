import sys
import numpy as np
import rospy
from autoware_msgs.msg import VehicleCmd  
import time

class ilerleme_feature:
    
    
    def __init__(self):
        self.velocity = 10
        self.angle = 0.5
        self.msg = VehicleCmd()
        self.pub = rospy.Publisher('/vehicle_cmd',VehicleCmd, queue_size = 10)
        
        rospy.init_node('hareket',anonymous=True)
        rate = rospy.Rate(10) # 10hz

        self.msg.twist_cmd.twist.angular.z = self.angle
        self.msg.twist_cmd.twist.linear.x = self.velocity
        self.pub.publish(self.msg)

       

def main(args):

    lf = ilerleme_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown")

main(sys.argv)