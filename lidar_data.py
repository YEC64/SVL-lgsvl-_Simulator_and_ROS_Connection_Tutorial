import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class Lidar_Feature:
    def __init__(self):
        
        self.subscriber2 = rospy.Subscriber("/points_raw", PointCloud2, self.callback_lidar, queue_size = 10)
        
        rospy.init_node("ays", anonymous = True)
        rate = rospy.Rate(10)   #10Hz
   

    def callback_lidar (self, ros_data):

        print(ros_data)

def main(srgs):
    lf = Lidar_Feature()
    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Ros Shutdown")

main(sys.argv)