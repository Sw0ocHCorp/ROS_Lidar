#!/usr/bin/env python


#############################################################################
# imports
#############################################################################
from sensor_msgs.msg import LaserScan  # type de message pour le lidar, cf comment ci-dessous
import math
import rospy
from geometry_msgs.msg import Twist

# type LaserScan :
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32 angle_min
# float32 angle_max
# float32 angle_increment
# float32 time_increment
# float32 scan_time
# float32 range_min
# float32 range_max
# float32[] ranges
# float32[] intensities


          
#############################################################################
# Lidar
#############################################################################
def processScan(data):
    dist_detect = 1 # 1 m, to be adjusted
    scan_range = data.angle_max - data.angle_min
    nb_values = len(data.ranges) # nombre de valeurs renvoyees par le lidar

    for count,value in enumerate(data.ranges):
            obst = ( (not math.isnan(value)) and value < dist_detect)
            current_angle = data.angle_min + count* data.angle_increment
	    # ...



#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
	rospy.init_node('lidar_scan', anonymous=True)
	Hz = 10
	rate = rospy.Rate(Hz)
	T = 1.0 / Hz
	
	#lidar
	rospy.Subscriber("scan", LaserScan, processScan)
	
	#loop at rate 
	while(not rospy.is_shutdown()):
		#publish the message
		rate.sleep()