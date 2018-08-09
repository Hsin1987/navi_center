#!/usr/bin/env python
import rospy

import cv2
import numpy as np
import math
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
import tf

import rviz_tools_py as rviz_tools


# Define exit handler
def cleanup_node():
    print "Shutting down node"
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')
pc_pub = rospy.Publisher("point", PointStamped, queue_size=1)

rospy.get_param('/use_sim_time', True)
# listener = tf.TransformListener()



def callback(data):
    """
    try:
        #listener.waitForTransform('laser_rear', 'map', rospy.Time(), rospy.Duration(100.0))

        #(trans, rot) = listener.lookupTransform('laser_rear', 'map', rospy.Time())


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    """
    rot = [0.0, 0.0, 0.354324581624, 0.935122500455]
    euler = tf.transformations.euler_from_quaternion(rot)
    trans = [18.4, 16.3, 0]

    frame = np.zeros((500, 500, 3), np.uint8)
    angle = data.angle_min + euler[2] - 80
    counter = 0
    for r in data.ranges:
        counter +=1
        if r <= data.range_max and r >= data.range_min:
            try:
                x = trans[0] + r * math.cos(angle)
                y = trans[1] + r * math.sin(angle)
                #print x, y
                # Line:

                # Publish a line between two ROS Point Msgs
                point2 = PointStamped()
                point2.header.frame_id = 'map'
                point2.point.x = x
                point2.point.y = y
                width = 0.01
                if counter % 2 == 0:
                    #markers.publishLine(point1, point2, 'red', width, 0.01)  # point1, point2, color, width, lifetime
                    pc_pub.publish(point2)
                angle = angle + data.angle_increment
            except:
                print('sthing wrong')
        else:
            print("r:" + str(r))




def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber("/scan_rear", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()