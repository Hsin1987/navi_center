#! /usr/bin/env python
"""
1. http://www.theconstructsim.com/read-laserscan-data/

2. Python
    https://zhuanlan.zhihu.com/p/27819816
3. laser_geometry
    http://wiki.ros.org/laser_geometry#Python_Usage
rviz tool
3. rviz_tools_py/example/demo.py

https://github.com/DavidB-CMU/rviz_tools_py/blob/master/example/demo.py
"""
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math
import numpy as np




"""
Task:
1. Use math function to describe the geometry of region on interest(EV).
2. Use math function to indicate the laser scan beam in one spin.
3. Check which beam could be reflected by th elevator.
4. Calculate the RR,reflection rate(%) on region of interest.
5. Monitor the RR variance.
6. If RR is below a threshold, find all the point in front of the interest obstacle (door, ev).
7. Filter and Clustering the reflection points.
8. Decide whether AMR should move away from entrance position.



4-A, Calculate missing percentage of the door/ ev reflection line.
4-B, Does the beam return reflection ?
    If it got reflection, is it before of after the elevator door / wall?


Door close:
    1. Percentage of the beams pointing the ev door got reflection increasing to over OO %

    2. 65


Most of the obstacle are moving out >>> hold steady, wait for people moving out.
Large Obstacle >>> moving away from the entrance position.

"""

rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()


pc2_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
pc_pub = rospy.Publisher("converted_pc", PointCloud, queue_size=1)
x_max = 18.2
x_min = 19.1

y_max = 17.35
y_min = 17.45


def gen_to_numpy(gen):
    return np.array([x for x in gen])

# transformLaserScanToPointCloud
def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    points_list = []
    for point in msg.points:
        points_list.append([point.x, point.y, point.z])



    array = np.asarray(points_list)
    ev = [np.where([np.logical_and((x_min <= point[0] <= x_max), (y_min <= point[1] <= y_max)) for point in array])]
    print(ev)





    # or a list of the individual points which is less efficient
    point_list = pc2.read_points_list(pc2_msg)

    # we can access the point list with an index, each element is a namedtuple
    # we can access the elements by name, the generator does not yield namedtuples!
    # if we convert it to a list and back this possibility is lost
    print(point_list[len(point_list) / 2].x)


rospy.Subscriber("/assemble_scans_client/output", PointCloud, scan_cb, queue_size=1)
rospy.spin()