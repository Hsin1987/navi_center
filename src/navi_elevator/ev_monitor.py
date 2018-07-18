#! /usr/bin/env python
"""
1. http://www.theconstructsim.com/read-laserscan-data/

2. 用Python打造无人驾驶车-激光雷达数据(1)
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
    pc2_msg = lp.projectLaser(msg)
    pc_msg = lg.transformLaserScanToPointCloud(msg)
    # now we can do something with the PointCloud2 for example:
    # publish it
    pc_pub.publish(pc_msg)
    pc2_pub.publish(pc2_msg)


    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)


    array = gen_to_numpy(point_generator)
    print(array)

    #print(counter)
    """
    ev = [np.where([np.logical_and((x_min <= point[0] <= x_max), (y_min <= point[1] <= y_max)) for point in array])]
    print(ev)



    # we can access a generator in a loop
    sum = 0.0
    num = 0
    for point in point_generator:
        if not math.isnan(point[2]):
            sum += point[2]
            num += 1
    # we can calculate the average z value for example
    print(str(sum / num))

    # or a list of the individual points which is less efficient
    point_list = pc2.read_points_list(pc2_msg)

    # we can access the point list with an index, each element is a namedtuple
    # we can access the elements by name, the generator does not yield namedtuples!
    # if we convert it to a list and back this possibility is lost
    print(point_list[len(point_list) / 2].x)
    """

rospy.Subscriber("/scan_rear", LaserScan, scan_cb, queue_size=1)
rospy.spin()