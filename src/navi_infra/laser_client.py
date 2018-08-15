#!/usr/bin/env python
"""
import roslib
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
roslib.load_manifest('laser_assembler')


def callback(laser_scan_msg):
    rospy.wait_for_service("assemble_scans")
    pub = rospy.Publisher("~output", PointCloud, queue_size=10)

    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
        now = rospy.get_rostime()
        response = assemble_scans(rospy.Time(0, 0), now)
        print "Got cloud with %u points" % len(response.cloud.points)
        pub.publish(response.cloud)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node("assemble_scans_client")

    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
"""

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math

rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)


def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    pc_pub.publish(pc2_msg)

    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)

    # we can access a generator in a loop
    sum = 0
    num = 0
    for point in point_generator:
        distance = ((point[0])**2 + (point[1]-2)**2)**0.5
        #print distance
        if distance < 2:
            sum += 1

    # we can calculate the average z value for example
    print(sum)

    # or a list of the individual points which is less efficient
    #point_list = pc2.read_points_list(pc2_msg)

    # we can access the point list with an index, each element is a namedtuple
    # we can access the elements by name, the generator does not yield namedtuples!
    # if we convert it to a list and back this possibility is lost
    #print(point_list[len(point_list) / 2].x)


rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()