#!/usr/bin/env python

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