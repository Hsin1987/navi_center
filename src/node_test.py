#!/usr/bin/env python
import rospy
from rss.node_monitor import *


nodes = [
            '/amcl',
            '/amcl_aux_localization',
            '/ar_base_serial',
            '/batteryLogging',
            '/batteryMonitor',
            '/bumper',
            '/camera_rear',
            '/camera_rear/apriltag_detector',
            '/camera_rear/image_proc',
            '/camera_top',
            '/camera_top/apriltag_detector',
            '/camera_top/image_proc',
            '/docking_navigation',
            '/laser_filter_laser_front',
            '/laser_filter_laser_rear',
            '/laser_front/rplidarNode',
            '/laser_rear/rplidarNode',
            '/map_server_amcl',
            '/map_server_nav',
            '/move_base',
            '/move_operation',
            '/mux_cmd_vel',
            '/navi_center',
            '/navi_server',
            '/position_listener',
            '/relay_scan_front',
            '/relay_scan_rear',
            '/robot_state_publisher',
            '/rosapi',
            '/rosbridge_websocket',
            '/rss_server',
            '/sysHealth']

if __name__ == "__main__":
    rospy.init_node('rosping_existence')

    r = rospy.Rate(0.01)
    while not rospy.is_shutdown():
        fine, result = checkNodeExistence(nodes)
        if fine:
            print("Everything is fine")
        else:
            for node, alive in result.items():
                if not alive:
                    print (node + " is dead.")
        r.sleep()