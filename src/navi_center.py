#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import yaml
import json
import std_srvs.srv
from std_msgs.msg import Empty, String, Float32, Float64, Bool
from navi_execution.RobotStatus import RobotStatus
from navi_planning.path_planner import PathPlanner
from navi_execution.map_manager import map_client
from navi_execution.goal_manager import set_tolerance, goal_agent
from navi_execution.basic_function import loading_service_parameter, loading_hotel_goal
from navi_infra.ev_manager import ev_agent_call, inform_ev_done, ev_agent_reach, check_elevator, release_ev
from navi_infra.ev_manager import entering_elevator, alighting_elevator
from navi_comm.connection_test import pinger
from rss.node_monitor import *
from rss.weixin_alarm import WXAlarm, sent_rss_notification

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

import dynamic_reconfigure.client
import time
import datetime
import threading
import re

battery_capacity_threshold = rospy.get_param("battery_capacity_threshold", 20.0)

# Subsystem Switch
rss_on = rospy.get_param("rss_on", False)
resource_check_on = rospy.get_param("resource_check_on", False)
alight_mb_on = rospy.get_param("alight_mb_on", True)


# Global Service parameter
service_dict = {}
goal_dict = {}
robotStatus = None

# Status Locker
lock = threading.Lock()

# ------------------------ Auxiliary Function ---------------------------- #
# Init the path_planner
path_planner = PathPlanner()
task_path = None

rss_notification = WXAlarm()
battery_rss_timestamp = None
charging_connected = False
pmu_online = True


# ===== Topic Publisher ===== #
# AMR machine reboot trigger
rebootPub = rospy.Publisher('/reqDeepReboot', Float64, queue_size=1)
# AMR Status Report Agent
robotStatusPub = rospy.Publisher('/amr_status_agent', String, queue_size=1, latch=True)
# Voice Channel
ttsPub = rospy.Publisher('/web/tts_request', String, queue_size=1)
# navigation_execution relative topic publisher
hotelGoalCB_Pub = rospy.Publisher('/hotelGoalCB', String, queue_size=1)
hotelGoalReachPub = rospy.Publisher('/hotelGoalReached', String, queue_size=1)
# Map & Position Init
changeNavMapPub = rospy.Publisher('/map_server_nav/reload', String, queue_size=1)
changeAMCLMapPub = rospy.Publisher('/map_server_amcl/reload', String, queue_size=1)
initPosePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

# ====== Action Trigger =====
# A1. Station Type
enterStationPub = rospy.Publisher('/enterStation', Bool, queue_size=1)
departStationPub = rospy.Publisher('/departStation', Bool, queue_size=1)

# B1. EV Type
checkEVPub = rospy.Publisher('/checkEV', Empty, queue_size=1)
doorReleasePub = rospy.Publisher('/doorRelease', Empty, queue_size=1)
enterEVPub = rospy.Publisher('/enterEV', String, queue_size=1)
alightEVPub = rospy.Publisher('/alightEV', String, queue_size=1)

# to do
currentFloorPub = rospy.Publisher('/currentFloor', String, queue_size=1, latch=True)

# M1. Move_Base Goal
goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)


def dynamic_reconfigure_cb():
    rospy.loginfo("[nc_ros] dynamicReconfigureCallback() is called")


# Call the move_base service: clear costmap for restart trying.
def clear_costmap():
    try:
        src_client = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)
        src_client()
        rospy.sleep(5.0)
    except rospy.ServiceException, e:
        str_msg = "[NC] Call clear_costmaps Service  failed: " + str(e)
        rospy.logwarn(str_msg)
        rospy.sleep(5.0)


# WiFi Agent
def wifi_connected():
    check_list = ['robocall_server_ip', 'robocall_server_ip']

    for target in check_list:
        if not pinger(service_dict[target]):
            msg = target + 'connection failed.'
            rospy.logwarn('[NC] ' + msg )
            ts = time.time()
            st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            if rss_on:
                rss_notification.sent(str(st) + " " + service_dict['AMR_ID'] + msg + " Request RSS. ", '@all')
            return False
    if not pinger(service_dict['hotel_public_ip']):
        rospy.logwarn('[NC] Hotel public ip connection failed.')
        msg = 'Hotel public ip connection failed'
        rospy.logwarn('[NC] ' + msg)
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        if rss_on:
            rss_notification.sent(str(st) + " " + service_dict['AMR_ID'] + msg + " Request RSS. ", '@all')
    return True


# PMU Monitor
def pmu_monitor(online):
    global pmu_online
    if not online.data:
        rospy.logwarn("[NC]: PMU offline.")
        # Check until onsite reboot
        pmu_online = False
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        if rss_on:
            rss_notification.sent(str(st) + " " + service_dict['AMR_ID'] + " : PMU offline! Request RSS. ", '@all')
    return


def setting_robot_mission(hotel_goal):
    global robotStatus, task_path
    # Check the hotel_goal? Is it in the list ?
    input_confirm, hotel_goal = goal_correct(hotel_goal, service_dict)
    if input_confirm:
        robotStatus.mission = hotel_goal
        pub_robot_status()
        rospy.loginfo("[NC] Received hotelGoal: " + hotel_goal)
    else:
        hotelGoalCB_Pub.publish('busy')
        rospy.loginfo('tts: 请输入正确的房号')
        announce = unicode('请输入正确的房号', 'utf-8')
        rospy.logwarn("[NC] Reject Task which is Not in the List.")
        ttsPub.publish(announce)
        robotStatus.mission = None
        pub_robot_status()
        return

    planning_path = path_planner.path_agent(robotStatus.position, robotStatus.mission)
    # Check the result is not empty.
    if not planning_path:
        hotelGoalCB_Pub.publish('busy')
        rospy.loginfo('tts: 无法规划路径，请重新输入正确房号')
        announce = unicode('无法规划路径，请重新输入正确房号', 'utf-8')
        ttsPub.publish(announce)
        rospy.logwarn("[NC] Path Planning Failed. Reject it And Ask for Correction.")
        robotStatus.mission = None
        pub_robot_status()
        return

    # Confirmation of Receiving task.
    rospy.loginfo('[NC] Start the Mission.')
    announce = unicode('好的...感谢有机会为您服务', 'utf-8')
    rospy.loginfo('tts: 好的...感谢有机会为您服务')
    ttsPub.publish(announce)
    rospy.sleep(2.0)
    task_path = planning_path
    rospy.loginfo('[NC] Task Path : ' + str(task_path))
    return


# Reject Hotel GOAL
def reject_goal(hotel_goal):
    global robotStatus
    # Feedback UI, it can't take task now.
    hotelGoalCB_Pub.publish('busy')
    # tts info
    rospy.loginfo('tts: 机器人电量过低.. 充电中，请稍候再试')
    announce = unicode('机器人电量过低.. 充电中，请稍候再试', 'utf-8')
    rospy.logwarn("[NC] Low Battery. Reject Task: " + str(hotel_goal))
    ttsPub.publish(announce)
    robotStatus.mission = None
    pub_robot_status()
    return


def return_station(hotel_goal):
    global robotStatus
    if hotel_goal == 'Station' or hotel_goal in service_dict['stationCode']:
        setting_robot_mission(hotel_goal)
        return
    else:
        # Feedback UI, it can't take task now.
        hotelGoalCB_Pub.publish('busy')
        # tts info
        rospy.loginfo('tts: 机器人电量过低.. 请输入 0100 返回充电区')
        announce = unicode('机器人电量过低.. 请输入 0100 返回充电区', 'utf-8')
        ttsPub.publish(announce)
        hotelGoalCB_Pub.publish('busy')
        rospy.logwarn("[NC] Should Return Station. Reject Other Task.")
        robotStatus.mission = None
        pub_robot_status()
        return


def goal_correct(task_input, service_dict):
    predefine_list = service_dict['hotelGoals']

    for raw, one_list in enumerate(predefine_list):
        if task_input in one_list:
            return True, task_input
        elif task_input[1:] in service_dict['stationCode']:
            return True, 'Station'
        elif task_input[1:] in one_list:
            return True, task_input[1:]
        elif re.match("EVW[12345678]", task_input):
            return True, task_input
        elif re.match("EVW[12345678]S", task_input):
            return True, task_input
        else:
            pass
    return False, task_input


# Monitor and check the delivery task input. Reject or Update the mission accordingly.
# If reject, simply feedback 'busy'. Otherwise, broadcast if through robot_status (by setting_robot_mission()).
def goal_monitor(received_goal):
    global lock, robotStatus
    # lock the global variable: robotStatus
    lock.acquire()
    # TODO: Add Node Test & Connection Test, PASS: Proceed, Fail: Rejecct goal & Warning.

    # Stage 0. With Mission, Reject
    if robotStatus.mission is not None:
        hotelGoalCB_Pub.publish('busy')
        rospy.logwarn("[NC] AMR is busy. Reject Task: " + str(received_goal.data))
        lock.release()
        return

    robotStatus.mission = "Received_Mission"
    pub_robot_status()
    # Reaction Plan
    reaction = {
        'charging': reject_goal,
        'need_charging': return_station,
        'available': setting_robot_mission}
    # Stage 1.
    # Choose the reaction plan according to the current status.
    if robotStatus.status in reaction.keys():

        function_to_call = reaction[robotStatus.status]
        function_to_call(received_goal.data)
        lock.release()
        return
    else:
        hotelGoalCB_Pub.publish('busy')
        rospy.logwarn("[NC] Status Error. Reject Task: " + str(received_goal.data))
        lock.release()
        return


# Check all the available node are online & alive.
def node_monitor():
    # TODO: Node monitor, try 3 mins with 3s interval. Success >> > proceed.Fail >> > Reboot
    # Check the node_list_1
    fine, result = checkNodeExistence(service_dict['node_list_1']+service_dict['node_list_2'])
    if not fine:
        ts = time.time()
        for node, alive in result.items():
            if not alive:
                msg = '[NC] Initlization Failed: ' + node + ' is dead. Reboot the Robot.'
                rospy.logfatal(msg)
                st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
                if rss_on:
                    rss_notification.sent(
                        str(st) + " " + service_dict['AMR_ID'] + msg, '@all')
        msg = '[NC] Initlization Failed. Reboot the Robot.'
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        if rss_on:
            rss_notification.sent(
                str(st) + " " + service_dict['AMR_ID'] + msg, '@all')
        return False
    else:
        return True


def system_health():
    global lock, robotStatus
    lock.acquire()
    if wifi_connected():
        rospy.loginfo('[NC] Init Process: Check WiFi Connection >>> PASS.')
    # TODO: Need Recovery Monitor
    """
    else:

        wifi_monitor_thread = threading.Timer()
        wifi_monitor_thread.daemon = True
        wifi_monitor_thread.start()
        # Block the AMR Service
        robotStatus.mission = 'WiFi_Fail'
        robotStatus.set_status('rss_request')
        pub_robot_status()
    """
    # Wait 5s for all the launch process finished.
    rospy.sleep(5)
    # Successfully
    if node_monitor():
        # Change Status from 'init' to 'charging' or 'available'
        rospy.loginfo('[NC] Init Process: Check subsystem >>> PASS.')
        lock.release()
        return
    # Boost Fail
    else:
        # Trigger the reboot
        rebootPub.publish(30.0)
        # TODO: Remove the #
        # rospy.sleep(120)
        lock.release()
        return


# Check the battery capacity. Update and trigger warning when it's too low.
def battery_monitor(battery_capacity):
    global robotStatus, battery_rss_timestamp
    robotStatus.capacity = battery_capacity.data

    if battery_capacity.data < battery_capacity_threshold and (battery_rss_timestamp is None):
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        if rss_on:
            rss_notification.sent(str(st) + " " + service_dict['AMR_ID'] + ": Low Battery ! PLEASE CHECK.", '@all')
        rospy.logwarn("[rss_server]: Low Battery ! PLEASE CHECK. Request RSS.")
        battery_rss_timestamp = ts
    elif battery_capacity.data >= 20.0:
        battery_rss_timestamp = None
    else:
        pass
    return


# Check whether AMR are in the station and well-connected.
def station_connection_monitor(connection_correct):
    global charging_connected
    if connection_correct:
        rospy.loginfo("[NC]: Confirm that charging station is well-connected. ")
        charging_connected = True
        return
    else:
        rospy.loginfo("[NC]: Confirm that charging station is detached. ")
        charging_connected = False
        return


def pub_robot_status():
    status = {
        # Robot Statue: indicate the stage of task.
        'status': robotStatus.status,
        # Last position of AMR.
        'position': robotStatus.position,
        # hotelGoal, on-going task.
        'mission': robotStatus.mission,
        # Timestamp of goal reception.
        'ts_start': robotStatus.ts_start,
        # Current battery capacity
        'capacity': robotStatus.capacity,
        # Aborted sub-task Counter
        'abortCount': robotStatus.abortCount
              }
    # status_json = json.dumps(status)
    robotStatusPub.publish(str(status))
    return


def resource_monitor():
    global robotStatus, lock
    reboot_time = service_dict['rebootTime']
    while True:
        # Daily Reboot
        if time.localtime().tm_hour == reboot_time and time.localtime().tm_min == 0:
            while True:
                if robotStatus.status in ['charging', 'need_charging', 'available'] and \
                                robotStatus.position is 'station':
                    lock.acquire()
                    rospy.loginfo('[NC] Daily Reboot.')
                    # Change the statue to block new task
                    robotStatus.set_status('init')
                    rospy.loginfo('tts: 执行机器人定期重启')
                    announceMsg = unicode('执行机器人定期重启', 'utf-8')
                    ttsPub.publish(announceMsg)
                    rospy.sleep(5)
                    rebootPub.publish(30.0)
                    rospy.sleep(120)
                    lock.release()
                else:
                    rospy.loginfo("[NC]: Wait for mission complete Then, reboot.")
                    rospy.sleep(10)
        # Critical Resource Monitor: Nodes, connection
        # TODO: Add WiFi Connection Monitor
        else:
            if resource_check_on:
                # Check the node_list_1
                fine, result = checkNodeExistence(service_dict['node_list_1'])
                if not fine:
                    for node, alive in result.items():
                        if not alive:
                            rospy.logwarn('[NC Resource Monitor]' + node + " is dead.")
                            # TODO: Add  node relaunch process.
                # Check the node_list_2
                fine, result = checkNodeExistence(service_dict['node_list_2'])
                if not fine:
                    ts = time.time()
                    for node, alive in result.items():
                        if not alive:
                            msg = '[NC Resource Monitor]' + node + " is dead. Request RSS."
                            rospy.logfatal(msg)
                            st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
                            rss_notification.sent(
                                str(st) + " " + service_dict['AMR_ID'] + msg + " PLEASE CHECK.", '@all')
                rospy.sleep(30)
            else:
                rospy.sleep(60)
            pass


def status_agent():
    global robotStatus, lock
    # Lock the global variable.
    lock.acquire()
    if robotStatus.status == 'init':
        # Default: Lobby
        if charging_connected:
            # Update the position
            robotStatus.position = 'Station'
            # Update the current floor
            robotStatus.floor = service_dict['station_floor']
            if robotStatus.capacity >= battery_capacity_threshold:
                robotStatus.set_status('available')
            else:
                robotStatus.set_status('charging')
        else:
            # TODO: Check the Tag ID for localization.
            # http://docs.ros.org/hydro/api/apriltags_ros/html/msg/AprilTagDetectionArray.html
            # Subscribe the tag_topic, then setting the robot.position.
            # tts info
            rospy.loginfo('tts: 机器人未于充电站内启动，请协助移动至充电站或各楼层电梯口前，完成定位')
            announce = unicode('机器人未于充电站内启动，请协助移动至充电站或各楼层电梯口前，完成定位', 'utf-8')
            ttsPub.publish(announce)
            # Subscribe the tag_topic, then setting the robot.position
            if robotStatus.capacity >= battery_capacity_threshold:
                robotStatus.set_status('available')
                robotStatus.position = 'Lobby'
            else:
                robotStatus.set_status('need_charging')
                robotStatus.position = 'Lobby'
        pub_robot_status()
        # Release the global variable.
        lock.release()
        return
    # Reset the status, not position.
    else:
        if charging_connected:
            robotStatus.position = 'Station'
            if robotStatus.capacity >= battery_capacity_threshold:
                robotStatus.set_status('available')
            else:
                robotStatus.set_status('charging')
        # For Task Complete Case.
        elif robotStatus.status == "reached":
            rospy.loginfo('[NC] Mission reached, reset status to "available"')
            robotStatus.set_status('available')
        else:
            rospy.loginfo('[NC] reset status to "available"')
            robotStatus.set_status('available')
            
        pub_robot_status()
        # Release the global variable.
        lock.release()
        return


def depart_station(mission_path):
    global lock, robotStatus
    lock.acquire()
    rospy.loginfo('[NC] AMR is going to depart the station.')
    # Subscribe the callback Function
    _sub = rospy.Subscriber('departStationCB', Bool, depart_callback)
    rospy.loginfo('tts: 请小心，机器人将离开充电区 ')
    msg = unicode('请小心, 机器人将离开充电区.', 'utf-8')
    rospy.loginfo("[NC] Departing Station >>> >>> >>> ")
    ttsPub.publish(msg)
    rospy.sleep(3.0)
    departStationPub.publish(True)
    if not mission_path[1:]:
        robotStatus.set_status('reached')
        rospy.loginfo('[NC] Path Finished, Goal Reached.')
    else:

        robotStatus.set_status('moving')
    pub_robot_status()
    # Release the global status.
    lock.release()
    # Wait for Departure Complete
    while robotStatus.position != 'Station_out':
        rospy.sleep(1)
    _sub.unregister()
    rospy.loginfo('[NC] >>> Departure Finished.')
    return mission_path[1:]


def depart_callback(success):
    global lock, robotStatus
    lock.acquire()
    if success:
        robotStatus.position = 'Station_out'
    else:
        robotStatus.position = 'Station_out'
        msg = '[NC] Departure Station Return False.'
        rospy.logwarn(msg)
        sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
    lock.release()
    return


def enter_station(mission_path):
    global lock, robotStatus, task_path
    lock.acquire()
    if pmu_online:
        rospy.loginfo('[NC] AMR is going to enter the Station.')
        # Subscribe the callback Function
        _sub = rospy.Subscriber('/enterStationCB', Bool, entering_callback)
        rospy.loginfo('tts: 机器人到家了..请小心, 机器人将回到充电区. ')
        msg = unicode('机器人到家了..请小心, 机器人将回到充电区..', 'utf-8')
        rospy.loginfo("[NC] Entering Station >>> >>> >>> ")
        ttsPub.publish(msg)
        rospy.sleep(3.0)
        departStationPub.publish(True)
        # Release the global status.
        lock.release()
        # Wait for Entering Complete
        # TODO: It's dangerous here.
        while robotStatus.position != 'Station':
            rospy.sleep(1)
        _sub.unregister()
        pub_robot_status()
        rospy.loginfo('[NC] >>> Entering Finished.')
        if not mission_path[1:]:
            robotStatus.set_status('reached')
            rospy.loginfo('[NC] Path Finished, Goal Reached.')
        lock.release()
        return mission_path[1:]
    else:
        rospy.logwarn('[NC] PMU_Offline, Abort Entering.')

        pub_robot_status()
        if not mission_path[1:]:
            robotStatus.set_status('reached')
            rospy.loginfo('[NC] Path Finished, Goal Reached.')
        lock.release()
        return mission_path[1:]


def entering_callback(success):
    global lock, robotStatus
    lock.acquire()
    if success:
        robotStatus.position = 'Station'
    else:
        robotStatus.position = 'Station'
        msg = '[NC] Entering Station Return False.'
        rospy.logwarn(msg)
        sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
    lock.release()
    return


def move_base_agent(mission_path, time_limit):
    global lock, robotStatus
    lock.acquire()
    # Subscribe the callback Topic
    _sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_callback)
    current_goal = mission_path[0]
    rospy.loginfo('[NC] Current Goal: ' + str(current_goal))
    # Loading the destination coordinate
    pose, yaw_tol, xy_tol = goal_agent(current_goal, goal_dict)
    # Set Tolerance
    # TODO: set_tolerance TEST on real robot
    # set_tolerance(yaw_tol, xy_tol)

    rospy.loginfo('[NC] Sent the goal to move_base.')
    # Publish the move_base goal.
    goalPub.publish(pose)
    # Release the global status.
    robotStatus.set_status('moving')
    pub_robot_status()
    lock.release()
    # Wait for Move_base's confirmation
    counter = 0
    while True:
        if robotStatus.status == 'node_reached':
            robotStatus.position = mission_path.pop(0)
            if robotStatus.position[0:3] == 'EVW' and task_path[0] == 'EVin':
                robotStatus.set_status('waitEV')
            pub_robot_status()
            _sub.unregister()
            # Return the following path.
            return mission_path, True
        elif robotStatus.status == 'retry':
            robotStatus.set_status('moving')
            rospy.loginfo('[NC] Assign move_base goal again.')
            pub_robot_status()
            goalPub.publish(pose)
            robotStatus.abortCount['move_base'] += 1
            counter += 1
        elif counter > time_limit:
            rospy.logwarn('[NC] move_base timeout.')
            _sub.unregister()
            return mission_path, False
        else:
            rospy.sleep(0.1)
            counter += 1


def move_base_callback(msg):
    global robotStatus, lock
    lock.acquire()
    if msg.status.status == 3:
        # node goal reached
        robotStatus.set_status('node_reached')

    elif msg.status.status == 2:
        # move_base goal preempted
        rospy.logwarn("[NC] Move_base Goal preempted.")

    elif msg.status.status == 4:
        # node goal aborted
        rospy.logwarn("[NC] Move_base aborted")
        robotStatus.abortCount['move_base'] += 1
        # Reset Goal
        rospy.loginfo("[NC] Retry Goal!")
        robotStatus.set_status('retry')

        # RSS Notification when aborting 5 times
        if robotStatus.abortCount['move_base'] % 10 == 0:
            msg = '[NC] AMR is blocked. Move_base aborted 10 times. Request RSS attention.'
            rospy.logwarn(msg)
            sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)

            # TODO: PHONE Pushing & Logging Thread
            """
            # push notification to operator
            msg = '机器人被大型障碍物挡住了, 请派服务员前往处理'
            threadY = pushingThread(msg)
            threadY.start()

            # logging
            msg = 'Got stopper obstacles'
            threadX = loggingThread(msg)
            threadX.start()
            """
        # Voice Warning
        tts_msg = unicode('请借过, 我需要多一点空间', 'utf-8')
        ttsPub.publish(tts_msg)

        # Clear costmap
        rospy.loginfo("[NC] Call /move_base/clear_costmaps")
        clear_costmap()

    else:
        # msg.status.status == 1,5,6,7 ...etc
        str_msg = "[NC] Move_base Issue, status = " + str(msg.status.status)
        rospy.logwarn(str_msg)
        sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], str_msg)
        robotStatus.set_status('retry')

    lock.release()
    return


def navi_center():
    global robotStatus, service_dict, rss_notification, path_planner, task_path, goal_dict
    rospy.loginfo("[NC] System Launched.")
    # ====================================================== #
    # 0. Initiation Phase.
    # ====================================================== #
    # ----------   Service Parameters Loading  ------------- #
    service_dict = loading_service_parameter()
    goal_dict = loading_hotel_goal()
    rss_notification.setting(service_dict)
    path_planner.setting(service_dict['hotelGoals'])
    # ----------   RobotStatus Init ------------------------ #
    robotStatus = RobotStatus()
    # ----------   Check System Boost up Successfully ------ #
    rospy.loginfo('[NC] Init Process: Check subsystem.')
    # Check all the necessary node are online and alive.
    # TODO: Enable system_health
    if resource_check_on:
        system_health()
        # Start the "All time resource_monitor": Check node, connection & daily reboot
        resource_monitor_thread = threading.Thread(target=resource_monitor)
        resource_monitor_thread.daemon = True
        resource_monitor_thread.start()

    # ====================================================== #
    # 1. Init >>> Standby Phase
    # ====================================================== #
    # 1-0. Subscribe Topic Relative to AMR Status or Safety:
    # ===== Battery ===== #
    rospy.Subscriber('/capacity', Float32, battery_monitor)
    rospy.Subscriber('/charging/connected', Bool, station_connection_monitor)
    # ===== Safety Monitor  ===== #
    # TODO: Add safety feature
    # PMU, Power Management Unit Monitor
    rospy.Subscriber('/PMU/online', Bool, pmu_monitor)
    # TODO: ADD /EMB /Station
    # rospy.Subscriber('/emergentStop', Bool, emergentStop)
    # rospy.Subscriber('/base/lock', Bool, bumperHit)

    # Initialize position(default or by TopView_Tag), floor and status
    status_agent()
    # Loading the map and position accordingly.
    map_client(robotStatus.floor, service_dict)

    # Register DWA_Planner Service

    # TODO: Check is here other way to replace wait_for
    """
    try:
        rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters", 5.0)
        client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30,
                                                   config_callback=dynamic_reconfigure_cb)
        rospy.loginfo("[NC] AMR Launched Success.")
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("[NC] DWAPlannerROS Service call failed: %s" % (e,))
        rospy.loginfo("[NC] AMR Launched Complete.")
    """
    rospy.loginfo("[NC] AMR Launched Success.")
    # 2. Planning Phase.
    # 2-1. Start the 'Goal_Monitor: 'Subscribe a /hotelGoal.
    # Monitor and check the delivery task input. Reject or Update the mission accordingly.
    # If reject, simply feedback 'busy'. Otherwise, broadcast if through robot_status (by setting_robot_mission()).
    rospy.Subscriber('/hotelGoal', String, goal_monitor)
    
    # 3. Task Execution Phase.
    while not rospy.is_shutdown():
        # TODO: Remove this.
        print("In the loop.")
        # Scene I, Got Mission with task_path.
        if robotStatus.mission is not None and task_path:
            # ================== Main Task Loop ========================================== #
            rospy.loginfo('[NC] Task Path: ' + str(task_path))
            rospy.loginfo('[NC] Current Position: ' + str(robotStatus.position))
            # Receive task path, And Execute it orderly.
            # Case 1, Departure Station
            if robotStatus.position == 'Station' and task_path[0] == 'Station_out':
                task_path = depart_station(task_path)
            # Case 2. Entering Station
            elif robotStatus.position == 'Station_out' and task_path[0] == 'Station':
                task_path = enter_station(task_path)
            # Case 3. Entering EV
            # elif robotStatus.position[0:3] == 'EVW' and task_path[1] == 'EVin':
            elif robotStatus.status is 'waitEV':
                # AMR_ID, status, tid, current_floor, target_floor
                tid = str(round(time.time(), 2))
                current_floor = robotStatus.position[3:] + 'F'
                target_floor = task_path[1][3:] + 'F'
                rospy.loginfo('[NC] Call the EV to ' + current_floor)
                # 3-1. Request EV
                ev_request_confirm = ev_agent_call(service_dict['AMR_ID'], tid, current_floor, target_floor)
                if ev_request_confirm:
                    robotStatus.set_status('waitEV_C')
                    pub_robot_status()
                    # 3-2. Wait for EV reach
                    reach_result = ev_agent_reach(current_floor, service_dict['ev_timeout'])
                    if reach_result is 'correct':
                        rospy.loginfo('[NC] EV reached: '+str(current_floor))
                        robotStatus.set_status('checkingEV')
                        pub_robot_status()
                        # 3-3. Check the EV Space
                        safety_check_result = check_elevator(checkEVPub)
                        if safety_check_result == 'pass':
                            robotStatus.set_status('enteringEV')
                            pub_robot_status()
                            pass
                        elif safety_check_result == 'not pass':
                            # Release the EV
                            release_ev(doorReleasePub, ttsPub)
                            # Wait for 15s and recall
                            robotStatus.set_status('waitEV')
                            pub_robot_status()
                            rospy.sleep(15.0)
                            robotStatus.abortCount['ev_check'] += 1
                            pass
                        elif safety_check_result == 'yield':
                            # Release the EV
                            release_ev(doorReleasePub, ttsPub)
                            # Try the Yield Position
                            task_path.insert(0, robotStatus.position)
                            task_path.insert(0, robotStatus.position+'S')
                            robotStatus.set_status('moving')
                            pub_robot_status()
                            robotStatus.abortCount['ev_check'] += 1
                            pass
                    # 3-2. Wait for EV reach >>> Timeout
                    elif reach_result == 'timeout':
                        msg = '[NC] Call EV >>> No response >>> Timeout.'
                        rospy.logwarn(msg)
                        sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                        pass
                    elif reach_result == 'wrong_floor':
                        msg = '[NC] Call EV >>> Floor Mismatch.'
                        rospy.logwarn(msg)
                        sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                        pass
                else:
                    # TODO: Add action to EV request >>> No response. More than three times.
                    msg = '[NC] Unable to sent request to EV.'
                    rospy.logwarn(msg)
                    sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                    robotStatus.error['ev_no_response'] += 1
            elif robotStatus.status == 'enteringEV':
                # done / aborted / timeout(1200)
                inform_ev = None

                counter = 0
                entering_result = entering_elevator(current_floor, enterEVPub)
                if entering_result == 'done':
                    robotStatus.position = task_path.pop(0)
                    robotStatus.set_status('inEV')
                    pub_robot_status()
                    rospy.loginfo('[NC] >>> Entering EV Done >>> ')
                    rospy.loginfo('[NC] Current Position: ' + str(robotStatus.position))
                    # Try to Inform the EV.
                    inform_ev = None
                    while not inform_ev:
                        if counter < 3:
                            inform_ev = inform_ev_done(service_dict['AMR_ID'], tid, current_floor, target_floor)
                            counter += 1
                            rospy.loginfo('[NC] Call the EV to ' + str(target_floor))
                            robotStatus.set_status('inEV_C')
                            pub_robot_status()
                        else:
                            msg = '[NC] Unable to inform EV that AMR has done entering.'
                            rospy.logwarn(msg)
                            sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                            robotStatus.error['ev_no_response'] += 1
                            break

                elif entering_result == 'abort':
                    task_path.insert(0, robotStatus.position)
                    robotStatus.set_status('moving')
                    pub_robot_status()
                    robotStatus.abortCount['entering'] += 1
                    pass
                elif entering_result == 'timeout':
                    msg = '[NC] Timeout, Unable to Confirm that AMR has done entering.'
                    rospy.logwarn(msg)
                    sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                    # TODO: Could be node fial. Set recovery mode here.
                    pass

            elif robotStatus.status in ['inEV', 'inEV_C']:
                rospy.loginfo('[NC] Task Path: ' + str(task_path))
                # TODO: Start EV Door Monitor Here.
                reach_result = ev_agent_reach(target_floor, service_dict['ev_timeout'])
                if reach_result == 'correct':
                    robotStatus.set_status('inEV_R')
                    pub_robot_status()
                    # 4-2. Wait for EV reach >>> Timeout
                elif reach_result == 'timeout':
                    msg = '[NC] EV can not reach target floor in 10m.'
                    rospy.logwarn(msg)
                    robotStatus.error['ev_no_response'] += 1
                    pub_robot_status()
                    sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                    # >>>   Retry
                    pass
                elif reach_result == 'wrong_floor':
                    msg = '[NC] Receive EV reached. >>> Floor Mismatch.'
                    rospy.logwarn(msg)
                    sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                    # >>>   Retry
                    pass
            elif robotStatus.status == 'inEV_R':
                rospy.loginfo('[NC] Task Path: ' + str(task_path))
                # alightEVPub.publish(target_floor)
                announce = unicode('机器人正要出电梯, 外面的朋友请小心... 外面的朋友请小心.. 机器人正要出电梯. 十分感谢您的配合', 'utf-8')
                ttsPub.publish(announce)
                robotStatus.set_status('alightingEV')
                pub_robot_status()
                # Method 1, Move_base
                if alight_mb_on:
                    # time_limit = 10s (100 * 0.1s)
                    task_path, node_reach = move_base_agent(task_path, 100)
                    if node_reach:
                        robotStatus.set_status('moving')
                        pub_robot_status()
                        pass
                    else:
                        skip_node = task_path.pop(0)
                        rospy.loginfo('[NC] Skip the node: ' + str(skip_node))
                        robotStatus.position = skip_node
                        pub_robot_status()
                        rospy.loginfo('[NC] Task Path: ' + str(task_path))
                        pass
                else:
                    target_floor = task_path[0][3:] + 'F'
                    currentFloorPub.publish(target_floor)
                    # Wait for tts
                    rospy.sleep(1.0)
                    alight_result = alighting_elevator(target_floor, alightEVPub)
                    if alight_result == 'done':
                        current_node = task_path.pop(0)
                        robotStatus.position = current_node
                        robotStatus.set_status('node_reached')
                        pub_robot_status()
                        # 4-2. Wait for EV reach >>> Timeout
                    elif alight_result == 'timeout':
                        msg = '[NC] EV Alighting timeout. Resent alight command.'
                        rospy.logwarn(msg)
                        robotStatus.error['ev_no_response'] += 1
                        sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                        # >>>   Retry
                        pass
                    elif alight_result == 'error':
                        msg = '[NC] EV Alighting Error. Resent alight command.'
                        rospy.logwarn(msg)
                        sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                        # >>>   Retry
                        pass
            else:
                # move_base timeout = 5m (300s)
                task_path, node_reach = move_base_agent(task_path, 3000)
                if node_reach:
                    if robotStatus.mission == robotStatus.position:
                        robotStatus.set_status('reached')
                        pub_robot_status()
                        pass
                else:
                    msg = '[NC] Send Goal to move_base >>> No response >>> Timeout.'
                    rospy.logwarn(msg)
                    sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
                    pass

        # Scene II, Got Mission and task_path is empty.
        elif robotStatus.mission is not None and robotStatus.status == "reached":
            # Inform the User Interface, Mission Complete.
            hotelGoalReachPub.publish(robotStatus.mission)
            # Reset Mission
            robotStatus.mission_done()
            # Reset Status
            status_agent()
            pub_robot_status()
        # No mission => Standby
        else:
            rospy.sleep(0.1)

    # R. Recovery Mode.

    # I. Issue Notification Mode.

if __name__ == '__main__':
    try:
        rospy.init_node("navi_center")
        navi_center()

    except rospy.ROSInterruptException:
        pass