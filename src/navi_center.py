#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import json
import std_srvs.srv
from std_msgs.msg import String, Float32, Float64, Bool
from navi_execution.RobotStatus import RobotStatus
from navi_planning.path_planner import PathPlanner
from navi_execution.map_manager import map_client
from navi_execution.goal_manager import set_tolerance, goal_agent
from rss.node_monitor import *
from rss.weixin_alarm import WXAlarm

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

import dynamic_reconfigure.client
import time
import datetime
import threading


# File Location of the hotel relatived setting.
param_path = rospy.get_param("param_path",
                             '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml')

battery_capacity_threshold = rospy.get_param("battery_capacity_threshold", 20.0)

# Subsystem Switch
rss_on = rospy.get_param("rss_on", False)


# Global Service parameter
service_dict = {}
robotStatus = None

# Status Locker
lock = threading.Lock()

# ------------------------ Auxiliary Function ---------------------------- #
# Init the path_planner
path_planner = PathPlanner()
task_path = []
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
# Map & Position Init
changeNavMapPub = rospy.Publisher('/map_server_nav/reload', String, queue_size=1)
changeAMCLMapPub = rospy.Publisher('/map_server_amcl/reload', String, queue_size=1)
initPosePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

# ====== Action Trigger =====
# A1. Station Type
enterStationPub = rospy.Publisher('/enterStation', Bool, queue_size=1)
departStationPub = rospy.Publisher('/departStation', Bool, queue_size=1)

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


# Loading the service parameter from robot_unique_parameter
def loading_service_parameter():
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)


def setting_robot_mission(hotel_goal):
    global robotStatus, task_path
    task_path = path_planner.path_agent(robotStatus.position, robotStatus.mission)
    # Check the result is not empty.
    if not task_path:
        hotelGoalCB_Pub.publish('busy')
        rospy.loginfo('tts: 无法规划路径，请重新输入正确房号')
        announce = unicode('无法规划路径，请重新输入正确房号', 'utf-8')
        rospy.logwarn("[NC] Path Planning Failed. Reject it And Ask for Correction.")
        # Reset the robot status
        robotStatus.mission_done()
        # Reset the status
        status_agent()
        ttsPub.publish(announce)
        pub_robot_status()
    robotStatus.set_status('received_mission')
    robotStatus.set_mission(hotel_goal)
    pub_robot_status()
    rospy.loginfo('[NC] Goal Monitor Confirm the hotelGoal.')
    # Confirmation of Receiving task.
    rospy.loginfo('[NC] Start the Mission.')
    announce = unicode('好的...感谢有机会为您服务', 'utf-8')
    ttsPub.publish(announce)
    return


# Reject Hotel GOAL
def reject_goal(hotel_goal):
    if robotStatus.status is 'charging':
        # Feedback UI, it can't take task now.
        hotelGoalCB_Pub.publish('busy')
        # tts info
        rospy.loginfo('tts: 机器人电量过低.. 充电中，请稍候再试')
        announce = unicode('机器人电量过低.. 充电中，请稍候再试', 'utf-8')
        rospy.logwarn("[NC] Low Battery. Reject Task: " + str(hotel_goal))
        ttsPub.publish(announce)
        return
    else:
        hotelGoalCB_Pub.publish('busy')
        rospy.logwarn("[NC] AMR is busy. Reject Task: " + str(hotel_goal))
        return


def return_station(hotel_goal):
    global robotStatus
    if hotel_goal == 'station' or hotel_goal in service_dict['stationCode']:
        setting_robot_mission(hotel_goal)
        return
    else:
        # Feedback UI, it can't take task now.
        hotelGoalCB_Pub.publish('busy')
        # tts info
        rospy.loginfo('tts: 机器人电量过低.. 请输入 0100 返回充电区')
        announce = unicode('机器人电量过低.. 请输入 0100 返回充电区', 'utf-8')
        rospy.logwarn("[NC] Should Return Station. Reject Other Task.")
        ttsPub.publish(announce)
        return


# Monitor and check the delivery task input. Reject or Update the mission accordingly.
# If reject, simply feedback 'busy'. Otherwise, broadcast if through robot_status (by setting_robot_mission()).
def goal_monitor(received_goal):
    global lock
    # lock the global variable: robotStatus
    lock.acquire()
    # TODO: Add Node Test & Connection Test, PASS: Proceed, Fail: Rejecct goal & Warning.
    # Reaction Plan
    reaction = {
        'charging': reject_goal,
        'need_charging': return_station,
        'available': setting_robot_mission}
    # Stage 1,
    # Check the hotel_goal
    # Is it in the list ?
    if received_goal.data in service_dict['hotelGoals'][0]:
        hotel_goal = received_goal.data
        rospy.loginfo("[NC] Received hotelGoal: " + hotel_goal)
    elif received_goal.data[1:5] in service_dict['stationCode']:
        hotel_goal = 'Station'
        rospy.loginfo("[NC] Received hotelGoal: " + hotel_goal)
    else:
        hotelGoalCB_Pub.publish('busy')
        rospy.loginfo('tts: 请输入正确的房号')
        announce = unicode('请输入正确的房号', 'utf-8')
        rospy.logwarn("[NC] Reject Task which is Not in the List.")
        ttsPub.publish(announce)
        return False
    # Stage 2, Action
    # Choose the reaction plan according to the current status.
    if robotStatus.status in reaction.keys():
        function_to_call = reaction[robotStatus.status]
        function_to_call(hotel_goal)
    else:
        reject_goal('busy')

    lock.release()
    return True


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
    # TODO: Add WiFi connection Test
    # Wait 5s for all the launch process finished.
    rospy.sleep(5)
    # Successfully
    if node_monitor():
        # Change Status from 'init' to 'charging' or 'available'
        rospy.loginfo('[NC] Init Process: Check subsystem >>> PASS.')
        return
    # Boost Fail
    else:
        # Trigger the reboot
        rebootPub.publish(30.0)
        rospy.sleep(120)
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
    status_json = json.dumps(status)
    robotStatusPub.publish(status_json)
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
            if rss_on:
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
            robotStatus.set_position('Station', service_dict['hotelGoals'])
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
                robotStatus.set_position('Lobby', service_dict['hotelGoals'])
            else:
                robotStatus.set_status('need_charging')
                robotStatus.set_position('Lobby', service_dict['hotelGoals'])
        pub_robot_status()
        # Release the global variable.
        lock.release()
        return
    # Reset the status, not position.
    else:
        if charging_connected:
            robotStatus.set_position('Station', service_dict['hotelGoals'])
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
    # Release the global status.
    robotStatus.set_status('moving')
    pub_robot_status()
    lock.release()
    # Wait for Departure Complete
    while robotStatus.position != 'Station_out':
        rospy.sleep(1)
    _sub.unregister()
    rospy.loginfo('[NC] >>> Departure Finished.')
    pub_robot_status()
    return mission_path.pop(0)


def depart_callback(success):
    global lock, robotStatus
    lock.acquire()
    if success:
        robotStatus.set_position('Station_out')
    else:
        robotStatus.set_position('Station_out')
        msg = '[NC] Departure Station Return False.'
        rospy.logwarn(msg)
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        if rss_on:
            rss_notification.sent(
                str(st) + " " + service_dict['AMR_ID'] + msg, '@all')
    lock.release()
    return


def enter_station(mission_path):
    global lock, robotStatus
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
        lock.release()
        return mission_path.pop(0)
    else:
        rospy.logwarn('[NC] PMU_Offline, Abort Entering.')
        lock.release()
        pub_robot_status()
        return mission_path.pop(0)


def entering_callback(success):
    global lock, robotStatus
    lock.acquire()
    if success:
        robotStatus.set_position('Station')
    else:
        robotStatus.set_position('Station')
        msg = '[NC] Entering Station Return False.'
        rospy.logwarn(msg)
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        if rss_on:
            rss_notification.sent(
                str(st) + " " + service_dict['AMR_ID'] + msg, '@all')
    lock.release()
    return


def move_base_agent(mission_path, client):
    global lock, robotStatus
    lock.acquire()
    current_goal = mission_path[0]
    # Loading the destination coordinate
    pose, yaw_tol, xy_tol = goal_agent(current_goal, service_dict)
    # Set Tolerance
    set_tolerance(yaw_tol, xy_tol, client)

    rospy.loginfo('[NC] Sent a goal to Move_base.')
    # Subscribe the callback Function
    _sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_callback)
    # Publish the move_base goal.
    goalPub.publish(pose)
    # Release the global status.
    robotStatus.set_status('moving')
    pub_robot_status()
    lock.release()
    # Wait for Move_base's confirmation
    while True:
        if robotStatus.status == 'moving_reached':
            robotStatus.set_position(current_goal)
            robotStatus.set_status('moving')
            pub_robot_status()
            _sub.unregister()
            # Return the following path.
            return mission_path.pop(0)
        elif robotStatus.status == 'retry_goal':
            robotStatus.set_status('moving')
            pub_robot_status()
            _sub.unregister()
            # Return the same path
            return mission_path
        else:
            rospy.sleep(1.0)


def move_base_callback(msg):
    global robotStatus, lock
    lock.acquire()
    if msg.status.status == 3:
        # node goal reached
        robotStatus.set_status('moving_reached')
        pub_robot_status()

    elif msg.status.status == 2:
        # move_base goal preempted
        rospy.logwarn("[NC] Move_base Goal preempted.")

    elif msg.status.status == 4:
        # node goal aborted
        rospy.logwarn("[NC] Move_base aborted")
        robotStatus.abortCount[0] += 1
        # RSS Notification when aborting 5 times
        if robotStatus.abortCount[0] % 5 == 0:
            ts = time.time()
            st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            if rss_on:
                rss_notification.sent(str(st) + " "
                                      + service_dict['AMR_ID']
                                      + " : Move_base aborted 5 times. Request RSS attention. ", '@all')

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
        rospy.loginfo("[NC] call /move_base/clear_costmaps")
        clear_costmap()

        # Reset Goal
        rospy.loginfo("[NC] Re-try Goal!")
        robotStatus.set_status('retry_goal')
        pub_robot_status()
    else:
        # msg.status.status == 1,5,6,7 ...etc
        str_msg = "[NC] Move_base Issue, status = " + str(msg.status.status)
        rospy.logwarn(str_msg)
        rospy.loginfo('[NC] Re-try Goal')
        robotStatus.set_status('retry_goal')
        pub_robot_status()
    lock.release()
    return


def navi_center():
    global robotStatus, service_dict, rss_notification, path_planner, task_path
    rospy.loginfo("[NC] Navigation Center Launched.")
    # ====================================================== #
    # 0. Initiation Phase.
    # ====================================================== #
    # ----------   Service Parameters Loading  ------------- #
    service_dict = loading_service_parameter()
    rss_notification.setting(service_dict)
    path_planner.setting(service_dict['hotelGoals'])
    # ----------   RobotStatus Init ------------------------ #
    robotStatus = RobotStatus()
    # ----------   Check System Boost up Successfully ------ #
    rospy.loginfo('[NC] Init Process: Check subsystem.')
    # Check all the necessary node are online and alive.
    # TODO: Add WiFi Connection Test in system_health
    # system_health()
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
    # TODO: ADD /EMB
    # rospy.Subscriber('/emergentStop', Bool, emergentStop)
    # rospy.Subscriber('/base/lock', Bool, bumperHit)

    # Initialize position(default or by TopView_Tag), floor and status
    status_agent()
    # Loading the map and position accordingly.
    map_client(robotStatus.floor, service_dict)

    # TODO: Check the purpose of this service:
    # Register DWA_Planner Service
    rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters")
    client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30,
                                               config_callback=dynamic_reconfigure_cb)
    rospy.loginfo("[NC] AMR Launched Success.")

    # 2. Planning Phase.
    # 2-1. Start the 'Goal_Monitor: 'Subscribe a /hotelGoal.
    # Monitor and check the delivery task input. Reject or Update the mission accordingly.
    # If reject, simply feedback 'busy'. Otherwise, broadcast if through robot_status (by setting_robot_mission()).
    rospy.Subscriber('/hotelGoal', String, goal_monitor)
    
    # 3. Task Execution Phase.
    while not rospy.is_shutdown():
        # Scene I, Got Mission with task_path.
        if robotStatus.mission is not None and task_path:
            # ================== Main Task Loop ========================================== #
            # Receive task path, And Execute it orderly.
            # Case 1, Departure Station
            if robotStatus.position == 'Station' and task_path[1] == 'Station_out':
                task_path = depart_station(task_path)
            # Case 2. Entering EV
            elif robotStatus.position == 'Station_out' and task_path[1] == 'Station':
                task_path = enter_station(task_path)
            # Case N. Move_base Moving
            else:
                task_path = move_base_agent(task_path, client)
            """ TODO: Crossing Map by EV
            # Case 3. Entering EV
            elif robotStatus.position[0:3] == 'EVW' and task_path[1] == 'EVin':
                rospy.loginfo('[NC] AMR is going to Entering the EV.')
            """
        # Scene II, Got Mission and task_path is empty.
        elif robotStatus.mission is not None and not task_path:
            robotStatus.set_status('reached')
            pub_robot_status()
            status_agent()
            pub_robot_status()
            hotelGoalCB_Pub(robotStatus.mission)
        # No mission => Standby
        else:
            rospy.sleep(1.0)

    # R. Recovery Mode.

    # I. Issue Notification Mode.




if __name__ == '__main__':
    try:
        rospy.init_node("navi_center")
        navi_center()

    except rospy.ROSInterruptException:
        pass