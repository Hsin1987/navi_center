#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import json
from std_msgs.msg import String, Float32, Float64, Bool
from navi_planning.hotelGoals import GoalMsg
from navi_execution.RobotStatus import RobotStatus
from rss.weixin_alarm import WXAlarm
import time
import datetime
import threading


param_path = rospy.get_param("~param_path",
                             '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml')

battery_capacity_threshold = rospy.get_param("battery_capacity_threshold", 20.0)

# Global Service parameter
service_dict = {}
robotStatus = None

# Status Locker
lock = threading.Lock()

# ------------------------ Auxiliary Function ---------------------------- #
rss_notification = WXAlarm()
battery_rss_timestamp = None
charging_connected = False

# ===== Topic Publisher ===== #

# AMR machine reboot trigger
rebootPub = rospy.Publisher('/reqDeepReboot', Float64, queue_size=1)

# AMR Status Report Agent
robotStatusPub = rospy.Publisher('/amr_status_agent', String, queue_size=1, latch=True)

# Voice Channel
ttsPub = rospy.Publisher('/web/tts_request', String, queue_size=1)

# navigation_execution relative topic publisher
hotelGoalCB_Pub = rospy.Publisher('/hotelGoalCB', String, queue_size=1)


# Loading the service parameter from robot_unique_parameter
def loading_service_parameter():
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)


def planning_path(hotel_goal):
    global robotStatus
    robotStatus.set_status('planning')
    pub_robot_status()
    # TODO: 20180718  Add path planner here.




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
        planning_path(hotel_goal)
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


# Pre-process for
def goal_monitor(received_goal):
    global lock
    # lock the global variable: robotStatus
    lock.acquire()
    # Reaction Plan
    reaction = {
        'charging': reject_goal,
        'need_charging': return_station,
        'available': planning_path}

    # Check the hotel_goal in the list:
    if received_goal.data in service_dict['hotelGoals'][0]:
        hotel_goal = received_goal.data
        rospy.loginfo("[NC] Received hotelGoal: " + hotel_goal)
    elif received_goal.data[1:4] in service_dict['stationCode']:
        hotel_goal = 'station'
        rospy.loginfo("[NC] Received hotelGoal: " + hotel_goal)
    else:
        right_goal = False
        hotel_goal = received_goal.data[1:4]
        for goal_list in service_dict['hotelGoals']:
            if hotel_goal in goal_list:
                right_goal = True
                break
        if right_goal:
            rospy.loginfo("[NC] Received hotelGoal: " + hotel_goal)
        else:
            hotelGoalCB_Pub.publish('busy')
            rospy.loginfo('tts: 请输入正确的房号')
            announce = unicode('请输入正确的房号', 'utf-8')
            rospy.logwarn("[NC] Reject Task which is Not in the List.")
            ttsPub.publish(announce)
            return

    # Choose the reaction plan according to the current status.
    if robotStatus.status in reaction.keys():
        functionToCall = reaction[robotStatus.status]
        functionToCall(hotel_goal)
    else:
        reject_goal('busy')

    lock.release()
    return True


# Check all the available node are online & alive.
def node_monitor():
    # TODO: Node monitor, try 3 mins with 3s gap.Success >> > proceed.Fail >> > Reboot
    return True


def system_health():
    global robotStatus
    # Successfully
    if node_monitor():
        # Change Status from 'init' to 'charging' or 'available'
        rospy.loginfo('[NC] Init Process: Check subsystem >>> PASS.')
        return
    # Boost Fail
    else:
        # TODO: export the failure list.
        rospy.logerr("[NC] Init Process: Check subsystem >>> FAIL, Reboot the robot.")
        # Trigger the reboot
        rebootPub.publish(30.0)
        rospy.sleep(120)
        return


def battery_monitor(battery_capacity):
    global robotStatus, battery_rss_timestamp
    robotStatus.capacity = battery_capacity.data
    ts = time.time()
    if battery_capacity.data < battery_capacity_threshold and (battery_rss_timestamp is None):
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
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


def status_initiaization():
    global robotStatus, lock
    # Lock the global variable.
    lock.acquire()

    # Default: Lobby
    if charging_connected:
        robotStatus.set_position('station', service_dict['hotelGoals'])
        if robotStatus.capacity >= battery_capacity_threshold:
            robotStatus.set_status('available')
        else:
            robotStatus.set_status('charging')
    else:
        # TODO: Check the Tag ID for localization.
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


def navi_center():
    global robotStatus, service_dict, rss_notification
    rospy.loginfo("[NC] Navigation Center Launched.")
    # ====================================================== #
    # 0. Initiation Phase.
    # ====================================================== #
    # ----------   Service Parameters Loading  ------------- #
    service_dict = loading_service_parameter()
    rss_notification.setting(service_dict)
    # ----------   RobotStatus Init ------------------------ #
    robotStatus = RobotStatus()
    # ----------   Check System Boost up Successfully ------ #
    rospy.loginfo('[NC] Init Process: Check subsystem.')
    # Check all the necessary node are online and alive.
    system_health()
    # ====================================================== #
    # 1. Init >>> Standby Phase
    # ====================================================== #
    # 1-0. Subscribe Topic Relative to AMR Status or Safety:
    # ===== Battery ===== #
    rospy.Subscriber('/capacity', Float32, battery_monitor)
    rospy.Subscriber('/charging/connected', Bool, station_connection_monitor)
    # ===== Safety Monitor  ===== #
    # TODO: Add safety feature
    # rospy.Subscriber('/emergentStop', Bool, emergentStop)
    # rospy.Subscriber('/base/lock', Bool, bumperHit)

    # Initialize position(default or by TopView_Tag) and status
    status_initiaization()
    rospy.loginfo("[NC] AMR Launched Success.")

    # 2. Planning Phase.
    # 2-1. Subscribe a /hotelGoal.
    rospy.Subscriber('/hotelGoal', String, goal_monitor)
    # 2-N. un-Subscribe /hotelGoal

    # 3. Task Execution Phase.

    # R. Recovery Mode.

    # I. Issue Notification Mode.

    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node("navi_center")
        navi_center()
    except rospy.ROSInterruptException:
        pass