#!/usr/bin/python
# coding=utf-8
"""
ev_agent :
1. request EV to carry AMR.
2. re-try request when fail
3. trigger safety check,
4. Start ev interaction according to the check result.
5. Start entering
"""


import requests
import time
import rospy
from std_msgs.msg import String, Bool
import requests
from src.navi_execution.basic_function import loading_service_parameter

service_dict = loading_service_parameter()
ev_ip = 'http://' + service_dict['elevator_server_ip']

reach_info = None
check_result = None


def inform_ev_done(robot_id, tid, current_floor, target_floor):
    retry = 10
    payload = {'robot_id': robot_id,
               'tid': tid,
               'current_floor': current_floor,
               'target_floor': target_floor}

    uri = ev_ip + ':8080/entering_done'
    for counter in range(retry):
        rospy.loginfo('[NC] Try to inform EV entering has Done.')
        try:
            req = requests.get(uri, params=payload, timeout=3)
            if req.ok:
                try:
                    tid_callback = req.text.encode('UTF8')
                    if tid_callback == payload['tid']:
                        rospy.loginfo('[NC] Inform EV Success.')
                        return True
                    else:
                        rospy.logwarn('[NC][EV_Agent] Response tid mis-match. Retry.')
                        time.sleep(1.0)
                        pass
                except:
                    rospy.loginfo('[NC][EV_Agent] Weired Response message:  ' + str(req.text))
                    pass
            else:
                time.sleep(1.0)
        except requests.exceptions.Timeout:
            rospy.loginfo('[NC][EV_Agent] request timeout: ' + str(counter) + '/' + str(retry))
            pass
        except requests.exceptions.RequestException as e:
            # catastrophic error. bail.
            rospy.loginfo('[NC][EV_Agent] request exceptions: ' + str(e))

    return False


def ev_agent_call(robot_id, tid, current_floor, target_floor):
    retry = 10
    payload = {'robot_id': robot_id,
               'tid': tid,
               'current_floor': current_floor,
               'target_floor': target_floor}

    uri = ev_ip + ':8080/call'
    for counter in range(retry):
        try:
            req = requests.get(uri, params=payload, timeout=3)
            if req.ok:
                try:
                    tid_callback = req.text.encode('UTF8')
                    if tid_callback == payload['tid']:
                        return True
                    else:
                        rospy.logwarn('[NC][EV_Agent] Response tid mis-match. Retry.')
                        time.sleep(1.0)
                        pass
                except:
                    rospy.loginfo('[NC][EV_Agent] Weired Response message:  ' + str(req.text))
                    pass
            else:
                time.sleep(1.0)
        except requests.exceptions.Timeout:
            rospy.loginfo('[NC][EV_Agent] request timeout: ' + str(counter) + '/' + str(retry))
            pass
        except requests.exceptions.RequestException as e:
            # catastrophic error. bail.
            rospy.loginfo('[NC][EV_Agent] request exceptions: ' + str(e))

    return False


def ev_agent_reach(floor, timeout_counter):
    global reach_info
    reach_sub = rospy.Subscriber('/elevatorCB', String, reach_callback)

    counter = 0
    while True:
        if reach_info is None:
            time.sleep(0.1)
            counter += 1

        elif reach_info == floor:
            reach_sub.unregister()
            reach_info = None
            return 'correct'

        elif counter >= timeout_counter:
            reach_sub.unregister()
            reach_info = None
            return 'timeout'
        else:
            reach_sub.unregister()
            reach_info = None
            return 'wrong_floor'


def reach_callback(msg):
    global reach_info

    reach_info = msg.data


# ===============================================

check_pass = None
entering_finish = None
ev_unsafe = None
alight_finish = None

# TODO : Add Door Monitor
# TODO : Add Moving Direction Monitor


# 1. Check EV.
def check_elevator(check_pub):
    """
    :return:
    :"pass": check pass
    :"not pass": not pass
    :"timeout": no result in 1m.
    """
    global check_result
    # Subscribe the result message
    check_result_sub = rospy.Subscriber('/checkEVcb', String, check_elevator_callback)
    check_pub.publish()
    rospy.loginfo('[NC] Request EV Check.')

    loop_counter = 0
    while True:
        if check_result is None:
            loop_counter += 1
            rospy.sleep(0.1)

        elif check_result == 'pass':
            # Finish all the topic request.
            rospy.loginfo('[NC] Check Pass, Take the EV.')
            check_result_sub.unregister()
            check_result = None
            return 'pass'
        elif check_result == 'not pass':
            check_result_sub.unregister()
            rospy.loginfo('[NC] Ignore the EV')
            check_result = None
            return 'not pass'
        elif check_result == 'yield':
            rospy.loginfo('[NC] Yield the path')
            check_result_sub.unregister()
            check_result = None
            return 'yield'
        # Over 1m
        elif loop_counter > 900:
            check_result_sub.unregister()
            return 'timeout'
        else:
            loop_counter += 1
            rospy.sleep(0.1)


def check_elevator_callback(msg):
    global check_result
    check_result = msg.data


# ===================================================================================================== #
def entering_elevator(floor, entering_pub):
    global entering_finish

    # Subscribe the result message
    entering_result_sub = rospy.Subscriber('/enterEVcb', Bool, entering_elevator_callback)
    safety_monitor_sub = rospy.Subscriber('/continuousSafetyCheckResultUnsafe', Bool, safety_monitor_client)
    entering_pub.publish(floor)
    rospy.loginfo('[NC] >>> Start Entering EV. >>> ')

    loop_counter = 0
    while True:
        if entering_finish is True:
            # Finish all the topic request.
            entering_result_sub.unregister()
            safety_monitor_sub.unregister()
            return 'done'
        # TODO: check the result enterEVcb when abort
        elif entering_finish is False or ev_unsafe is True:
            entering_result_sub.unregister()
            safety_monitor_sub.unregister()
            return 'abort'
        # Over 2m
        elif loop_counter > 1200:
            entering_result_sub.unregister()
            safety_monitor_sub.unregister()
            return 'timeout'
        else:
            loop_counter += 1
            rospy.sleep(0.1)


def entering_elevator_callback(result):
    global entering_finish
    entering_finish = result.data


def safety_monitor_client(result):
    global ev_unsafe
    ev_unsafe = result.data


# ===================================================================================================== #
def entering_elevator_done(robot_id, tid, current_floor, target_floor):
    retry = 10
    payload = {'robot_id': robot_id,
               'tid': tid,
               'current_floor': current_floor,
               'target_floor': target_floor}

    uri = ev_ip + ':8080/entering_done'
    for counter in range(retry):
        try:
            req = requests.get(uri, params=payload, timeout=3)
            if req.ok:
                try:
                    tid_callback = req.text.encode('UTF8')
                    if tid_callback == payload['tid']:
                        return True
                    else:
                        rospy.logwarn('[NC][EV_Agent] Response tid mis-match. Retry.')
                        time.sleep(1.0)
                        pass
                except:
                    rospy.loginfo('[NC][EV_Agent] Weired Response message:  ' + str(req.text))
                    pass
            else:
                time.sleep(1.0)

        except requests.exceptions.Timeout:
            rospy.loginfo('[NC][EV_Agent] request timeout: ' + str(counter) + '/' + str(retry))
            pass

        except requests.exceptions.RequestException as e:
            # catastrophic error. bail.
            rospy.loginfo('[NC][EV_Agent] request exceptions: ' + str(e))

    return False


# ===================================================================================================== #

def alighting_elevator(floor, alighting_pub):
    global entering_finish
    # Subscribe the result message
    alighting_result_sub = rospy.Subscriber('/alightEVcb', Bool, alight_elevator_callback)
    alighting_pub.publish(floor)
    rospy.loginfo('[NC] >>> Start alighting EV. >>> ')

    loop_counter = 0
    while True:
        if alight_finish is True:
            # Finish all the topic request.
            alighting_result_sub.unregister()
            return 'done'
        # TODO: check the result enterEVcb when abort
        elif alight_finish is False:
            alighting_result_sub.unregister()
            return 'error'
        # Over 2m
        elif loop_counter > 600:
            alighting_result_sub.unregister()
            return 'timeout'
        else:
            loop_counter += 1
            rospy.sleep(0.1)


def alight_elevator_callback(result):
    global alight_finish
    alight_finish = result.data


# ============================
def release_ev(door_release_pub, tts_pub):
    # Release the EV
    rospy.loginfo('[NC] Pass & Release the EV control')
    door_release_pub.publish()
    # tts
    rospy.loginfo("[NC] Pass the EV and recall later.")
    announce = unicode('电梯你们先用吧, 我搭下一次', 'utf-8')
    tts_pub.publish(announce)