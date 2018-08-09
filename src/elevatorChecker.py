#!/usr/bin/env python

from threading import Timer
from threading import Lock

import rospy
from std_msgs.msg import *
from apriltags_ros.msg import *
from NavigationCenter.moving_filter import  moving_filter


lock = Lock()

checkCBPub = rospy.Publisher('/checkEVcb', Bool, queue_size=1)
continuousSafetyCheckResultUnsafePub = rospy.Publisher('/continuousSafetyCheckResultUnsafe', Bool, queue_size=1)

countTotal = 0
safety_check_threshold = 0.9
safety_monitor_threshold = 0.9

# these are continuous safety check parameters
# Continuous Safety Check Start Flag
safety_monitor_start = False
safety_check_start = False

# Safety Monitor Period Setting
safety_monitor_period = 3.0
safety_check_period = 3.0

check_buffer = []
safety_buffer = []


def tags_agent(tags_array):
    global countTotal
    global safety_monitor_threshold

    global safety_check_start, safety_monitor_start
    global check_buffer, safety_buffer

    if safety_check_start:
        temp_list = [0] * 5
        # Pre-process:
        for item in tags_array.detections:
            if item.id in range(1,6):
                temp_list[item.id - 1] = 1
            else:
                rospy.loginfo("[EC] Found unlisted tag: " + str(item.id))
        check_buffer.append(temp_list)

    if safety_monitor_start:
        temp_list = [0] * 5
        # Pre-process:
        for item in tags_array.detections:
            if item.id in range(1, 6):
                temp_list[item.id - 1] = 1
            else:
                rospy.loginfo("[EC] Found unlisted tag: " + str(item.id))
       
        safety_buffer.append(temp_list)
        monitor_score = moving_filter(safety_buffer)

        if monitor_score < safety_monitor_threshold:
            lock.acquire()
            rospy.loginfo("[EC] Safety Monitor report unsafe! Score: " + str(round(monitor_score, 2)))
            continuousSafetyCheckResultUnsafePub.publish(True)
            safety_monitor_start = False
            lock.release()
        else:
            # rospy.loginfo("[EC] Safety Monitor Score: " + str(round(monitor_score, 2)))
            pass

    countTotal += 1
    return


def check_elevator(msg):
    global safety_check_start, check_buffer

    # Reset the Flag and counter
    safety_check_start = True
    check_buffer = []

    rospy.loginfo("[EC] Safety Check Starts.")
    # Give it 1s to count.
    t = Timer(2, check_t0_score)
    t.daemon = True
    t.start()
    return


def check_t0_score():
    global countTotal, checkCBPub
    # Filter Trigger
    global safety_check_start, check_buffer
    rospy.loginfo('len(check_buffer)' + str(len(check_buffer)))
    # Calculate the score
    score_filtered = moving_filter(check_buffer)

    if score_filtered >= safety_check_threshold:
        rospy.loginfo("[EC] Safety Check report safe. Score: " + str(round(score_filtered, 2)))
        checkCBPub.publish(True)
    else:
        rospy.loginfo("[EC] Safety Check report unsafe. Score: " + str(round(score_filtered, 2)))
        checkCBPub.publish(False)

    # Reset the trigger and buffer
    safety_check_start = False
    check_buffer = []
    return


def safety_monitor_starter(msg):
    global safety_monitor_start, safety_buffer
    rospy.loginfo("[EC] Safety monitor Starts.")
    safety_monitor_start = True
    safety_buffer = []

    t2 = Timer(safety_monitor_period, safety_monitor_end)
    t2.daemon = True
    t2.start()
    return


def safety_monitor_end():
    global safety_monitor_start
    global safety_monitor_start, safety_buffer

    # Reset the Flag Ending the safety_monitor
    rospy.loginfo("[EC] Safety_Monitor OFF.")
    safety_monitor_start = False
    safety_buffer = []
    return


if __name__ == "__main__":
    rospy.init_node('elevator_checker')
    rospy.Subscriber('/camera_rear/tag_detections', AprilTagDetectionArray, tags_agent)
    rospy.Subscriber('/checkEV', Bool, check_elevator)

    rospy.Subscriber('/continuousSafetyCheckStart', Bool, safety_monitor_starter)

    rospy.spin()
