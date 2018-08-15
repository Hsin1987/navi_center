#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import *
from move_base_msgs.msg import *

goalReachedPub = rospy.Publisher('/move_base/result', MoveBaseActionResult,queue_size=1)
elevatorReachedPub = rospy.Publisher('/elevatorCB', String, queue_size=1)

checkPub = rospy.Publisher('/checkEVcb', String, queue_size=1)

continuousCheckPub = rospy.Publisher('/continuousSafetyCheckResultUnsafe', Bool, queue_size=1)

enterEVpub = rospy.Publisher('/enterEVcb', Bool, queue_size=1)
alightEVpub = rospy.Publisher('/alightEVcb', Bool, queue_size=1)
hotelGoalCBPub = rospy.Publisher('/hotelGoalCB', String, queue_size=1)
emergentStopPub = rospy.Publisher('/emergentStop', Bool, queue_size=1)

enterStationCBPub = rospy.Publisher('/enterStationCB', Bool, queue_size=1)
departStationCBPub = rospy.Publisher('/departStationCB', Bool, queue_size=1)


def goalAbort():
    global goalReachedPub
    msg = MoveBaseActionResult()
    msg.status.status = 4
    goalReachedPub.publish(msg)
    return


def goalReached():
    global goalReachedPub
    msg = MoveBaseActionResult()
    msg.status.status = 3
    goalReachedPub.publish(msg)
    return


def elevatorReached():
    global elevatorReachedPub
    print("Input the floor. EX: 2F")
    key = raw_input()
    elevatorReachedPub.publish(key)
    return


def evSafetyCheckResponse_Safe():
    global checkPub

    checkPub.publish('pass')
    return


def evSafetyCheckResponse_Unsafe():
    global checkPub
    checkPub.publish('not pass')
    return

def evSafetyCheckResponse_yield():
    global checkPub
    checkPub.publish('yield')
    return

def evSafetyCheck_CountinuousCheck_Fail():
    global checkPub
    msg = Bool()
    msg.data = False
    continuousCheckPub.publish(msg)
    return

def enterResponse():
    global enterEVpub
    msg = Bool()
    msg.data = True
    enterEVpub.publish(msg)
    return

def alightResponse():
    global alightEVpub
    msg = Bool()
    msg.data = True
    alightEVpub.publish(msg)
    return

def emergentStop():
    global emergentStopPub
    msg = Bool()
    msg.data = True
    emergentStopPub.publish(msg)


def enterStationResponse():
    global enterStationCBPub
    msg = Bool()
    msg.data = True
    enterStationCBPub.publish(msg)
    return


def departStationResponse():
    global departStationCBPub
    msg = Bool()
    msg.data = True
    departStationCBPub.publish(msg)
    return

pubBindings = {
    '0': goalAbort,
    '1': goalReached,
    '2': elevatorReached,
    '3': evSafetyCheckResponse_Safe,
    '4': evSafetyCheckResponse_Unsafe,
    '5': evSafetyCheckResponse_yield,
    '6': evSafetyCheck_CountinuousCheck_Fail,
    '7': enterResponse,
    '8': alightResponse,
    'en': enterStationResponse,
    'de': departStationResponse,
    'em': emergentStop
}

msg = """
0. moveBaseGoal  Abort
1: moveBaseGoal   Reached
===================================================
2: evReached
===================================================
3: evSafetyCheck >>> Safe
4: evSafetyCheck >>> Unsafe
5: evSafetyMonitor >>> Yield
===================================================
6. ev_monitor >>>> Unsafe
===================================================
7: evEnter >>> Done
8: evAlight >>> Done
===================================================
en: enterStation >>> Done
de: departStation >>> Done
em: emergentStop
"""

if __name__=="__main__":

    rospy.init_node('fake_data')
    while not rospy.is_shutdown():
        try:
            sys.stderr.write("\x1b[2J\x1b[H")
            print msg
            key = raw_input()
            if key in pubBindings.keys():
                pubBindings[key]()
            else:
                print('Please choose the right one.')
        except:
            print 'shit happens'
    rospy.spin()

