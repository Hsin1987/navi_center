#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Pose2D,  PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *

def goalSub(msg):
    print 'new goal ', msg.pose.position
    return

def callEVsub(msg):
    print 'elevator is going to ', msg.data
    return

def checkSub(msg):
    print 'check elevator'
    return

def enterEVsub(msg):
    print 'enter elevator', msg.data
    return

def alightEVsub(msg):
    print 'alight elevator', msg.data
    return

def changeMap(msg):
    print msg.data
    return

def changeAmclMap(msg):
    print msg.data
    return

def hotelGoalCB(msg):
    print msg.data
    return

def doorOpen(msg):
    print 'door open'
    return


def doorClose(msg):
    print 'door close'
    return


def enterStation(msg):
    print 'enterStation', msg.data
    return


def departStation(msg):
    print 'departStation', msg.data
    return


def goalCancel(msg):
    print 'goalCancel', msg
    return


def initPose(msg):
    print 'initPose', msg.pose.pose.position
    return


def hotelGoalReachedCB(msg):
    print 'hotelGoalReachedCB', msg
    return


def doorRelease(msg):
    print 'doorRelease', msg.data
    return


def amr_status_agent(msg):
    print 'amr_status_agent', msg.data
    return


def currentFloor(msg):
    print 'currentFloor', msg
    return


def tts_request(msg):
    print 'tts_request'
    return

def reqDeepReboot(msg):
    print 'reqDeepReboot', msg
    return

if __name__=="__main__":
    rospy.init_node('fake_data_sub')

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalSub)
    rospy.Subscriber('/move_base/cancel', GoalID, goalCancel)
    rospy.Subscriber('/callEV',Int16,callEVsub)
    rospy.Subscriber('/checkEV',Bool, checkSub)
    rospy.Subscriber('/enterEV', String, enterEVsub)
    rospy.Subscriber('/alightEV', String, alightEVsub)
    rospy.Subscriber('/map_server_nav/reload', String, changeMap)
    rospy.Subscriber('/map_server_amcl/reload', String, changeAmclMap)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, initPose)
    rospy.Subscriber('/hotelGoalCB', String, hotelGoalCB)
    rospy.Subscriber('/hotelGoalReached', String, hotelGoalReachedCB)
    rospy.Subscriber('/doorOpen', Bool, doorOpen)
    rospy.Subscriber('/doorClose', Bool, doorClose)
    rospy.Subscriber('/doorRelease', Bool, doorRelease)
    rospy.Subscriber('/amr_status_agent', String, amr_status_agent)
    rospy.Subscriber('/currentFloor', String, currentFloor)
    rospy.Subscriber('/web/tts_request', String, tts_request)

    # For Charging Process
    rospy.Subscriber('/enterStation', Bool, enterStation)
    rospy.Subscriber('/departStation', Bool, departStation)

    # For reboot
    rospy.Subscriber('/reqDeepReboot', Float64, reqDeepReboot)

    rospy.spin()


"""
goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
goalCancel = rospy.Publisher('/move_base/cancel',GoalID,queue_size = 1)
callEVpub = rospy.Publisher('/callEV',Int16,queue_size=1)
checkEVpub = rospy.Publisher('/checkEV',Bool,queue_size=1)
enterEVpub = rospy.Publisher('/enterEV',String,queue_size=1)
alightEVpub = rospy.Publisher('/alightEV',String,queue_size=1)
changeNavMapPub = rospy.Publisher('/map_server_nav/reload',String, queue_size=1)
changeAmclMapPub = rospy.Publisher('/map_server_amcl/reload',String, queue_size=1)
initPosePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
hotelGoalCBpub = rospy.Publisher('/hotelGoalCB',String,queue_size=1)
hotelGoalReachPub = rospy.Publisher('/hotelGoalReached',String,queue_size=1)
doorOpenPub = rospy.Publisher('/doorOpen',Bool,queue_size=1)
doorClosePub = rospy.Publisher('/doorClose',Bool,queue_size=1)
doorReleasePub = rospy.Publisher('/doorRelease',Bool,queue_size=1)
robotStatusPub = rospy.Publisher('/amr_status_agent', String, queue_size=1, latch=True)

enterStationPub = rospy.Publisher('/enterStation', Bool, queue_size=1)
departStationPub = rospy.Publisher('/departStation', Bool, queue_size=1)

ttsPub = rospy.Publisher('/web/tts_request', String, queue_size=1)
currentFloorPub = rospy.Publisher('/currentFloor', String, queue_size=1, latch=True)

continuousSafetyCheckStartPub = rospy.Publisher('/continuousSafetyCheckStart', Bool, queue_size=1)

rebootPub = rospy.Publisher('/reqDeepReboot', Float64, queue_size=1)

"""