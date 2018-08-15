#!/usr/bin/env python
# coding=utf-8

import cherrypy
import requests

import rospy
import sys
import time
import threading
from std_msgs.msg import *

# elevatorIP = 'http://192.168.2.201:8080/'
# elevatorIP = 'http://192.168.30.33:8080/'
elevatorIP = 'http://192.168.65.200:8080/'
# elevatorIP = 'http://192.168.30.59:8080/'
#phoneServerIP = 'http://192.168.25.210:8080/'
phoneServerIP = 'http://192.168.65.100:8080/'
password = ''

# TODO: move IP
# TODO: floor


class myRequestsThread (threading.Thread):
    def __init__(self, uri, ttsPublishFunc):
        threading.Thread.__init__(self)
        self.uri = uri
        self.ttsPublish = ttsPublishFunc

    def run(self):
        s = requests.Session()
        for attemp in range(10):
            try:
                r = s.get(self.uri)
            except:
                print("[nc_pc] Exception in myRequestThread for uri: "+self.uri)
                announceMsg = unicode('呼叫电梯请求失败', 'utf-8')
                self.ttsPublish(announceMsg)
                rospy.loginfo("[nc_pc] Exception in myRequestThread for uri: "+self.uri)
                continue
            else:
                print('[nc_pc] got http 200 ok for uri:' + self.uri)
                rospy.loginfo('[nc_pc] got http 200 ok for uri: ' + self.uri)
                break
        pass


class PC_server(object):
    elevatorReachedPub = rospy.Publisher('/elevatorCB',Bool,queue_size=1)

    @cherrypy.expose
    def index(self):
        return "Hello world!"
    
    @cherrypy.expose
    def reached(self):
        print 'elevator reached'
        rospy.loginfo('[nc_pc] Elevator reached')
        self.elevatorReachedPub.publish(True)
        return "GOOOOOOOOOOOOD!"
    
    @cherrypy.expose
    def shutdown(self):  
        cherrypy.engine.exit()

def doorOpen(msg):
    print '[nc_pc] in doorOpen()'
    rospy.loginfo('[nc_pc] in doorOpen()')
    #s1 = requests.Session()
    #r1 = s1.get(elevatorIP + 'open')
    uri = str(elevatorIP + 'open')
    print(uri)
    threadX = myRequestsThread(uri, ttsPublish)
    threadX.start()
    return

def doorClose(msg):
    print '[nc_pc] in doorClose()'
    rospy.loginfo('[nc_pc] in doorClose()')
    #s2 = requests.Session()
    #r2 = s2.get(elevatorIP + 'close')
    uri = str(elevatorIP + 'close')
    print(uri)
    threadX = myRequestsThread(uri, ttsPublish)
    threadX.start()
    return

def doorRelease(msg):
    print '[nc_pc] in doorRelease()'
    rospy.loginfo('[nc_pc] in doorRelease()')
    #s2 = requests.Session()
    #r2 = s2.get(elevatorIP + 'close')
    uri = str(elevatorIP + 'release_button')
    print(uri)
    threadX = myRequestsThread(uri, ttsPublish)
    threadX.start()
    return

def ttsPublish(announceMsg):
    ttsPub.publish(announceMsg)
    return

def callElevator(msg):
    print '[nc_pc] in callElevator(), call to floor: ', msg.data
    rospy.loginfo('[nc_pc] in callElevator(), call to floor:' + str(msg.data))
    # print str(msg.data)
    #s0 = requests.Session()
    #r0 = s0.get(elevatorIP + 'call?floor=' + str(msg.data))
    uri = str(elevatorIP + 'call?floor=' + str(msg.data))
    print('[nc_pc] uri = '+uri)
    rospy.loginfo('[nc_pc] uri = '+uri)
    threadX = myRequestsThread(uri, ttsPublish)
    threadX.start()
    return

def updatePW(msg):
    global password
    print '[nc_pc] in updatePW()'
    rospy.loginfo('[nc_pc] in updatePW()')
    password = msg.data
    return


def goalReachCB(msg):
    global password
    print '[nc_pc] in goalReachCB(), msg.data: ',msg.data
    rospy.loginfo('[nc_pc] in updatePW()'+str(msg.data))
    if password != '':
        if msg.data != 'Lobby' and msg.data != '100' and msg.data != '199' :
            uri = str(phoneServerIP + 'robocall?roomId=' + msg.data + '&pw=' + password)
            #uri = str(phoneServerIP + 'robocall?roomId=15&pw=' + password)
            print(uri)

            #s = requests.Session()
            #r = s.get(phoneServerIP + 'robocall?roomId=15&pw=' + password)
            threadX = myRequestsThread(uri, ttsPublish)
            threadX.start()
        else:
            print '[nc_pc] returned to Lobby'
    else:
        print 'No password'
    password = ''

    return


if __name__ == '__main__':
    rospy.init_node('NaviCenter',anonymous = False)
    rospy.Subscriber('/doorOpen',Bool,doorOpen)
    rospy.Subscriber('/doorClose',Bool,doorClose)
    rospy.Subscriber('/doorRelease',Bool,doorRelease)
    rospy.Subscriber('/callEV',Int16,callElevator)
    rospy.Subscriber('/hotelGoalReached',String,goalReachCB)
    rospy.Subscriber('/pw',String,updatePW)
    ttsPub = rospy.Publisher('/web/tts_request',String,queue_size=1)
    # rospy.spin()

    # if rospy.is_shutdown:
    #     print 'ros node dead'

    cherrypy.server.socket_host = '0.0.0.0'
    cherrypy.server.socket_port = 7070
    cherrypy.server.thread_pool = 10
    cherrypy.quickstart(PC_server())
