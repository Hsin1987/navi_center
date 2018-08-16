#!/usr/bin/env python
# coding=utf-8

import cherrypy
import requests
import rospy
import time
import threading
from std_msgs.msg import Bool, String
from navi_execution.basic_function import loading_service_parameter
from rss.weixin_alarm import WXAlarm, sent_rss_notification

rss_on = rospy.get_param("rss_on", False)

# Global Service parameter
service_dict = {}
rss_notification = WXAlarm()

elevatorIP = 'http://192.168.65.200:8080/'
phoneServerIP = 'http://192.168.65.100:8080/'
password = ''

ttsPub = rospy.Publisher('/web/tts_request', String, queue_size=1)
elevatorReachedPub = rospy.Publisher('/elevatorCB', String, queue_size=1)


def http_service(target_ip, payload, function, retry=10):
    uri = target_ip + function
    for counter in range(retry):
        try:
            req = requests.get(uri, params=payload, timeout=10)
            if req.ok:
                try:
                    tid_callback = req.text.encode('UTF8')
                    if tid_callback == payload['tid']:
                        rospy.loginfo('[NS] Http Service: Success.')
                        return True
                    else:
                        rospy.logwarn('[NS] Http Service: Response tid mis-match. Retry.')
                        rospy.sleep(1.0)
                        pass
                except:
                    rospy.loginfo('[NS] Http Service: Weired Response message:  ' + str(req.text))
                    pass
            else:
                rospy.sleep(1.0)
        except requests.exceptions.Timeout:
            rospy.loginfo('[NS] Http Service: Request timeout: ' + str(counter) + '/' + str(retry))
            pass
        except requests.exceptions.RequestException as e:
            # catastrophic error. bail.
            rospy.loginfo('[NS] Http Service: Request exceptions: ' + str(e))
    return False


class NaviServer(object):
    @cherrypy.expose
    def reached(self, robot_id, floor):
        if robot_id == service_dict['AMR_ID']:
            rospy.loginfo('[NS] Elevator reached: ' + str(floor))
            self.elevatorReachedPub.publish(floor)
            return "OK"
        else:
            rospy.loginfo('[NS] Elevator reached: ' + str(floor))
            self.elevatorReachedPub.publish(floor)
            return "wrong_robot_id"
    
    @cherrypy.expose
    def shutdown(self):  
        cherrypy.engine.exit()


def release_elevator(msg):
    global elevatorIP
    rospy.loginfo('[NS] Release EV Control.')
    tid = round(time.time(), 2)

    payload = {
        'robot_id': service_dict['AMR_ID'],
        'tid': tid
    }

    release_success = http_service(elevatorIP, payload, 'release', 10)
    if not release_success:
        msg = '[NS]: Fail to inform the EV release command. Just For Record.'
        sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
        rospy.logwarn(msg)
    return


def update_password(msg):
    global password
    rospy.loginfo('[NS] Received Password: '+str(msg.data))
    password = msg.data
    return


def goal_reach_inform(msg):

    global password
    room_id = msg.data

    if room_id.startswith('EV') or room_id.startswith('Station') or room_id.startswith('Lobby'):
        rospy.loginfo('[NS] Receive goal_reach: ' + str(room_id))
        rospy.loginfo('[NS] Pass')
    else:
        tid = round(time.time(), 2)
        if password == '':
            msg = '[NS]: Need to inform the guest. But the PW is empty. ROOM_ID: ' + str(room_id)
            sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
            rospy.logwarn(msg)

        payload = {
            'roomId': room_id,
            'pw': password,
            'tid': tid
        }

        call_success = http_service(phoneServerIP, payload, 'robocall', 10)
        if not call_success:
            msg = '[NS]: Fail to inform the guest. Request RSS. ROOM_ID: ' + str(room_id) + "; PW: " + str(password)
            sent_rss_notification(rss_on, rss_notification, service_dict['AMR_ID'], msg)
            rospy.logwarn(msg)

    # Reset the PW
    password = ''
    return


if __name__ == '__main__':
    rospy.init_node('navi_server', anonymous=False)
    # Loading the service setting
    service_dict = loading_service_parameter()
    rss_notification.setting(service_dict)

    elevatorIP = 'http://' + service_dict['elevator_server_ip'] + ':8080/'
    phoneServerIP = 'http://' + service_dict['robocall_server_ip'] + ':8080/'

    rospy.Subscriber('/doorRelease', Bool, release_elevator)

    rospy.Subscriber('/hotelGoalReached', String, goal_reach_inform)
    rospy.Subscriber('/pw', String, update_password)

    cherrypy.server.socket_host = '0.0.0.0'
    cherrypy.server.socket_port = 7070
    cherrypy.server.thread_pool = 10
    cherrypy.quickstart(NaviServer())

    rospy.spin()

