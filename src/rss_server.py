#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from rss.weixin_alarm import WXAlarm
import time, datetime
import threading
import yaml

# AMR ID
AMR_ID = "AMR_TEST"

# For Task
task_start = None
task_max_time = rospy.get_param('task_max_time', 1200)  # 20 minute

# For EV
callEV_start = None
callEV_max_time = rospy.get_param('callEV_max_time', 300)  # 5 minute
enterEV_start = None
enterEV_max_time = rospy.get_param('enterEV_max_time', 120)  # 2 minute

# For Station
enterStation_start = False

rss_notification = WXAlarm()
battery_rss_timestmap = None

# EMG_Alarm
e_raised = False

def battery(msg):
    global battery_rss_timestmap
    ts = time.time()
    if msg.data < 20.0 and (battery_rss_timestmap is None):
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        rss_notification.sent(str(st) + " " + AMR_ID + ": Low Battery ! PLEASE CHECK.", '@all')
        rospy.logwarn("[rss_server]: Low Battery ! PLEASE CHECK. Request RSS.")
        battery_rss_timestmap = ts
    elif msg.data > 20.0:
        battery_rss_timestmap = None
    else:
        pass


def emergentStop(msg):
    if msg.data == True:
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        rss_notification.sent(str(st) + " "  + AMR_ID + ": Manually interrupt task ! PLEASE CHECK.", '@all')
        rospy.logwarn("[rss_server]: Manually interrupt task ! PLEASE CHECK.")


def hotelGoalCB(msg):
    global task_start
    if msg.data == "success":
        # Logging the Task Start timestamp.
        task_start = time.time()
        rospy.loginfo("[rss]: Start Mission timeout counting.")
    return


def hotelGoalReachedCB(msg):
    global task_start
    rospy.loginfo("[rss]: Finish Mission timeout counting.")
    task_start = None
    return


def taskMonitor():
    # Wait for system boot up
    global task_start, task_max_time

    while True:
        if task_start != None and time.time() - task_start > task_max_time:
            ts = time.time()
            st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            rospy.logwarn("[rss_server]: Task Lasts for too long. Request RSS.")
            rss_notification.sent(str(st) + " "  + AMR_ID + ": Task Lasts for too long. Please check. ! ", '@all')
            task_start = None
            rospy.sleep(1)
        else:
            rospy.sleep(5)


def callEV(msg):
    global callEV_start
    rospy.loginfo("[rss]: EV request timeout counting.")
    callEV_start = time.time()
    return


def evCB(msg):
    global callEV_start
    rospy.loginfo("[rss]: Finish EV request timeout counting.")
    callEV_start = None
    return


def enterEV(msg):
    global enterEV_start
    rospy.loginfo("[rss]: Entering EV timeout counting.")
    enterEV_start = time.time()
    return


def enterEVCB(msg):
    global enterEV_start
    rospy.loginfo("[rss]: Finish Entering EV timeout counting.")
    enterEV_start = None
    return


def evMonitor():
    # Wait for system boot up
    global callEV_start, callEV_max_time, enterEV_start, enterEV_max_time

    while True:
        if callEV_start != None and time.time() - callEV_start > callEV_max_time:
            ts = time.time()
            st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            rospy.logwarn("[rss_server]: EV request timeout. Request RSS.")
            rss_notification.sent(str(st) + " " + AMR_ID + ": EV request timeout. Please check ! ", '@all')
            callEV_start = None
            rospy.sleep(1)

        elif enterEV_start != None and time.time() - enterEV_start > enterEV_max_time:
            ts = time.time()
            st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            rospy.logwarn("[rss_server]Entering EV timeout. Request RSS.")
            rss_notification.sent(str(st) + " " + AMR_ID +": Entering EV timeout. Please check ! ", '@all')
            enterEV_start = None
            rospy.sleep(1)

        else:
            rospy.sleep(5)


def enterStationCB(msg):
    if msg.data:
        rospy.loginfo("[rss]: Entering station Succeed.")
    else:
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        rospy.logwarn("[rss_server]Entering Station error. Request RSS.")
        rss_notification.sent(str(st) + " " + AMR_ID + ": Entering Station Error. Request RSS ! ", '@all')
    return


def pmu_monitor(msg):
    if not msg.data:
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        rospy.logwarn("[rss]: PMU offline. Request on-site reboot.")
        rss_notification.sent(str(st) + " " + AMR_ID + ": PMU offline. Request on-site reboot ! ", '@all')
    return


def emg_monitor(pushed):
    global e_raised
    if pushed.data and not e_raised:
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        rospy.logwarn("[rss]: EMG Buttom Pushed.")
        rss_notification.sent(str(st) + " " + AMR_ID + ": EMG Buttom Pushed. Contact User! ", '@all')
        e_raised = True
    elif not pushed.data and e_raised:
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        rospy.logwarn("[rss]: EMG Buttom Release.")
        rss_notification.sent(str(st) + " " + AMR_ID + ": EMG Buttom Release! ", '@all')
        e_raised = False
    return


def main():
    global AMR_ID
    rospy.init_node('rss_server')

    # ----------   Parameters Loading  ------------ #
    # LOCAL paramters loading from package /param
    param_path = rospy.get_param("~param_path",
                                 '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml')
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    param_dict = yaml.load(params_raw)
    AMR_ID = param_dict['AMR_ID']

    rss_notification.setting(param_dict)

    # Task Monitor
    taskMonitorThread = threading.Thread(target=taskMonitor)
    taskMonitorThread.start()

    # ev Monitor
    evMonitorThread = threading.Thread(target=evMonitor)
    evMonitorThread.start()

    # Sub Basic Info
    # rospy.Subscriber('/currentFloor', String, current_Floor)
    rospy.Subscriber('/capacity', Float32, battery)
    # rospy.Subscriber('/amr_status_agent', String, amr_status)

    # Subscribe Safety Alarm
    rospy.Subscriber('/emergentStop', Bool, emergentStop)
    # rospy.Subscriber('/base/lock', Bool, bumperHit)

    # Sub Task
    rospy.Subscriber('/hotelGoalCB', String, hotelGoalCB)

    rospy.Subscriber('/hotelGoalReached', String, hotelGoalReachedCB)

    # Sub EV request
    rospy.Subscriber('/callEV', Int16, callEV)
    rospy.Subscriber('/elevatorCB', Bool, evCB)

    # Sub EV entrance
    rospy.Subscriber('/enterEV', String, enterEV)
    rospy.Subscriber('/enterEVcb', Bool, enterEVCB)

    # Sub Station entrance
    rospy.Subscriber('/enterStationCB', Bool, enterStationCB)

    # Battery Monitor entrance
    rospy.Subscriber('/PMU/online', Bool, pmu_monitor)

    # Battery Monitor entrance
    rospy.Subscriber('/EMG_alarm', Bool, emg_monitor)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


