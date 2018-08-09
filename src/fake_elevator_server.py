import yaml
import paho.mqtt.client as mqtt
param_path = '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml'

import time
import threading
import json

daily_reboot = 3

elevator_status = {
    'status': 'standby',
    'mission': None,
    'task_floor': None,
}
lock = threading.Lock()

# Loading the service parameter from robot_unique_parameter
def loading_service_parameter():
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)


def status_agent(client):
    # Convert the status to string
    global elevator_status
    while True:
        msg = json.dumps(elevator_status)
        client.publish('elevator_status', qos=1, payload=msg)
        print('one out')
        time.sleep(1.0)


def command_monitor():
    global elevator_status, lock
    status = ['standby', 'on-mission', 'rebooting']
    mission = [None , 'to', 'carry_to', 'hold_open']
    task_floor = range(1, 8)

    while True:
        for i, j in enumerate(status):
            lock.acquire()
            elevator_status['status'] = status[i]
            elevator_status['mission'] = mission[i]
            elevator_status['task_floor'] = task_floor[i]
            print(elevator_status)
            lock.release()
        time.sleep(1.0)

# {Status: Standby, On-Mission, Daily Reboot}
# {Mission: None, to _F, Carry to _F, Hold_Open}
# Listen to command
# {Switch Flag]


if __name__ == '__main__':

    # Loading IP setting
    service_dict = loading_service_parameter()
    # Publish State in 1 Hz
    robocall_ip = service_dict['robocall_server_ip']
    elevator_ip = service_dict['elevator_server_ip']
    # Connect to the MQTT broker
    print('Build mqtt client')
    client = mqtt.Client(client_id=elevator_ip)
    client.connect(robocall_ip, 1883)
    # Wait for connection establish
    time.sleep(4.0)

    # Status Agent, Publish EV State in 1 Hz
    command_monitor_thread = threading.Thread(target=command_monitor)
    command_monitor_thread.daemon = True
    command_monitor_thread.start()

    # Status Agent, Publish EV State in 1 Hz
    ev_monitor_thread = threading.Thread(target=status_agent(client))
    ev_monitor_thread.daemon = True
    ev_monitor_thread.start()


