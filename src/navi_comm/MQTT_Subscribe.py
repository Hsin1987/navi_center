# Subscriber.py
import paho.mqtt.client as mqtt
count = 0

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("elevator_status", qos=1)

def on_message(client, userdata, msg):
    global count
    print(str(count) + " " + msg.topic+" "+str(msg.payload))
    count += 1

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.30.120", 1883, 60)
client.loop_forever()





