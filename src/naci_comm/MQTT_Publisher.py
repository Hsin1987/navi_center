# Publisher.py
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import *



class publish:
    def __init__(self, topic):
        self.mqttc = mqtt.Client("python_pub")
        self._g_cst_ToMQTTTopicServerIP = "192.168.30.120"
        # port
        self._g_cst_ToMQTTTopicServerPort = 1883
        topic_name = topic
        self._g_cst_MQTTTopicName = topic_name
        rospy.Subscriber(topic_name, String, self.remote_controller)
        print("MOM, I'm here")
        rospy.spin()

    def remote_controller(self, msg):
        try:
            self.mqttc.connect(self._g_cst_ToMQTTTopicServerIP, self._g_cst_ToMQTTTopicServerPort)
            self.mqttc.publish(self._g_cst_MQTTTopicName, msg.data)
            rospy.sleep(0.1)
            print("DAD, I'm here")
        except:
            print("[nc] Exception :" )



if __name__ == '__main__':
    rospy.init_node('elevator_RC', anonymous=False)
    publish('my_topic')

"""
    while not rospy.is_shutdown():
        try:
            publish('my_topic')
        except rospy.ROSInterruptException:
            pass
"""
