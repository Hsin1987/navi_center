#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String


def call(msg):
    print msg

from navi_infra.navi_server import alighting_elevator
# Alighting Test
alight_elevator_pub = rospy.Publisher('/alightEV', String, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('alighting_test')
    rospy.Subscriber('/alightEV', String, call)
    rospy.sleep(2)

    result = alighting_elevator("1F", alight_elevator_pub)
    print result
    rospy.spin()




# entering_test
"""
from navi_infra.ev_manager import entering_elevator
enter_ev_pub = rospy.Publisher('/enterEV', String, queue_size=1)
if __name__ == '__main__':
    rospy.init_node('entering_test')
    rospy.Subscriber('/enterEV', String, call)
    rospy.sleep(2)

    result = entering_elevator("1F", enter_ev_pub)
    print result
    rospy.spin()
"""

# check_test
""""
from navi_infra.ev_manager import check_elevator
check_pub = rospy.Publisher('/checkEV', Bool, queue_size=1)

def call(msg):
    print msg


if __name__ == '__main__':
    rospy.init_node('ev_test')
    rospy.Subscriber('/checkEV', Bool, call)
    rospy.sleep(2)

    result = check_elevator(check_pub)
    print result
    rospy.spin()
"""