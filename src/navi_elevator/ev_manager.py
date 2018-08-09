import rospy
import re
from std_msgs.msg import Bool

#
# 0. Call the EV to "_F" floor.
def call_elevator(robot_position):
    if re.match("EVW[12345678]", robot_position):
        rospy.loginfo('[NC] AMR Out, Call EV to '+str(robot_position[-1])+'F.')
    elif robot_position == 'EVin':
        rospy.loginfo('[NC] AMR In EV, Call EV to ' + str(robot_position[-1]) + 'F.')

# TODO : Add Door Monitor

# TODO : Add Moving Direction Monitor


# 1. Check EV.
def check_elevator():
    check_pub = rospy.Publisher('/checkEV', Bool, queue_size=1)
    check_pub.publish(True)
    check_pub.unregister()
    rospy.loginfo('[NC] Request EV Check.')


def entering_elevator()



if __name__ == '__main__':
    rospy.init_node('ev_manager')
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_callback)
    move_base_agent()
    rospy.spin()