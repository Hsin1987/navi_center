#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf

import dynamic_reconfigure.client

# M1. Move_Base Goal
goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

current_pose = None
refined_goal = PoseStamped()

status = "moving_reached"


def set_tolerance(free_yaw_tolerance, xy_tol):
    if free_yaw_tolerance:
        try:
            rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters", 5.0)
            client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30)
            client.update_configuration({"yaw_goal_tolerance": 3.1416})
            client.update_configuration({"xy_goal_tolerance": xy_tol})
            rospy.loginfo("[NC] DWAPlannerROS Update tolerance DONE")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("[NC] DWAPlannerROS Service call failed: %s" % (e,))
    else:
        try:
            rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters", 5.0)
            client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30)
            client.update_configuration({"yaw_goal_tolerance": 0.1})
            client.update_configuration({"xy_goal_tolerance": xy_tol})
            rospy.loginfo("[NC] DWAPlannerROS Update tolerance DONE")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("[NC] DWAPlannerROS Service call failed: %s" % (e,))


def move_base_agent():
    global status
    # Loading the destination coordinate
    # Wait for Move_base's confirmation
    while True:
        if status == 'moving_reached':
            pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_agent)

            counter = 0
            while current_pose is None:
                if counter < 600:
                    rospy.sleep(0.1)
                else:
                    rospy.logwarn('Can get the current pose.')
                    break
            set_tolerance(False, 1.0)
            goalPub.publish(refined_goal)
            pose_sub.unregister()
            status = "adjust_ori"
            # Return the following path.
            return


# Calculate the Correction Pose
def pose_agent(msg):
    global current_pose
    if current_pose is None:
        current_pose = msg.pose.pose
        quaternion = (
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        rospy.loginfo("current euler: " + str(euler))
        yaw = euler[2]
        rospy.loginfo('Current YAW.' + str(yaw))
        # Find the closest pose for docking.
        closest = [0, 100]
        for ori in [0, 1.57079632679, 3.14159265359, 4.71238898038, 6.28318530718]:
            ori_diff = abs(yaw - ori)
            rospy.loginfo('Diff.   ' + str(ori_diff))
            rospy.loginfo('closest[1].   ' + str(closest[1]))
            if ori_diff < closest[1]:
                rospy.loginfo('Update.   ' + str(ori_diff))
                closest[0] = ori
                closest[1] = ori_diff

        qua = quaternion_from_euler(0, 0, closest[0])
        refined_goal.header.frame_id = 'map'
        refined_goal.pose.position = current_pose.position
        refined_goal.pose.orientation.x = qua[0]
        refined_goal.pose.orientation.y = qua[1]
        refined_goal.pose.orientation.z = qua[2]
        refined_goal.pose.orientation.w = qua[3]
        rospy.loginfo('Update the Ori-correction Goal.   ' + str(closest[0]))
    else:
        pass


def move_base_callback(msg):
    rospy.loginfo(status)
    rospy.loginfo(msg)
    set_tolerance(False, 0.1)
    return


if __name__ == '__main__':
    rospy.init_node('goal_manager')
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_callback)
    move_base_agent()
    rospy.spin()