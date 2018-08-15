import re
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client
from basic_function import loading_service_parameter
# Global Service parameter
# service_dict = {}
param_path = '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml'

goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)


def dynamic_reconfigure_callback(config):
    print(config)
    rospy.loginfo("[nc_ros] dynamicReconfigureCallback() is called")


def set_tolerance(free_yaw_tolerance, xy_tol):
    if free_yaw_tolerance:
        try:
            rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters", 5.0)
            client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30)
            client.update_configuration({"yaw_goal_tolerance": 3.1416})
            client.update_configuration({"xy_goal_tolerance": xy_tol})
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("[NC] DWAPlannerROS Service call failed: %s" % (e,))
    else:
        try:
            rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters", 5.0)
            client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30)
            client.update_configuration({"yaw_goal_tolerance": 0.1})
            client.update_configuration({"xy_goal_tolerance": xy_tol})
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("[NC] DWAPlannerROS Service call failed: %s" % (e,))


def goal_agent(task_goal, goal_dict):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    try:
        pose_setting = goal_dict[task_goal]
        goal.pose.position.x = pose_setting[0]
        goal.pose.position.y = pose_setting[1]
        goal.pose.orientation.z = pose_setting[2]
        goal.pose.orientation.w = pose_setting[3]
        wide_yaw_tolerance = pose_setting[4]
        xy_goal_tolerance = pose_setting[5]
    except:
        rospy.logwarn("[NC] Can access goal setting. Check: " + str(task_goal))
        return None, None, None
    return goal, wide_yaw_tolerance, xy_goal_tolerance

"""
if __name__ == '__main__':
    rospy.init_node('goal_manager')
    # rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters")
    # client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30,
    #                                            config_callback=dynamic_reconfigure_callback)
    try:
        service_dict = loading_service_parameter()
        pose, yaw_tol, xy_tol = goal_agent('202', service_dict)
        set_tolerance(yaw_tol, xy_tol)
        goalPub.publish(pose)
    except rospy.ROSInterruptException:
        pass

"""