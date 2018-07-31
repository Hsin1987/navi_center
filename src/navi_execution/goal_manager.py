import re
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client

# Global Service parameter
# service_dict = {}
param_path = '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml'

goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)


def dynamic_reconfigure_callback(config):
    print(config)
    rospy.loginfo("[nc_ros] dynamicReconfigureCallback() is called")


# Loading the service parameter from robot_unique_parameter
def loading_service_parameter():
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)


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


def goal_agent(task_goal, service_setting):
    goal = PoseStamped()
    goal.header.frame_id = 'map'

    if re.match("EVW[12345678]", task_goal):
        pose_setting = service_setting['EVW_pose'][0]
        goal.pose.position.x = pose_setting[0]
        goal.pose.position.y = pose_setting[1]
        goal.pose.orientation.z = pose_setting[2]
        goal.pose.orientation.w = pose_setting[3]
        wide_yaw_tolerance = pose_setting[4]
        xy_goal_tolerance = pose_setting[5]
    elif re.match("EVW[12345678]S", task_goal):
        pose_setting = service_setting['EVWS_pose'][0]
        goal.pose.position.x = pose_setting[0]
        goal.pose.position.y = pose_setting[1]
        goal.pose.orientation.z = pose_setting[2]
        goal.pose.orientation.w = pose_setting[3]
        wide_yaw_tolerance = pose_setting[4]
        xy_goal_tolerance = pose_setting[5]
    elif task_goal == 'Station':
        pose_setting = service_setting['Station_pose'][0]
        goal.pose.position.x = pose_setting[0]
        goal.pose.position.y = pose_setting[1]
        goal.pose.orientation.z = pose_setting[2]
        goal.pose.orientation.w = pose_setting[3]
        wide_yaw_tolerance = pose_setting[4]
        xy_goal_tolerance = pose_setting[5]
    elif task_goal == 'Station_out':
        pose_setting = service_setting['Station_out_pose'][0]
        goal.pose.position.x = pose_setting[0]
        goal.pose.position.y = pose_setting[1]
        goal.pose.orientation.z = pose_setting[2]
        goal.pose.orientation.w = pose_setting[3]
        wide_yaw_tolerance = pose_setting[4]
        xy_goal_tolerance = pose_setting[5]
    elif task_goal == 'Lobby':
        pose_setting = service_setting['Lobby_pose'][0]
        goal.pose.position.x = pose_setting[0]
        goal.pose.position.y = pose_setting[1]
        goal.pose.orientation.z = pose_setting[2]
        goal.pose.orientation.w = pose_setting[3]
        wide_yaw_tolerance = pose_setting[4]
        xy_goal_tolerance = pose_setting[5]
    # If task_goal is a room number
    elif task_goal.isdigit():
        # Room: 1 ~ 22 ; Index: 0 ~ 21
        if task_goal in service_setting['special_room_index'][0]:
            room_index = int(task_goal[-2:]) + service_setting['special_room_index'][1][0] - 1
            pose_setting = service_setting['room_pose'][room_index]
            goal.pose.position.x = pose_setting[0]
            goal.pose.position.y = pose_setting[1]
            goal.pose.orientation.z = pose_setting[2]
            goal.pose.orientation.w = pose_setting[3]
            wide_yaw_tolerance = pose_setting[4]
            xy_goal_tolerance = pose_setting[5]
        else:
            room_index = int(task_goal[-2:]) - 1
            pose_setting = service_setting['room_pose'][room_index]
            goal.pose.position.x = pose_setting[0]
            goal.pose.position.y = pose_setting[1]
            goal.pose.orientation.z = pose_setting[2]
            goal.pose.orientation.w = pose_setting[3]
            wide_yaw_tolerance = pose_setting[4]
            xy_goal_tolerance = pose_setting[5]
    else:
        print("Sth wrong")
    return goal, wide_yaw_tolerance, xy_goal_tolerance


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

