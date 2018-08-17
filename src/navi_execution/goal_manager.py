
import rospy
from geometry_msgs.msg import Pose
import dynamic_reconfigure.client
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from move_base_msgs.msg import MoveBaseActionResult


# Global Service parameter

goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

current_pose = None
refined_goal = PoseStamped()
correction_done = False

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


# Calculate the Correction Pose
def pose_agent(msg):
    global current_pose
    if current_pose is None:
        current_pose = Pose()
        current_pose = msg.pose.pose
    else:
        pass


def ori_agent(task_goal, goal_dict):
    global correction_done
    pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_agent)

    # Access the Pose Setting
    pose_setting = goal_dict[task_goal]
    ori_z = pose_setting[2]
    ori_w = pose_setting[3]
    wide_yaw_tolerance = pose_setting[4]
    
    if not wide_yaw_tolerance:
        rospy.logwarn("[NC] No Wide Yaw Tolerance Setting for " + str(task_goal))
        return False
    
    # Wait for Current Pose Update.
    counter = 0
    while current_pose is None:
        if counter < 30:
            rospy.sleep(0.1)
        else:
            rospy.logwarn("[NC] Can't update the current Pose. Abort Ori Adjustment.")
            return False

    quaternion = (
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w)

    current_yaw = euler_from_quaternion(quaternion)[2]

    quaternion = (0, 0, ori_z, ori_w)

    setting_yaw = euler_from_quaternion(quaternion)[2]

    yaw_list = [setting_yaw]
    # Setting Yaw List:
    if setting_yaw <= 3.14159265359:
        yaw_list.append(setting_yaw + 3.14159265359)
    else:
        yaw_list.append(setting_yaw - 3.14159265359)

    rospy.loginfo('[NC] Current YAW: ' + str(current_yaw))
    # Find the closest pose for docking.
    closest = [0, 100]
    for ori in yaw_list:
        ori_diff = abs(current_yaw - ori)
        if ori_diff <= closest[1]:
            closest[0] = ori
            closest[1] = ori_diff
    rospy.loginfo('[NC] Adjust YAW: ' + str(closest[0]))
    # Setting the pose
    qua = quaternion_from_euler(0, 0, closest[0])
    refined_goal.header.frame_id = 'map'
    refined_goal.pose.position = current_pose.position
    refined_goal.pose.orientation.x = qua[0]
    refined_goal.pose.orientation.y = qua[1]
    refined_goal.pose.orientation.z = qua[2]
    refined_goal.pose.orientation.w = qua[3]
    rospy.loginfo('Update the Ori-correction Goal.   ' + str(closest[0]))
    set_tolerance(False, 1.0)
    
    # Subscribe the move_base result:
    corr_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, correction_callback)
    goalPub.publish(refined_goal)

    # Give it 5s to correct.
    counter = 0
    while not correction_done:
        if counter >= 5:
            break
        counter += 1
        rospy.sleep(1)

    pose_sub.unregister()
    corr_sub.unregister()
    return True


def correction_callback(msg):
    global correction_done
    if msg.status.status == 3:
        # node goal reached
        correction_done = True
    else:
        correction_done = False





    




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