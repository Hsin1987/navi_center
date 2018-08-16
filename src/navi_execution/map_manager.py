import rospy
import rospkg
import re
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from basic_function import loading_service_parameter

# Map & Position Init
changeNavMapPub = rospy.Publisher('/map_server_nav/reload', String, queue_size=1)
changeAMCLMapPub = rospy.Publisher('/map_server_amcl/reload', String, queue_size=1)
currentFloorPub = rospy.Publisher('/currentFloor', String, queue_size=1, latch=True)
initPosePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)


def init_pose_on_map(floor, service_setting):
    init_pose = PoseWithCovarianceStamped()
    init_pose_setting = service_setting['init_pose']
    # Universal Setting
    init_pose.header.frame_id = 'map'
    init_pose.pose.covariance[0] = 0.25
    init_pose.pose.covariance[7] = 0.25
    init_pose.pose.covariance[35] = 0.066

    # Deal with the standard floor:
    if floor in service_setting['station_floor']:
        floor = 0
    elif floor in service_setting['standard_floor']:
        # Choose the second setting for standard floor.
        floor = 1
    else:
        rospy.logwarn('[NC map_client] Initialize Pose Failure. Receive a wrong floor input.')
        rospy.logwarn('[NC map_client] input = ' + str(floor))
        return None
    # Fill in the pose setting
    init_pose.pose.pose.position.x = init_pose_setting[floor][0]
    init_pose.pose.pose.position.y = init_pose_setting[floor][1]
    init_pose.pose.pose.orientation.z = init_pose_setting[floor][2]
    init_pose.pose.pose.orientation.w = init_pose_setting[floor][3]
    return init_pose


def map_client(input_setting, service_setting):
    ros_package = rospkg.RosPack()
    path = ros_package.get_path('robot_unique_parameters')

    # Decide the floor according to the postion.
    predefine_list = service_setting['hotelGoals']
    floor = None
    if re.match("[12345678]F", input_setting):
        floor = input_setting
    else:
        for raw, one_list in enumerate(predefine_list):
            if input_setting in one_list and raw == 0:
                floor = str(raw+1) + 'F'
                break
            elif input_setting in one_list:
                floor = str(raw) + 'F'
                break
    if floor is None:
        rospy.logwarn('[NC] map_client receive wrong input: ' + str(input_setting))
        return False

    nav_map_path = path + '/map/map' + floor[0:-1] + '_nav.yaml'
    amcl_map_path = path + '/map/map' + floor[0:-1] + '_amcl.yaml'
    currentFloorPub.publish(floor)
    changeNavMapPub.publish(nav_map_path)
    changeAMCLMapPub.publish(amcl_map_path)
    # Time for map loading
    rospy.loginfo('[NC] Request Changing Map to ' + str(floor))
    rospy.loginfo('[NC] wait 3s.')
    for i in range(4):
        rospy.loginfo('[NC] Countdown: ' + str(3-i))
        rospy.sleep(0.9)
    init_pose = init_pose_on_map(floor, service_setting)
    init_pose.header.stamp = rospy.Time.now()
    initPosePub.publish(init_pose)
    # Time for position setting.
    # Time for map loading
    rospy.loginfo('[NC] Reset Position.')
    rospy.loginfo('[NC] wait 6s.')
    for i in range(7):
        rospy.loginfo('[NC] Countdown: ' + str(6 - i))
        rospy.sleep(0.9)
    return True

if __name__ == '__main__':
    try:
        service_dict = loading_service_parameter()
        rospy.init_node("map_client")
        map_client('2F', service_dict)

    except rospy.ROSInterruptException:
        pass