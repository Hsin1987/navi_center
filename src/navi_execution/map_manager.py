import rospy
import rospkg
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
# Map & Position Init
changeNavMapPub = rospy.Publisher('/map_server_nav/reload', String, queue_size=1)
changeAMCLMapPub = rospy.Publisher('/map_server_amcl/reload', String, queue_size=1)
initPosePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

# Global Service parameter
# service_dict = {}
param_path = '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml'


# Loading the service parameter from robot_unique_parameter
def loading_service_parameter():
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)


def init_pose_on_map(floor, service_dict):
    init_pose = PoseWithCovarianceStamped()
    init_pose_setting = service_dict['init_pose']
    # Universal Setting
    init_pose.header.frame_id = 'map'
    init_pose.pose.covariance[0] = 0.25
    init_pose.pose.covariance[7] = 0.25
    init_pose.pose.covariance[35] = 0.066

    # Deal with the standard floor:
    if floor in service_dict['station_floor']:
        floor = 0
    elif floor in service_dict['standard_floor']:
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


def map_client(floor, service_dict):
    ros_package = rospkg.RosPack()
    path = ros_package.get_path('navi_center')

    nav_map_path = path + '/map/map' + floor[0:-1] + '_nav.yaml'
    amcl_map_path = path + '/map/map' + floor[0:-1] + '_amcl.yaml'

    changeNavMapPub.publish(nav_map_path)
    changeAMCLMapPub.publish(amcl_map_path)
    # Time for map loading
    rospy.sleep(3.0)
    init_pose = init_pose_on_map(floor, service_dict)
    init_pose.header.stamp = rospy.Time.now()
    initPosePub.publish(init_pose)
    # Time for position setting.
    rospy.sleep(6.0)
    return

if __name__ == '__main__':
    try:
        service_dict = loading_service_parameter()
        rospy.init_node("map_client")
        map_client('2F')

    except rospy.ROSInterruptException:
        pass