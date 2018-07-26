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
service_dict = {}
param_path = '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml'


def loading_service_parameter():
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)


def change_map(floor):
    global service_dict
    ros_package = rospkg.RosPack()
    path = ros_package.get_path('navi_center')

    map_path = path + '/map/'

    # pubMsg = {1F:'change to 1F', 2F:'change to 2F', 3F:'change to 3F', 4F:'change to 4F'}
    pubNavMsg = {
        1: packPath+'/map/map'+str(floor)+'_nav.yaml',
        2: packPath+'/map/map'+str(floor)+'_nav.yaml',
        3: packPath+'/map/map'+str(floor)+'_nav.yaml',
        4: packPath+'/map/map'+str(floor)+'_nav.yaml',
        5: packPath+'/map/map'+str(floor)+'_nav.yaml',
        6: packPath+'/map/map'+str(floor)+'_nav.yaml',
        7: packPath+'/map/map'+str(floor)+'_nav.yaml',
        8: packPath+'/map/map'+str(floor)+'_nav.yaml'
    }

    pubAmclMsg = {
        1: packPath+'/map/map'+str(floor)+'_amcl.yaml',
        2: packPath+'/map/map'+str(floor)+'_amcl.yaml',
        3: packPath+'/map/map'+str(floor)+'_amcl.yaml',
        4: packPath+'/map/map'+str(floor)+'_amcl.yaml',
        5: packPath+'/map/map'+str(floor)+'_amcl.yaml',
        6: packPath+'/map/map'+str(floor)+'_amcl.yaml',
        7: packPath+'/map/map'+str(floor)+'_amcl.yaml',
        8: packPath+'/map/map'+str(floor)+'_amcl.yaml'
    }
    """
    changeNavMapPub.publish(pubNavMsg[floor])
    changeAmclMCLMapPub.publish(pubAmclMsg[floor])
    rospy.sleep(6.0)
    newMapPose = hotelGoals.getInitPose(floor)
    newMapPose.header.stamp = rospy.Time.now()
    initPosePub.publish(newMapPose)
    """
    return


service_dict = loading_service_parameter()
change_map()