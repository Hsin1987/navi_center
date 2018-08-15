import yaml
import rospkg
import json

# Loading the service parameter from robot_unique_parameter
def loading_service_parameter():
    ros_package = rospkg.RosPack()
    param_path = ros_package.get_path('robot_unique_parameters')
    param_path += '/service_setting/service_setting.yaml'
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)


def loading_hotel_goal():
    ros_package = rospkg.RosPack()
    goal_path = ros_package.get_path('robot_unique_parameters')
    goal_path += '/service_setting/hotelGoal.txt'
    f = open(goal_path, 'r')
    goal_dict = json.load(f)
    for key in goal_dict.keys():
        temp = key.encode('UTF8')
        goal_dict[temp] = goal_dict.pop(key)

    return goal_dict


def loading_init_position():
    ros_package = rospkg.RosPack()
    goal_path = ros_package.get_path('robot_unique_parameters')
    goal_path += '/service_setting/init_setting.txt'
    f = open(goal_path, 'r')
    init_setting = json.load(f)
    for key in init_setting.keys():
        temp = key.encode('UTF8')
        init_setting[temp] = init_setting.pop(key)

    return init_setting


