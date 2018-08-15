import rospy
import yaml


param_path = '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml'


def loading_service_parameter():
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)



service_dict = loading_service_parameter()

for floor, goal_list in enumerate(service_dict['hotelGoals']):
    if '000' in goal_list:
        print('In ' + str(floor) + " F")
    else:
        print('No')