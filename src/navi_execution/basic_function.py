import yaml
import rospkg


# Loading the service parameter from robot_unique_parameter
def loading_service_parameter():
    ros_package = rospkg.RosPack()
    param_path = ros_package.get_path('robot_unique_parameters')
    param_path += '/params/service_setting.yaml'
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)