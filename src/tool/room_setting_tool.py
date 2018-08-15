import json
import rospkg

hotelGoal_dict = {
    # 'room_id': [x, y, z, w, Ori_Release, Position_tolerance]
    'Station': [3.45, 3.87, 0.7, 0.7,  False, 0.1],
    'Station_out': [2.11, 6.25, 0.7, 0.7, False, 0.1],
    'Lobby': [2.11, 6.27, 0, 1,  False, 0.1],
    '101':   [2.94, -3.77, 0, 1,  True, 0.1],
    '102':   [3.45, 3.87, 0, 1,  True, 0.1],
    '404':   [-0.72, 3.33, 0.7, 0.7,  True, 0.1],
    'EVW1':  [3.06, 7.24, 1, 0,  False, 0.1],
    'EVW4':  [3.06, 7.24, 1, 0,  False, 0.1],
    'EVW1S': [-0.72, 3.33, 0.7, 0.7,  True, 0.1],
    'EVW4S': [-0.72, 3.33, 0.7, 0.7,  True, 0.1]
}


print(str(hotelGoal_dict))

ros_package = rospkg.RosPack()
param_path = ros_package.get_path('robot_unique_parameters')
param_path += '/service_setting/hotelGoal.txt'

mydatafile = "hotelGoal.txt"

with open(param_path, 'w') as f:
    json.dump(hotelGoal_dict, f)

print("END")

"""
with open(mydatafile) as f:
    a = json.load(f)
    for key in a.keys():
        temp = key.encode('UTF8')
        a[temp] = a.pop(key)
    print a

for key in hotelGoal_dict.keys():
    print key
    print hotelGoal_dict[key]
"""