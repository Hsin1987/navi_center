#!/usr/bin/env python
import rospy
import re
from collections import defaultdict
from heapq import heappop, heappush


# For Testing
import yaml
"""
Fruntion for planning a path for navigating to a position on the predefined list.

"""
# Global Service parameter
# service_dict = {}
param_path = '/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml'


# Loading the service parameter from robot_unique_parameter
def loading_service_parameter():
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    return yaml.load(params_raw)


class PathPlanner:

    def __init__(self):
        """
        # ============== FOR TESTING
        f = open('/home/ubuntu/amr_ws/src/robot_unique_parameters/params/service_setting.yaml', 'r')
        params_raw = f.read()
        f.close()
        service_dict =  yaml.load(params_raw)
        hotel_goals = service_dict['hotelGoals']
        # ============== FOR TESTING
        """
        self.pathDic = defaultdict(list)
        self.path = None
        self.cost = None

    def setting(self, hotel_goals):
        for floor, goals in enumerate(hotel_goals):
            # Station-Related
            # ['Station', 'Station_out','Lobby']
            if floor == 0:
                if goals == ['Station', 'Station_out', 'Lobby']:
                    self.pathDic['Station'] = [(1, 'Station_out')]
                    self.pathDic['Station_out'] = [(1, 'Station'), (1, 'Lobby')]
                    self.pathDic['Lobby'] = [(1, 'Station_out')]
                else:
                    # rospy.report error
                    print("List is nor correct.")
            # Lobby; 1F
            # - ['101', '102', 'EVW1', 'EVW1S']
            elif floor == 1:
                for goal in goals:
                    # Make sure they all connect to the station through lobby position.
                    self.pathDic['Lobby'].append((1, goal))
                    self.pathDic[goal].append((1, 'Lobby'))
                    # Add other goal in the same floor.
                    goals_buff = goals[:]
                    goals_buff.remove(goal)
                    for goal_s in goals_buff:
                        self.pathDic[goal].append((1, goal_s))
                # Connect each floor through 'EVin'
                evw = 'EVW' + str(floor)
                evin = 'EVin'
                evws = evw + 'S'
                self.pathDic[evw].append((1, evin))
                self.pathDic[evin].append((1, evw))
                self.pathDic[evw].append((1, evws))
                self.pathDic[evws] = [(1, evw)]
            else:
                for goal in goals:
                    # Add other goal in the same floor.
                    goals_buff = goals[:]
                    goals_buff.remove(goal)
                    for goal_s in goals_buff:
                        self.pathDic[goal].append((1, goal_s))
                # Connect each floor through 'EVin'
                evw = 'EVW' + str(floor)
                evin = 'EVin'
                evws = evw + 'S'
                self.pathDic[evw].append((1, 'EVin'))
                self.pathDic[evin].append((1, evw))
                self.pathDic[evw].append((1, evws))
                self.pathDic[evws] = [(1, evw)]

    def path_agent(self, start, destination):
        rospy.loginfo('[NC] Start Path Planner. From: ' + str(start) + ' to ' + str(destination) + '.')
        planner_success = self.dijkstra(start, destination)
        if planner_success:
            rospy.loginfo('[NC] Path : ' + str(self.path))
            #
            if re.match("EVW[12345678]", self.path[0]) or self.path[0] == 'Station_out':
                return self.path
            else:
                return self.path[1:]
        else:
            rospy.logwarn('[NC] Path Planning Fail.')
            return []

    # Dijkstra Algorithm: find the shortest path
    def dijkstra(self, start, destination):
        path_buffer = list()
        q, seen = [(0, start, path_buffer)], set()
        while q:
            # Pop and return the smallest item from the heap, maintaining the heap invariant. If the heap is empty,
            # IndexError is raised. To access the smallest item without popping it, use heap[0].
            (self.cost, v1, self.path) = heappop(q)
            if v1 not in seen:
                seen.add(v1)
                self.path.append(v1)
                if v1 == destination:
                    return True

                for c, v2 in self.pathDic.get(v1, ()):
                    if v2 not in seen:
                        path2 = self.path[0:]
                        # Push the value item onto the heap, maintaining the heap invariant.
                        heappush(q, (self.cost + c, v2, path2))

        return False

if __name__ == '__main__':
    service_dict = loading_service_parameter()
    path_planner = PathPlanner()
    path_planner.setting(service_dict['hotelGoals'])
    print(path_planner.path_agent('Station_out', 'Station'))
