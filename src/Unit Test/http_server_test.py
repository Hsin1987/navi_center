#!/usr/bin/python

import cherrypy
import numpy as np

# stand_by, on_mission, no_response
robot_in_command = None



class StringGenerator(object):
    @cherrypy.expose
    def call(self, robot_id, tid, current_floor, target_floor):
        global robot_in_command
        robot_in_command = robot_id
        # Random Feedback:
        p = 0.8
        available = np.random.binomial(1, p)
        if available:
            
            return str(tid)
        else:
            # Fake, no response.
            # return str([robot_id, status, tid, current_floor, target_floor])
            print ("PASS...............................")
            pass

    @cherrypy.expose
    def entering_done(self, robot_id, tid, current_floor, target_floor):
        if robot_id != robot_in_command:
            print('WEEIRED. Different Robot ID.')
        # Random Feedback:
        p = 0.8
        available = np.random.binomial(1, p)
        if available:
            return str(tid)
        else:
            # Fake, no response.
            # return str([robot_id, status, tid, current_floor, target_floor])
            print ("PASS...............................")
            pass

    @cherrypy.expose
    def release(self, robot_id, tid, current_floor, target_floor):
        global robot_in_command
        if robot_id == robot_in_command:
            # Reset Flag
            robot_in_command = None
        
        # Random Feedback:
        p = 0.9
        available = np.random.binomial(1, p)
        if available:
            return str(tid)
        else:
            # Fake, no response.
            # return str([robot_id, status, tid, current_floor, target_floor])
            print ("PASS...............................")
            pass

if __name__ == '__main__':
    cherrypy.server.socket_host = '192.168.30.132'
    cherrypy.quickstart(StringGenerator())
