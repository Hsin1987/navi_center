#!/usr/bin/python

import cherrypy
import numpy as np

# stand_by, on_mission, no_response
ev_status = 'stand_by'

class StringGenerator(object):
    @cherrypy.expose
    def call(self, robot_id, tid, current_floor, target_floor):
        global ev_status
        # Random Feedback:
        p = 0.8
        available = np.random.binomial(1, p)
        if available:
            # TODO: Need SQL Record Here.

            return str(tid)
        else:
            # Fake, no response.
            # return str([robot_id, status, tid, current_floor, target_floor])
            print ("PASS...............................")
            pass

    @cherrypy.expose
    def entering_done(self, robot_id, tid, current_floor, target_floor):
        global ev_status
        # Random Feedback:
        p = 0.8
        available = np.random.binomial(1, p)
        if available:
            # TODO: Need SQL Record Here.

            return str(tid)
        else:
            # Fake, no response.
            # return str([robot_id, status, tid, current_floor, target_floor])
            print ("PASS...............................")
            pass

if __name__ == '__main__':
    cherrypy.server.socket_host = '192.168.30.132'
    cherrypy.quickstart(StringGenerator())
