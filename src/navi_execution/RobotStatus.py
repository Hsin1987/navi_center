"""
This module contain all the status. They indicate the mission stage and all the interaction & action of AMR should be
contain in the specific status.

    robot status:
         # A. Initiation Phase
            A1. init
         # B. Standby Phase:
            B1. charging: Battery capacity below 20%, in station, in charging.
                ps: charging should not be available.
            B2. need_charging: Boost up in low battery. And not in the Station.
            B3. available: in waiting position or in charging station(battery_capacity > 20%)

         # C. Planning Phase:
            C1. planning: received goal and planning a path.

         # D. Task Execution Phase
            D1. moving: on task, in moving stage
            D2. waitingEV: in waitingEV Position
            D3. waitingEV_C: confirm that EV received request to pick up AMR.
            D4. checkingEV
            D5. enteringEV: moving into the elevator
            D6. inEV: in the elevator
            D7. in_EV_C: confirm that EV received request to left AMR.
            D8. in_EV_R: received elevator Reached.
            D9. alightingEV: moving out of the elevator
            D10. reached: reached the hotelGoal

        # Else:
            E1. rss_request: sent request remote support and wait.
            E2. rss_mode: remote support take control

    # Use
"""

robot_status = ('init',   'charging',  'need_charging', 'available',   'received_mission',
                'moving', 'moving_reached', 'retry_goal'
                'waitingEV', 'waitingEV_C',   'checkingEV',  'enteringEV',
                'inEV',   'inEV_C',    'inEV_R',        'alightingEV', 'reached',
                'rss_request', 'rss_mode')


class RobotStatus:
    def __init__(self):
        # Robot Statue: indicate the stage of task.
        self.status = 'init'
        # Current floor.
        # ==  Default: Lobby
        self.floor = '1F'
        # ==  Default: Lobby
        self.position = 'Lobby'
        # hotelGoal, received task.
        # ==  Default: empty
        self.mission = None
        # Timestamp of goal.
        self.ts_start = None
        # Battery Capacity
        self.capacity = 100.0
        # Aborted Task Counter: [move_base, ev_safetyCheck, ev_c_Check]
        self.abortCount = [0, 0, 0]

    # ===== Status Setting ===== #
    def set_status(self, status_input):
        global robot_status
        if status_input in robot_status:
            self.status = status_input
            # Set successfully
            return True
        else:
            return False

    # ===== Setting Mission ===== #
    def set_mission(self, new_mission, hotel_goals):
        global robot_status
        for floor, goal_list in enumerate(hotel_goals):
            if new_mission in goal_list:
                self.mission = new_mission
                return True
            else:
                return False

    # ===== Setting Mission ===== #
    def mission_done(self):
        global robot_status
        self.mission = None

    # ===== Setting Mission ===== #
    def set_position(self, position_input, hotel_goals):
        global robot_status
        for floor, goal_list in enumerate(hotel_goals):
            if position_input in goal_list:
                self.position = position_input
                return True
            else:
                return False
