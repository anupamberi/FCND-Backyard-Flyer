import argparse
import time
from enum import Enum

import numpy as np
import math

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = self.calculate_box()
        self.in_mission = True
        self.waypoint_count = 0

        # initial state
        self.flight_state = States.MANUAL

        # Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def is_waypoint_close(self, expected_position, current_position, tolerance):
        """Detect if the current and expected positions North & East in local frame of reference 
        match with a given tolerance level"""
        is_north_close = math.isclose(expected_position[0], current_position[0], abs_tol=tolerance)
        is_east_close = math.isclose(expected_position[0], current_position[0], abs_tol=tolerance)
        return is_north_close and is_east_close

    def local_position_callback(self):
        if self.flight_state == States.WAYPOINT:
            # Check if have reached a waypoint win the tolerance of 0.5
            if self.is_waypoint_close(self.target_position, self.local_position, 0.5):
                # Go to next waypoint
                self.waypoint_transition()
          
    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.TAKEOFF:
            # Initiate the drone to transition to its waypoints
            self.waypoint_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()


    def calculate_box(self):
        # Return waypoints to fly a box
        waypoints = [(10.0, 0.0, 3.0, 0.0),
                     (10.0, 10.0, 3.0, 0.0),
                     (0.0, 10.0,3.0, 0.0),
                     (0.0, 0.0, 3.0, 0.0)]
        return waypoints


    def arming_transition(self):
        print("arming transition")
        # Take control and arm the drone
        self.take_control()
        # Arm the drone
        self.arm()
        # set the home location to the current position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        # set the state to ARMING
        self.flight_state = States.ARMING


    def takeoff_transition(self):
        print("takeoff transition")
        # Set target position to 3m altitude
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        # Take to an altitude of 3 m
        self.takeoff(target_altitude)
        # Transition to TAKEOFF state
        self.flight_state = States.TAKEOFF

    def get_next_waypoint(self):
        # Return the next waypoint by reading the list of available waypoints
        if self.waypoint_count != len(self.all_waypoints):
            next_waypoint = self.all_waypoints[self.waypoint_count]
            self.waypoint_count += 1
            return next_waypoint

    def waypoint_transition(self):
        # Get the next waypoint
        next_waypoint = self.get_next_waypoint()

        if not (next_waypoint is None):
            # Set the target position
            self.target_position[0] = next_waypoint[0]
            self.target_position[1] = next_waypoint[1]
            print("waypoint transition")
            print("target position", self.target_position)
            # Transition to the next waypoint
            self.cmd_position(*next_waypoint)
            self.flight_state = States.WAYPOINT
        else:
            # We have traversed all the waypoints. We now check if we can land the drone
            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]
            # check if altitude is within 95% of target and compare North & East distances with a tolerance of 0.02
            if altitude > 0.95 * self.target_position[2] and self.is_waypoint_close(self.target_position, self.local_position, 0.02):
                self.landing_transition()

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
