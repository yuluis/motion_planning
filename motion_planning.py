import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import re
import utm

import time
from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        #self.timeout = 10
        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], np.deg2rad(0.0))

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        myStartTime = time.clock()
        print("Searching for a path ...(T0)")

        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 4
        self.target_position[2] = TARGET_ALTITUDE

        file = open("colliders.csv","r")
        file.readline()
        data = file.readline()
        lat0, lon0 = data.split(",")
        lat0 = float(lat0)
        lon0 = float(lon0)
        file.close()

        print("starting latitude, longitude",lat0,lon0) #TODO map start and goal onto valid locally defined space
        # DO: set home position to (lat0, lon0, 0)
        self.set_home_position(lon0, lat0, 0)  # set the current location to be the home position
        # DO: convert to current local position using global_to_local()
        global_home = [lon0, lat0, 0]
        global_position = [lon0, lat0, 0]
        local_position = global_to_local(global_position, global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
        # Determine offsets between grid and map
        north_offset = int(np.abs(np.min(data[:, 0])))
        east_offset = int(np.abs(np.min(data[:, 1])))

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        print("Finished reading collider data. Time elapsed: ", time.clock() - myStartTime)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        # Define starting point on the grid (this is just grid center)
        grid_start = (north_offset, east_offset) # center of map?
        # DO: convert start position to current position rather than map center
        start = (int(local_position[0]+north_offset), int(local_position[1]+east_offset))
        
        # Set goal as some arbitrary position on the grid
        #local_target = global_to_local((-122.396384, 37.793278, 0), global_home) # up market
        #local_target = global_to_local((-122.398805, 37.793372, 0), global_home) # around the corner
        #local_target = global_to_local((-122.398321, 37.791719, 0), global_home) # down market
        #local_target = global_to_local((-122.397762, 37.793118, 0), global_home)  # around the building

        #grid_goal = (int(north_offset + local_target[0]), int(east_offset + local_target[1]))
        grid_goal = (461,510)
 #       grid_goal = (461,500)
 #       grid_goal = (461,490)

        while grid[grid_goal] == 1.0 :
            grid_goal = (grid_goal[0] + 1, grid_goal[1]) # place goal to first available position north of requested

        # DO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # DO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not gfgdone here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("Time is after astar: ", time.clock())
        #path = [[305,435],grid_goal]
        # DO: prune path to minimize number of waypoints
        # DO (if you're feeling ambitious): Try a different approach altogether!
        pruned_path = prune_path(path)
        print(len(pruned_path))
        print("Time is after pruning: ", time.clock())

        # Convert path to waypoints
        waypoints = [(p[0] - north_offset, p[1] - east_offset, TARGET_ALTITUDE+1) for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # DO: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port))
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
