#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)
Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.
Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function

import math
import time

import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil

import numpy as np


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.",default='udp:127.0.0.1:14550')
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def generateMap(distToGoal, startLocation):
    # Based on distance between start and goal nodes(may need another data structure,
    # 2D or 3D array may be too costly, slow to calculate for real-time flight/object avoidance)

    mapGen = np.zeros((11,11),dtype='double')
    return mapGen

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print(" Waiting for home location ...")

    # We have a home location.
    vehicle.home_location=vehicle.location.global_frame
    home_loc = vehicle.home_location
    print("\n Home location: %s" % home_loc)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    vehicle.mode = VehicleMode("GUIDED")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame-relative to drone heading and position
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

def condition_yaw(heading, yaw_speed, relative=False):
    vehicle.mode = VehicleMode("GUIDED")
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle

    # Set proper yaw direction(ccw or cw)
    diff = vehicle.heading - heading
    if diff == 0.0: yaw_speed = 0
    if diff >= 0:
        if abs(diff)>180.0:
            dir = 1
        else: dir = -1
    else:
        if abs(diff)>180.0:
            dir = -1
        else: dir = 1
    print(yaw_speed, dir)
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    print("Changing Yaw, yaw rn: ", vehicle.heading)
    time.sleep(1)
    vehicle.send_mavlink(msg)
    print("yaw changed?, yaw rn: ", vehicle.heading)
    time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# A-Star Functions:
class Node():
    """A node class for A* Pathfinding"""
    # position for now is 2D vector with only x and y value, no z yet
    def __init__(self, parent=None, position=None, location=None):
        self.parent = parent
        self.position = position
        # Latitude(Y),Longitude ( X )
        dlat = goalLoc[0] - startLoc[0]
        dlong = goalLoc[1] - startLoc[1]
        bearing = get_bearing(sLoc,gLoc)
        #print("bearing: ", bearing)
        dist_goal_degrees = (((dlat*dlat) + (dlong*dlong))**0.5)
        #dist_to_goal_meters = (((dlat*dlat) + (dlong*dlong))**0.5) * 111319.5 # meters
        #distance_to_goal_idx = (((goal[0]-start[0])**2) + ((goal[1]-5)**2))**0.5
        #euclid_dist = (((position[0]-start[0])**2) + ((position[1]-5)**2))**0.5 # idx
        #print("euclid Dist: ", euclid_dist)
        # self.location = [((((goalLoc[0] - startLoc[0])/10)*(position[1]-5))+startLoc[0])
        #     ,((((goalLoc[1] - startLoc[1])/10)*(10-position[0]))+startLoc[1])]
        #[dNorth,dEast] = [((((euclid_dist)/10)*(position[1]-5)))*math.cos(np.deg2rad(bearing))
        #   ,((((euclid_dist)/10)*(10-position[0])))*math.sin(np.deg2rad(bearing))]
        #print("dNorth,dEast: ", [dNorth,dEast])

        # First make map oriented same as NSEW, then rotate by using trig
        # Location: [Latitude, Longitude] in array index: [Lat=Y(col), Long=X(row)] its reversed
        # delta-y(latitude) = 10-pos[0], delta-x(longitude) = pos[1]-5
        orig_lat = (((dist_goal_degrees)/10)*(10-position[0]))
        orig_long = (((dist_goal_degrees)/10)*(position[1]-5))
        orig_dist = ((orig_long**2) + (orig_lat**2))**0.5
        self.location = [
            #orig_lat-(orig_dist-(orig_dist*math.cos(np.deg2rad(90+360.0-bearing)))),
            startLoc[0]-(orig_long-(orig_dist*math.sin(np.deg2rad(90+360.0-bearing)))),
            startLoc[1]+(orig_lat-(orig_dist-(orig_dist*math.cos(np.deg2rad(90+360.0-bearing)))))]
        self.location = LocationGlobal(startLoc[0]-(orig_long-(orig_dist*math.sin(np.deg2rad(90+360.0-bearing)))),
                                       startLoc[1]+(orig_lat-(orig_dist-(orig_dist*math.cos(np.deg2rad(90+360.0-bearing))))),
                                       vehicle.location.global_frame.alt)
        print("pos: ",self.position)
        print("Location: ", self.location)
        print(" ")
        self.g = 0
        self.h = getHeuristic(position[0],position[1], goal)
        self.f = 0

    def __eq__(self, other):
        return (self.position == other.position)

def getHeuristic(x,y,goal):
    # Euclidean distance:
    euclid_dist = ((abs(float(goal[0]-x))**2.0)+(abs(float(goal[1]-y))**2.0))**0.5
    # return h value based on "Manhattan Distance"
    Manhattan_Distance = abs(goal[0]-x) + abs(goal[1]-y)
    return euclid_dist

# Start Location(usually): -35.363261,149.165230
# Goal location: -35.354073, 149.152031

def aStar(map, start, goal):
    # get the starting and goal nodes setup:
    solnMap = map.copy()
    startNode = Node(None, start, [-35.363261,149.165230])
    goalNode = Node(None, goal, [-35.354073, 149.152031])

    # Initialize Open and Closed lists
    open_list = []
    closed_list = []

    open_list.append(startNode)

    # loop until goal is reached
    while(len(open_list)>0):

        currentNode=open_list[0]
        currentIndex = 0

        for index,item in enumerate(open_list):
            if item.f < currentNode.f:
                currentNode = item
                currentIndex = index

        open_list.pop(currentIndex)
        solnMap[item.position[0],item.position[1]] = 0.8
        # Already traveled to nodes are in closed list
        closed_list.append(currentNode)

        # if the goal node is found:
        if currentNode == goalNode:
            path = []
            current = currentNode
            while current is not None:
                path.append([current.position,current.location])
                current = current.parent
            return path[::-1] # Return reversed path

        # Do expansion of current Node:
        # Can do an angle based expansion, not just expanding in "rectangle"
        # "finding children"

        # Do basic expansion - Later change to angle based?
        # expansion method if using array below, adjacent nodes
        expansionList = []
        expansionAdjacent = np.array([[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]])
        #print(currentNode.position)
        for new_position in expansionAdjacent:
            newNode_position = np.array(currentNode.position) + new_position
            # Check if new node is a real location on node map
            if(newNode_position[0] not in range(0,map.shape[0]) or newNode_position[1] not in range(0,map.shape[1])):
                #print("new node not in range", newNode_position)
                continue
            #print("new node position", newNode_position)
            # Check if node is not obstacle
            if map[newNode_position[0],newNode_position[1]] == 100:
                #print("Obstacle at: ", newNode_position)
                continue
            newNode = Node(parent=currentNode, position=newNode_position.tolist())
            expansionList.append(newNode)

        # Loop through expanded nodes:
        for child in expansionList:
            if any(item==child for item in closed_list):
                #print("child in closed list: ", child.position)
                continue
            child.g = currentNode.g + 1
            child.f = child.g + child.h
            if child in open_list and child.g > open_list[open_list.index(child)].g:
                #print("child in open list: ", child.position)
                continue
            open_list.append(child)

#Main
arm_and_takeoff(5)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3


# TEST velocity control
#send_ned_velocity(5,0,0,10)
vehicle.simple_goto(vehicle.location.global_frame,1)
# Goal Latitude/Longitude(global):

# AStar commands here:
# Start Location(usually): -35.363261,149.165230
# Goal location: -35.354073, 149.152031
print("Vehicle current altitude(global frame): ", vehicle.location.global_frame.alt)
goalLocation = dronekit.LocationGlobal(-35.354073, 149.152031, vehicle.location.global_frame.alt)
newHeading = get_bearing(vehicle.location.global_frame,goalLocation)
print("Bearing towards goal is: ", newHeading)
condition_yaw(newHeading,5)
print("Current Heading: ", vehicle.heading)
dist_to_goal = get_distance_metres(vehicle.location.global_frame, goalLocation)
map = generateMap(dist_to_goal, vehicle.location.global_frame)

startLoc = [vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_frame.alt]
goalLoc = [-35.354073, 149.152031]
sLoc = LocationGlobal(startLoc[0],startLoc[1],startLoc[2])
gLoc = LocationGlobal(goalLoc[0],goalLoc[1],startLoc[2])
print(np.subtract(goalLoc,startLoc[:2]))
goal = [0,5]
map[1:10,4] = 100
solnMap = map.copy()
hmap = map.copy()
start = [10,5]
map[0,5] = 2
map[10,5] = 1
solnPath = aStar(map, start, goal)

print("Starting Map: \n", map)
print("Solution Path: ", solnPath)
solnMap = np.array(solnMap)
solnMap = solnMap
moving = True
while moving:
    for pos in solnPath:
        print(pos)
        #solnMap[0,pos[0][0],pos[0][1]] = pos[1]
        solnMap[pos[0][0],pos[0][1]] = 5
        new_loc = pos[1]
        vehicle.simple_goto(new_loc, 20)
    moving = False
#solnMap[0,5] = 3
#solnMap[10,5] = 2
print("Solution on map: \n", solnMap)





# print("Going towards first point for 30 seconds ...")
# point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
# vehicle.simple_goto(point1)
#
# # sleep so we can see the change in map
# time.sleep(30)
#
# print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
# point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
# vehicle.simple_goto(point2, groundspeed=10)
#
# # sleep so we can see the change in map
time.sleep(6)

print("Landing")
vehicle.mode = VehicleMode("LAND")

# print("Returning to Launch")
# vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()