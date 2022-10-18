import numpy as np
import math

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    # if type(original_location) is LocationGlobal:
    #     targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    # elif type(original_location) is LocationGlobalRelative:
    #     targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    # else:
    #     raise Exception("Invalid Location object passed")

    return [newlat, newlon];

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = aLocation2[1] - aLocation1[1]
    off_y = aLocation2[0] - aLocation1[0]
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;


def generateMap(straightLineDistance, startingLocation):
    # Based on distance between start and goal nodes(may need another data structure,
    # 2D or 3D array may be too costly, slow to calculate for real-time flight/object avoidance)
    mapGen = np.ones((11,11),dtype='float')
    return mapGen

class Node():
    """A node class for A* Pathfinding"""
    # position for now is 2D vector with only x and y value, no z yet
    def __init__(self, parent=None, position=None, location=None):
        self.parent = parent
        self.position = position
        # Latitude(Y),Longitude ( X )
        dlat = goalLoc[0] - startLoc[0]
        dlong = goalLoc[1] - startLoc[1]
        bearing = get_bearing(startLoc,goalLoc)
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

map = generateMap(11, [-35.363261,149.165230])
startLoc = [-35.363261,149.165230]
goalLoc = [-35.354073, 149.152031]
print(np.subtract(goalLoc,startLoc))
goal = [0,5]
map[1:10,4] = 100
solnMap = map.copy()
hmap = map.copy()
# heuristic ?
for x in range(0,10):
    for y in range(0,10):
        hmap[x,y] = round(getHeuristic(x,y,goal),1)
start = [10,5]
map[0,5] = 2
map[10,5] = 1
solnPath = aStar(map, start, goal)

print(hmap)
print("Starting Map: \n", map)
print("Solution Path: ", solnPath)
solnMap = np.array(solnMap)
solnMap = solnMap
for pos in solnPath:
    print(pos)
    #solnMap[0,pos[0][0],pos[0][1]] = pos[1]
    solnMap[pos[0][0],pos[0][1]] = 5
#solnMap[0,5] = 3
#solnMap[10,5] = 2
print("Solution on map: \n", solnMap)
# openset of nodes that are visited(initialized with the start node)
# for openset, need to find node with minimum f or cost

# need a function to expand from current node
# need a data structure to store cameFrom nodes to reconstruct path when planning is finished
# need a closedSet of nodes already visited

