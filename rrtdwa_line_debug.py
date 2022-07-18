
# Planning
# Dynamic Window Approach (Local Planning) with moving obstacles

import pygame, math, time, random, copy
from pygame.locals import *

pygame.init()

# set the width and height of the screen (pixels)
WIDTH = 1500
HEIGHT = 1000

size = [WIDTH, HEIGHT]

# Initialise Pygame display screen
screen = pygame.display.set_mode(size)


max_iterations = 3000

import random
from typing import List
import numpy as np

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT2D:
    def __init__(self, x, y):
        self.swath = []
        self.path = []
        self.x_bound = x
        self.y_bound = y

    def sample(self):
        """
        Returns a random node for expansion
        """
        x_sample = int(random.uniform(0, self.x_bound))
        y_sample = int(random.uniform(0, self.y_bound))
        return Node(x_sample, y_sample)

    @staticmethod
    def find_nearest(swath: List[Node], r_node: Node):
        """
        Returns the index of the nearest node in the swath from the given
        randomly sampled one
        """
        return np.argmin(
            [(node.x - r_node.x) ** 2 + (node.y - r_node.y) ** 2 for node in
             swath]
        )

    @staticmethod
    def close_to_point(r_node: Node, goal: Node , tol: float = 5) -> bool:
        """
        """
        from math import sqrt
        dist = sqrt((goal.x - r_node.x) ** 2 + (goal.y - r_node.y) ** 2)
        return dist < tol

    @staticmethod
    def get_new_node_at_fixed_distance(random_node, nearest) -> Node:
        """
        """
        from math import atan2, sin, cos
        dist = 0.025
        theta = atan2(random_node.y - nearest.y, random_node.x - nearest.x)
        x_new = nearest.x + dist*cos(theta)
        y_new = nearest.y + dist*sin(theta)
        return Node(x_new, y_new)

    def plan(self, start: Node, goal: Node):
        """
        """
        pygame.draw.circle(screen, red, (int(goal.x), int(goal.y)), int(k * BARRIERRADIUS), 0)
        self.swath.append(start)
        for i in range(max_iterations):
            print("rrt iterations:", i)
            sample = self.sample()
            nearest = self.swath[self.find_nearest(self.swath, sample)]
            new_node = self.get_new_node_at_fixed_distance(sample, nearest)

            new_node.parent = nearest
            self.swath.append(new_node)

            pygame.draw.line(screen, (255, 255, 255), (25 * nearest.x, 25 * nearest.y), (25 * new_node.x, 25 * new_node.y))

            if self.close_to_point(new_node, goal):
                goal.parent = new_node
                self.swath.append(goal)
                break

        # Form the path from swath and display
        current = goal

        while current and (current.x, current.y, current.parent) != (start.x, start.y, start.parent):
            # print("There you go, start!", (start.x, start.y, start.parent))
            # print("There you go!, curremt", (current.x, current.y, current.parent))
            self.path.append(current)
            print("New node appended", len(self.path))
            current = current.parent
        self.path.reverse()  # Since it is formed in the reverse order

    def draw_path(self):
        for idx, waypoint in enumerate(self.path):
            if idx != len(self.path)-1:
                coord1 = [waypoint.x, waypoint.y]
                coord2 = [self.path[idx+1].x, self.path[idx+1].y]
                pygame.draw.line(screen, (76, 187, 23), coord1, coord2)


# Constants and variables
# Units here are in metres and radians using our standard coordinate frame
BARRIERRADIUS = 0.1
ROBOTRADIUS = 0.10
W = 2 * ROBOTRADIUS # width of robot
SAFEDIST = ROBOTRADIUS      # used in the cost function for avoiding obstacles

MAXVELOCITY = 0.5     #ms^(-1) max speed of each wheel
MAXACCELERATION = 0.5 #ms^(-2) max rate we can change speed of each wheel


BARRIERVELOCITYRANGE = 0.15


# The region we will fill with obstacles
PLAYFIELDCORNERS = (-4.0, -3.0, 4.0, 3.0)



# Starting pose of robot
#x = PLAYFIELDCORNERS[0] - 0.5
x = 0
y = 0
theta = 0.0

# Use for displaying a trail of the robot's positions
locationhistory = []

# Starting wheel velocities
vL = 0.00
vR = 0.00

# Timestep delta to run control and simulation at
dt = 0.1
STEPSAHEADTOPLAN = 10
TAU = dt * STEPSAHEADTOPLAN

# Barrier (obstacle) locations
barriers = []
# barrier contents are (bx, by, visibilitymask)
# Generate some initial random barriers
for i in range(50):
        (bx, by, vx, vy) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]), random.gauss(0.0, BARRIERVELOCITYRANGE), random.gauss(0.0, BARRIERVELOCITYRANGE))
        barrier = [bx, by, vx, vy]
        barriers.append(barrier)


# rrt_path = [(1,1, 0.1, 0.1),(22,22, 0.0, 0.0),(32,31, 0.0, 0.0),(46,57, 0.0, 0.0),(66,69, 0.0, 0.0),(75,77)]
#rrt_path = [(1,1,0.1, 0.1),(2,2,0.1, 0.1),(-1,-2,0.1),(2,5,0.1, 0.1),(3,6,0.1, 0.1),(-0.11,-0.11,0.2),(1,6,0.1, 0.1),(-5,-2),(7,7,0.1, 0.1)]
# rrt_path = [(1,1),(2,2),(2,5),(3,6),(1,6),(7,7)]

targetindex = 0

rrt_path = []



def printBarriers():
        for (i, barrier) in enumerate(barriers):
                print (i, barrier[0], barrier[1], barrier[2], barrier[3])

def moveBarriers(dt):
        for (i, barrier) in enumerate(barriers):
                barriers[i][0] += barriers[i][2] * dt
                if barriers[i][0] < PLAYFIELDCORNERS[0]:
                        barriers[i][2] = -barriers[i][2]
                if barriers[i][0] > PLAYFIELDCORNERS[2]:
                        barriers[i][2] = -barriers[i][2]
                barriers[i][1] += barriers[i][3] * dt
                if barriers[i][1] < PLAYFIELDCORNERS[1]:
                        barriers[i][3] = -barriers[i][3]
                if barriers[i][1] > PLAYFIELDCORNERS[3]:
                        barriers[i][3] = -barriers[i][3]



# Constants for graphics display
# Transformation from metric world frame to graphics frame
# k pixels per metre
# Horizontal screen coordinate:     u = u0 + k * x
# Vertical screen coordinate:       v = v0 - k * y


black = (20,20,40)
lightblue = (0,120,255)
darkblue = (0,40,160)
red = (255,100,0)
white = (255,255,255)
blue = (0,0,255)
grey = (70,70,70)
k = 80 # pixels per metre for graphics

# Screen centre will correspond to (x, y) = (0, 0)
u0 = WIDTH / 2
v0 = HEIGHT / 2



# This makes the normal mouse pointer invisible in graphics window
pygame.mouse.set_visible(0)


# Array for path choices use for graphics 
pathstodraw = []






# Function to predict new robot position based on current pose and velocity controls
# Uses time deltat in future
# Returns xnew, ynew, thetanew
# Also returns path. This is just used for graphics, and returns some complicated stuff
# used to draw the possible paths during planning. Don't worry about the details of that.
def predictPosition(vL, vR, x, y, theta, deltat):
        # Simple special cases
        # Straight line motion
        if (round (vL,3) == round(vR,3)):
                xnew = x + vL * deltat * math.cos(theta)
                ynew = y + vL * deltat * math.sin(theta)
                thetanew = theta
                path = (0, vL * deltat)   # 0 indicates pure translation
        # Pure rotation motion
        elif (round(vL,3) == -round(vR,3)):
                xnew = x
                ynew = y
                thetanew = theta + ((vR - vL) * deltat / W)
                path = (1, 0) # 1 indicates pure rotation
        else:
                # Rotation and arc angle of general circular motion
                # Using equations given in Lecture 2
                R = W / 2.0 * (vR + vL) / (vR - vL)
                deltatheta = (vR - vL) * deltat / W
                xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
                ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
                thetanew = theta + deltatheta

                # To calculate parameters for arc drawing (complicated Pygame stuff, don't worry)
                # We need centre of circle
                (cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))
                # Turn this into Rect
                Rabs = abs(R)
                ((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
                if (R > 0):
                        start_angle = theta - math.pi/2.0
                else:
                        start_angle = theta + math.pi/2.0
                stop_angle = start_angle + deltatheta
                path = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle) # 2 indicates general motion

        return (xnew, ynew, thetanew, path)

# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
def calculateClosestObstacleDistance(x, y):
        closestdist = 100000.0
        # Calculate distance to closest obstacle
        for (i,barrier) in enumerate(barriers):
                dx = barrier[0] - x
                dy = barrier[1] - y
                d = math.sqrt(dx**2 + dy**2)
                # Distance between closest touching point of circular robot and circular barrier
                dist = d - BARRIERRADIUS - ROBOTRADIUS
                if (dist < closestdist):
                        closestdist = dist
        return closestdist

# Draw the barriers on the screen
def draw_target(x,y):
        bcol = red
        x1 = int(u0 + k * x)
        y1 = int(v0 - k * y)
        pygame.draw.circle(screen, bcol, (x1, y1), int(k * BARRIERRADIUS), 0)

def drawBarriers(barriers):
        for (i,barrier) in enumerate (barriers):
                bcol = white
                pygame.draw.circle(screen, bcol, (int(u0 + k * barrier[0]), int(v0 - k * barrier[1])), int(k * BARRIERRADIUS), 0)


goflag = 0


# Main loop
rrt_planner = RRT2D(4, 3)
running = True

while(running):

        # Planning is finished; now do graphics
        screen.fill(grey)
        for loc in locationhistory:
            pygame.draw.circle(screen, grey,
                               (int(u0 + k * loc[0]), int(v0 - k * loc[1])), 3, 0)
        drawBarriers(barriers)

        # Did the user click the window close button?
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                        running = False
                        
                if pygame.mouse.get_pressed()[0]: # LEFT
                        pos = pygame.mouse.get_pos()
                        start_node = Node(-3.9,2.7)
                        goal_node = Node(3.0, -3.9)
                        rrt_planner.plan(start_node, goal_node)
                        path = rrt_planner.path
                        # rrt_path = [[0.1 * u0 + 0.1 * k * node.x, 0.1 * v0 + 0.1 * k * node.y] for node in rrt_path]

                        for node in path:
                            node_x = 25 * node.x
                            node_y = 25 * node.y
                            if node_x > 3.5 or node_y > 3.5:
                                continue
                            rrt_path.append([node_x, node_y])
                        print("rrt_path",rrt_path)
                
        if rrt_path:
            rrt_planner.draw_path()

            # For display of trail
            locationhistory.append((x, y))

            # # Planning is finished; now do graphics
            # screen.fill(black)
            # for loc in locationhistory:
            #         pygame.draw.circle(screen, grey, (int(u0 + k * loc[0]), int(v0 - k * loc[1])), 3, 0)
            # drawBarriers(barriers)
            draw_target(rrt_path[targetindex][0],rrt_path[targetindex][1])


            # Planning
            # We want to find the best benefit where we have a positive component for closeness to target,
            # and a negative component for closeness to obstacles, for each of a choice of possible actions
            bestBenefit = -100000
            FORWARDWEIGHT = 12
            OBSTACLEWEIGHT = 6666
    
    
            # Copy of barriers so we can predict their positions
            barrierscopy = copy.deepcopy(barriers)
    
            for _ in range(STEPSAHEADTOPLAN):
                    moveBarriers(dt)
    
    
            # Range of possible motions: each of vL and vR could go up or down a bit
            vLpossiblearray = (vL - MAXACCELERATION * dt, vL, vL + MAXACCELERATION * dt)
            vRpossiblearray = (vR - MAXACCELERATION * dt, vR, vR + MAXACCELERATION * dt)
            pathstodraw = [] # We will store path details here for plotting later
            newpositionstodraw = [] # Also for possible plotting of robot end positions
            for vLpossible in vLpossiblearray:
                    for vRpossible in vRpossiblearray:
                            # We can only choose an action if it's within velocity limits
                            if (vLpossible <= MAXVELOCITY and vRpossible <= MAXVELOCITY and vLpossible >= -MAXVELOCITY and vRpossible >= -MAXVELOCITY):
                                    # Predict new position in TAU seconds
                                    (xpredict, ypredict, thetapredict, path) = predictPosition(vLpossible, vRpossible, x, y, theta, TAU)
                                    pathstodraw.append(path)
                                    newpositionstodraw.append((xpredict, ypredict))
                                    # What is the distance to the closest obstacle from this possible position?
                                    distanceToObstacle = calculateClosestObstacleDistance(xpredict, ypredict)
                                    # Calculate how much close we've moved to target location
                                    # previousTargetDistance = math.sqrt((x - barriers[targetindex][0])**2 + (y - barriers[targetindex][1])**2)
                                    previousTargetDistance = math.sqrt((x - rrt_path[targetindex][0])**2 + (y - rrt_path[targetindex][1])**2)
                                    newTargetDistance = math.sqrt((xpredict - rrt_path[targetindex][0])**2 + (ypredict - rrt_path[targetindex][1])**2)
                                    # newTargetDistance = math.sqrt((xpredict - barriers[targetindex][0])**2 + (ypredict - barriers[targetindex][1])**2)
                                    distanceForward = previousTargetDistance - newTargetDistance
                                    # Alternative: how far have I moved forwards?
                                    # distanceForward = xpredict - x
                                    # Positive benefit
                                    distanceBenefit = FORWARDWEIGHT * distanceForward
                                    # Negative cost: once we are less than SAFEDIST from collision, linearly increasing cost
                                    if (distanceToObstacle < SAFEDIST):
                                            obstacleCost = OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
                                    else:
                                            obstacleCost = 0.0
                                    # Total benefit function to optimise
                                    benefit = distanceBenefit - obstacleCost
                                    if (benefit > bestBenefit):
                                            vLchosen = vLpossible
                                            vRchosen = vRpossible
                                            bestBenefit = benefit
            vL = vLchosen
            vR = vRchosen
    
            barriers = copy.deepcopy(barrierscopy)
    
    
    
            # # Planning is finished; now do graphics
            # screen.fill(black)
            # for loc in locationhistory:
            #         pygame.draw.circle(screen, grey, (int(u0 + k * loc[0]), int(v0 - k * loc[1])), 3, 0)
            # drawBarriers(barriers)
            # draw_target(rrt_path[targetindex][0],rrt_path[targetindex][1])
    
    
    
    
    
            # Draw robot
            u = u0 + k * x
            v = v0 - k * y
            pygame.draw.circle(screen, white, (int(u), int(v)), int(k * ROBOTRADIUS), 3)
            # Draw wheels as little blobs so you can see robot orientation
            # left wheel centre
            wlx = x - (W/2.0) * math.sin(theta)
            wly = y + (W/2.0) * math.cos(theta)
            ulx = u0 + k * wlx
            vlx = v0 - k * wly
            WHEELBLOB = 0.04
            pygame.draw.circle(screen, blue, (int(ulx), int(vlx)), int(k * WHEELBLOB))
            # right wheel centre
            wrx = x + (W/2.0) * math.sin(theta)
            wry = y - (W/2.0) * math.cos(theta)
            urx = u0 + k * wrx
            vrx = v0 - k * wry
            pygame.draw.circle(screen, blue, (int(urx), int(vrx)), int(k * WHEELBLOB))
    
    
            if (1):
                    # Draw paths: little arcs which show the different paths the robot is selecting between
                    # A bit complicated so don't worry about the details!
                    for path in pathstodraw:
                            #if path[0] = 1:    # Pure rotation: nothing to draw
                            if path[0] == 0:    # Straight line
                                    straightpath = path[1]
                                    linestart = (u0 + k * x, v0 - k * y)
                                    lineend = (u0 + k * (x + straightpath * math.cos(theta)), v0 - k * (y + straightpath * math.sin(theta)))
                                    pygame.draw.line(screen, (0, 200, 0), linestart, lineend, 1)
                            if path[0] == 2:    # General case: circular arc
                                    # path[2] and path[3] are start and stop angles for arc but they need to be in the right order to pass
                                    if (path[3] > path[2]):
                                            startangle = path[2]
                                            stopangle = path[3]
                                    else:
                                            startangle = path[3]
                                            stopangle = path[2]
                                    # Pygame arc doesn't draw properly unless angles are positive
                                    if (startangle < 0):
                                            startangle += 2*math.pi
                                            stopangle += 2*math.pi
                                    if (path[1][1][0] > 0 and path[1][0][0] > 0 and path[1][1][1] > 1):
                                            #print (path[1], startangle, stopangle)
                                            pygame.draw.arc(screen, (0, 200, 0), path[1], startangle, stopangle, 1)
    
            # Uncomment to also draw circles for predicted end positions of robot
            #for newposition in newpositionstodraw:
            #u = u0 + k * newposition[0]
            #v = v0 - k * newposition[1]
            #pygame.draw.circle(screen, (0,100,0), (int(u), int(v)), int(k * ROBOTRADIUS), 3)
            #time.sleep(1.0)
            #pygame.display.flip()
    
            # Update display
            pygame.display.flip()
    
            # Actually now move robot based on chosen vL and vR
            (x, y, theta, tmppath) = predictPosition(vL, vR, x, y, theta, dt)
    
    
            moveBarriers(dt)
    
    
            # Wraparound: check if robot has reached target; if so reset it to the other side, randomise
            # target position and add some more barriers to go again
            disttotarget = math.sqrt((x - rrt_path[targetindex][0])**2 + (y - rrt_path[targetindex][1])**2)
            if (disttotarget < (BARRIERRADIUS + ROBOTRADIUS)):
                    # Add new barriers
                    for i in range(3):
                            (bx, by) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]))
                            (bx, by, vx, vy) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]), random.uniform(-BARRIERVELOCITYRANGE, BARRIERVELOCITYRANGE), random.uniform(-BARRIERVELOCITYRANGE, BARRIERVELOCITYRANGE))
                            barrier = [bx, by, vx, vy]
                            barriers.append(barrier)
                    targetindex +=1
    
    
                    # Reset trail
                    locationhistory = []
    
    
            # Sleeping dt here runs simulation in real-time
            # time.sleep(dt / 50)
            time.sleep(0.1)
