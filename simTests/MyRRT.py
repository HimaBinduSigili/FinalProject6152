#!/usr/bin/python
##Author: Hima Bindu Sigili
##E-mail: hsigili@uncc.edu
## The file demonstrates:
##   1. Adding trees and fence to the environment (refer to buildWorldnew.py as well)
##   2. Setting up a unicycle robot
##   3. Perform collision checking
##   4. Building an RRT and visualize
##   5. finding a path from initial to goal configuration

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import klampt.model.collide as collide
import RRTUtil as RRT
import time
import math
import random
import buildWorldnew as bW
sys.path.append("./kinematics/")
from sphero6DoF import sphero6DoF
from turtlebot import turtlebot
from decimal import Decimal
from unicycle import unicycle

def collisionchecking(node,obstacleList):
    for (ox, oy, a,b,tr) in obstacleList:
        dx = ox - node[0]
        dy = oy - node[1]
        d = ((dx * dx)/(a*a) + (dy * dy)/(b*b))
        if d <= 1+0.04:
            return True,tr
    if abs(node[0])>wx/2 or abs(node[1])>wy/2:
        return True,"Wall"
    return False,""


if __name__ == "__main__":
    if len(sys.argv)<=1:
        print("USAGE: MyRRT.py [world_file]")
        exit()

    ## Creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

    ##Building my environment with trees and fruits
    wx=6
    wy=6
    wz=0.3
    obs=bW.myEnvironment(world, wx, wy, wz)

    ## Add the world to the visualizer
    vis.add("world",world)

    vp = vis.getViewport()
    vp.w,vp.h = 1500,1100
    vis.setViewport(vp)

    ## Create robot object. Change the class to the desired robot. 
    ## Also, make sure the robot class corresponds to the robot in simpleWorld.xml file

    robot=unicycle(world.robot(0), "unicycle", vis)
    robot.setAltitude(0.1)

    ## Display the world coordinate system
    vis.add("WCS", [so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 12)


    print "Visualization items:"
    vis.listItems(indent=2)
    #vis.autoFitCamera()
    vis.addText("textCol", "No collision")
    vis.setAttribute("textCol","size",24)
    collisionFlag = False
    collisionChecker = collide.WorldCollider(world)
    

    ## On-screen text display
    vis.addText("textConfig","Robot configuration: ")
    vis.setAttribute("textConfig","size",24)
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

    print "Starting visualization window#..."

    ## Run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation of Unicycle Robot")
    
    simTime = 3000
    startTime = time.time()
    oldTime = startTime
    start=[-1,-1,math.pi/4]
    goal=[2.5,2,math.pi/4]
    goalstartflag = False
    colli_flag,obsName=collisionchecking(goal,obs)
    if colli_flag:
        strng = "Goal collides with " + obsName
        print(strng)
        goalstartflag =True
        vis.addText("textCol", strng)
        vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
    colli_flag,obsName=collisionchecking(goal,obs)
    if colli_flag:
        strng = "Start configuration collides with " + obsName
        print(strng)
        goalstartflag =True
        vis.addText("textCol", strng)
        vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
    
    #Start the implementation of RRT only if the robot start and goal positions 
    #does not collide with the obstacles in the environment
    if not goalstartflag:
        rrt = RRT.RRT(bW,world,robot,start, goal,obs,randArea=[-wx/2, wy/2])
        path = rrt.Planning()
        print("In Main file",path)
        fqlist=rrt.simulate(path)
        #Show the Visualization once the path is find
        vis.show()
        while time.time() - startTime < simTime:
            for qIt in fqlist:
                for q in qIt:
                    vis.lock()
                    robot.setConfig(q)
                    bW.getfruits(world,0.01,0.01,0.01,0,"f",[q[0],q[1],0])
                    vis.add("path",world)
                    #bW.getTop(0.01,0.01,0.01,[q[0],q[1],0],0)
                    q2f = [ '{0:.2f}'.format(elem) for elem in q]
                    strng = "Robot configuration: " + str(q2f)
                    vis.addText("textConfig", strng)
                    #time.sleep(0.01)
                    vis.unlock()
        vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()

