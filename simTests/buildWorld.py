#!/usr/bin/python

##Author: Saurav Agarwal
##E-mail: sagarw10@uncc.edu

## The functions are for creating additional components for the environment
##  1. getWall: Get the geometry of a wall based on given dimensions
##  2. getWall_terrain: Attach a single wall to terrain
##  3. getDoubleRoomDoor: Build two rooms with a door on the separating wall
##  4. getDoubleRoomWindow: Build two rooms with a window on the separating wall
import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import time
import math

def getWall(dimX, dimY, dimZ, pos = [0, 0, 0], rotZ = 0):
        ## Get the wall geometry
	wall = Geometry3D()
	wall.loadFile("cube.off") # The wall is based upon cube primitive of unit dimension
        ## Not sure why the scaling is done through the rotation matrix, works though!
	#wall.transform([dimX, 0, 0, 0, dimY, 0, 0, 0, dimZ], pos)
        wall.scale(dimX, dimY, dimZ)
        rotMat = so3.rotation((0, 0, 1), math.radians(rotZ))
        wall.transform(rotMat, pos)
        return wall

def getTree(dimX, dimY, dimZ, pos = [0, 0, 0], rotZ = 0):
        ## Get the wall geometry
	wall = Geometry3D()
	wall.loadFile("cylinder.off") # The wall is based upon cube primitive of unit dimension
        ## Not sure why the scaling is done through the rotation matrix, works though!
	#wall.transform([dimX, 0, 0, 0, dimY, 0, 0, 0, dimZ], pos)
        wall.scale(dimX, dimY, dimZ)
        rotMat = so3.rotation((1, 0, 0), math.radians(rotZ))
        wall.transform(rotMat, pos)
        return wall

def getTop(dimX, dimY, dimZ, pos = [0, 0, 0], rotZ = 0):
        ## Get the wall geometry
	wall = Geometry3D()
	wall.loadFile("sphere.off") # The wall is based upon cube primitive of unit dimension
        ## Not sure why the scaling is done through the rotation matrix, works though!
	#wall.transform([dimX, 0, 0, 0, dimY, 0, 0, 0, dimZ], pos)
        wall.scale(dimX, dimY, dimZ)
        rotMat = so3.rotation((1, 0, 0), math.radians(rotZ))
        wall.transform(rotMat, pos)
        return wall


def getWall_terrain(world, dimX, dimY, dimZ, pos = [0, 0, 0], nameWall="wall", color = [0.85, 0.85, 0.85, 1]):
        ## Attach a single wall to the world
        wall = getWall(dimX, dimY, dimZ, pos)
	world_wall = world.makeTerrain(nameWall)
	world_wall.geometry().set(wall)
        r = color[0]
        g = color[1]
        b = color[2]
        alpha = color[3]
	world_wall.appearance().setColor(r, g, b, alpha)
	return world_wall

def getDoubleRoomDoor(world, dimX, dimY, dimZ, color = [0.85, 0.85, 0.85, 1], wall_thickness = 0.01):
    ## Build a double room with a single door in the middle of the wall
    ## The width of the door is dimX/4
    x2 = dimX/2.0
    x8 = dimX/8.0
    y2 = dimY/2.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 0)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)
    w5 = getWall(3*x8, wall_thickness, dimZ, [-x2, 0, 0], 0)
    w6 = getWall(3*x8, wall_thickness, dimZ, [x8, 0, 0], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4, w5, w6]):
	g = Geometry3D(elem)
	DRDgeom.setElement(i,g)
    drd_setup = world.makeRigidObject("DRD")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    drd_setup.appearance().setColor(r, g, b, alpha)

def getDoubleRoomWindow(world, dimX, dimY, dimZ, color = [0.85, 0.85, 0.85, 1], wall_thickness = 0.01):
    ## Build a double room with a single window in the middle of the wall
    ## The dimensions of the window are dimX/4, dimZ/3  
    x2 = dimX/2.0
    x8 = dimX/8.0
    y2 = dimY/2.0
    z3 = dimZ/3.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 0)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)
    w5 = getWall(dimX, wall_thickness, z3, [-x2, 0, 0], 0)
    w6 = getWall(3*x8, wall_thickness, z3, [-x2, 0, z3], 0)
    w7 = getWall(3*x8, wall_thickness, z3, [x8, 0, z3], 0)
    w8 = getWall(dimX, wall_thickness, z3, [-x2, 0, 2.0*z3], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4, w5, w6, w7, w8]):
	g = Geometry3D(elem)
	DRDgeom.setElement(i,g)
    drd_setup = world.makeRigidObject("DRD")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    drd_setup.appearance().setColor(r, g, b, alpha)


def myEnvironment(world, dimX, dimY, dimZ, color = [0, 0.85, 0.35, 1], wall_thickness = 0.01):
## The dimensions of the window are dimX/4, dimZ/3  
    x2 = dimX/2.0
    #x8 = dimX/8.0
    y2 = dimY/2.0
    z3 = dimZ/3.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 0)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4]):
	g = Geometry3D(elem)
	DRDgeom.setElement(i,g)
    drd_setup = world.makeRigidObject("DRD")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    tree1 = Geometry3D()
    tree1a = getTree(0.1,0.1,1,[0.2,-0.2,0],0)
    tree1_setup= world.makeRigidObject("tree1")
    tree1_setup.geometry().set(tree1a)
    tree1_setup.appearance().setColor(0.44, 0.27, 0.07, alpha)

    tree1t = Geometry3D()
    tree1at = getTop(0.15,0.15,0.3,[0.2,-0.2,1],0)
    tree1t_setup= world.makeRigidObject("tree1top")
    tree1t_setup.geometry().set(tree1at)
    tree1t_setup.appearance().setColor(0, 1, 0, alpha)

    tree2 = Geometry3D()
    tree2a = getTree(0.25,0.35,0.75,[1.5,-2,0],0)
    tree2_setup= world.makeRigidObject("tree2")
    tree2_setup.geometry().set(tree2a)
    tree2_setup.appearance().setColor(0.44, 0.17, 0.07, 0.8)

    tree2t = Geometry3D()
    tree2at = getTop(0.25,0.55,0.3,[1.5,-2,0.75],0)
    tree2t_setup= world.makeRigidObject("tree2top")
    tree2t_setup.geometry().set(tree2at)
    tree2t_setup.appearance().setColor(0, 1, 0, alpha)

    tree3 = Geometry3D()
    tree3a = getTree(0.32,0.18,0.65,[-2,2,0],0)
    tree3_setup= world.makeRigidObject("tree3")
    tree3_setup.geometry().set(tree3a)
    tree3_setup.appearance().setColor(0.44, 0.07, 0.07, 0.9)

    tree3t = Geometry3D()
    tree3at = getTop(0.32,0.28,0.3,[-2,2,0.65],0)
    tree3t_setup= world.makeRigidObject("tree3top")
    tree3t_setup.geometry().set(tree3at)
    tree3t_setup.appearance().setColor(0, 0.75, 0.07, 0.9)

    goal = Geometry3D()
    goala = getTop(0.05,0.05,0.05,[2.6,2,0.1],0)
    goal_setup= world.makeRigidObject("tree3top")
    goal_setup.geometry().set(goala)
    goal_setup.appearance().setColor(1, 0.07, 0.07, 0.9)


    drd_setup.appearance().setColor(r, g, b, alpha)
    #box1_setup.appearance().setColor(0.44, 0.27, 0.07, alpha)


