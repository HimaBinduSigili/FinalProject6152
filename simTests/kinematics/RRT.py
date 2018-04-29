
import math
import random
import klampt.model.collide as collide
sys.path.append("./kinematics/")
from sphero6DoF import sphero6DoF
from turtlebot import turtlebot
class RRT():    
    
    def __init__(self,world=WorldModel(),robot, start, goal,randArea, expandDis=1.0,goalSampleRate=5, maxIter=500):
        self.world=world
        self.robot=robot
        self.start = start
        self.end = end
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter

    def Planning(self):
        self.nodeList = [self.start]
        robo=self.robot
        collisionChecker = collide.WorldCollider(self.world)
        while True:
            # Random Sampling
            if random.randint(0,100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
 	    newNode = VDP2(newNode.x,newNode.y,theta,self.expandDis)
            newNode.parent = nind
            
            robo.setConfig(newNode)
            collRT2 = collisionChecker.robotObjectCollisions(self.world.robo)
            for i,j in collRT2:
                collisionFlag = True
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            #if animation:
                #self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind
    def rK3(a, b, c, fa, fb, fc, hs):
        a1 = fa(a, b, c)*hs
    	b1 = fb(a, b, c)*hs
    	c1 = fc(a, b, c)*hs
    	ak = a + a1*0.5
    	bk = b + b1*0.5
    	ck = c + c1*0.5
    	a2 = fa(ak, bk, ck)*hs
    	b2 = fb(ak, bk, ck)*hs
    	c2 = fc(ak, bk, ck)*hs
    	ak = a + a2*0.5
    	bk = b + b2*0.5
    	ck = c + c2*0.5
    	a3 = fa(ak, bk, ck)*hs
    	b3 = fb(ak, bk, ck)*hs
    	c3 = fc(ak, bk, ck)*hs
    	ak = a + a3
    	bk = b + b3
    	ck = c + c3
    	a4 = fa(ak, bk, ck)*hs
    	b4 = fb(ak, bk, ck)*hs
    	c4 = fc(ak, bk, ck)*hs
    	a = a + (a1 + 2*(a2 + a3) + a4)/6
    	b = b + (b1 + 2*(b2 + b3) + b4)/6
    	c = c + (c1 + 2*(c2 + c3) + c4)/6
    	return a, b, c

    def fa2(a, b, c):
        return 0.9*(1 - b*b)*a - b + math.sin(c)

    def fb2(a, b, c):
        return a

    def fc2(a, b, c):
    	return 0.5

    def VDP2(a1,b1,c1,hs1):
        a, b, c, hs = a1,b1,c1,hs1
        for i in range(20):
            a, b, c = rK3(a, b, c, fa2, fb2, fc2, hs)
	return a,b,c


class Node():
    def __init__(self, x, y,theta1):
        self.x = x
        self.y = y
        self.theta1 = theta1
        self.parent = None


def GetPathLength(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le
