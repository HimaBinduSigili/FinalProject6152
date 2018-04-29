# -*- coding: utf-8 -*-
"""
Created on Sat Apr 21 09:29:25 2018

@author: bindu
"""
import math
import sys
import random
import klampt.model.collide as collide
sys.path.append("./kinematics/")
class RRT():    
    
    def __init__(self,world,robot, start, goal,obs,randArea, maxIter=5000):
        self.world=world
        self.robot=robot
        self.start = Node(start[0],start[1],start[2])
        self.end = Node(goal[0],goal[1],goal[2])
        self.obs=obs
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.maxIter = maxIter

    def Planning(self):
        self.nodeList = [self.start] #Tree init
        self.nodeList2 = [self.end]  #Tree goal
        #robo=self.robot
        #collisionChecker = collide.WorldCollider(self.world)
        flaggoal=False
        while self.maxIter>0:
            # Random Sampling
            rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand),random.uniform(0,2*math.pi)]
           
            print("random  ny:",rnd)
            colli_flag,obsName=collisionchecking(rnd,self.obs)
            if colli_flag:
                    self.maxIter-=1   
                    continue#if the configuration is colliding with any obstacle continue the while loop
                    
            result,newnode = self.Extend(rnd,self.nodeList)
            #print("result is {} and newnode is {}".format(result,newnode))
            if not result=="Trapped":
                self.nodeList.append(newnode)
                result1,list1=self.Extend([newnode.x,newnode.y,newnode.theta1],self.nodeList2)
                #print("result is {} and list1 is {}".format(result1,list1))
                if result1=="Reached":
                    self.nodeList2.append(list1)
                    print("Goal!!!")
                    flaggoal=True
                    break
            #swap the trees
                
            #print("swapping trees")
            #print("Before swap Ta:{}".format([[nodes.x,nodes.y,nodes.theta1,nodes.parent] for nodes in self.nodeList]))
            #print("Tb :{}".format([[nodes1.x,nodes1.y,nodes1.theta1,nodes1.parent] for nodes1 in self.nodeList2]))
            nodelis=self.nodeList
            self.nodeList=self.nodeList2
            self.nodeList2=nodelis
            #print("After swap Ta:{}".format([[nodes.x,nodes.y,nodes.theta1] for nodes in self.nodeList]))
            #print("Tb :{}".format([[nodes1.x,nodes1.y,nodes1.theta1] for nodes1 in self.nodeList2]))
            self.maxIter -= 1
            print(self.maxIter)
            
        #store all the configurations in a list (path) to ease the simulation    
        if flaggoal:
            path=[]
            lastIndex = len(self.nodeList2) - 1
            while self.nodeList2[lastIndex].parent is not None:
                node = self.nodeList2[lastIndex]
                path.append([node.x, node.y,node.theta1,[self.nodeList2[node.parent].x,self.nodeList2[node.parent].y,self.nodeList2[node.parent].theta1],node.fromVel])
                #print("Ingoallist2:",node.x,node.y,node.theta1,node.parent)
                lastIndex = node.parent
            lastIndex = len(self.nodeList) - 1
            if self.nodeList2[0].x==self.start.x:  
                path.append([self.start.x, self.start.y,self.start.theta1,[self.start.x, self.start.y,self.start.theta1],""])
            elif self.nodeList2[0].x==self.end.x:
                path.append([self.end.x, self.end.y,self.end.theta1,[self.end.x, self.end.y,self.end.theta1],""])
            path1=list(reversed(path))
            while self.nodeList[lastIndex].parent is not None:
                node = self.nodeList[lastIndex]
                path1.append([node.x, node.y,node.theta1,[self.nodeList[node.parent].x,self.nodeList[node.parent].y,self.nodeList[node.parent].theta1],node.fromVel])
                #print("Ingoallist:",node.x,node.y,node.theta1,node.parent)
                lastIndex = node.parent
            if self.nodeList2[0].x==self.end.x:  
                path1.append([self.start.x, self.start.y,self.start.theta1,[self.start.x, self.start.y,self.start.theta1],""])
                print("TB :",list(reversed(path1)) )
                return list(reversed(path1)) 
            elif self.nodeList2[0].x==self.start.x:
                path1.append([self.end.x, self.end.y,self.end.theta1,[self.end.x, self.end.y,self.end.theta1],""])
                print("TB :",path1)
                return path1
            
        else:     #show the path of the robot even if it doesnot reach the goal
            print("Max Iterations reached No goal found!!")
            #self.robot.setConfig([self.start.x, self.start.y,self.start.theta1])
            path=[]
            lastIndex = len(self.nodeList2) - 1
            while self.nodeList2[lastIndex].parent is not None:
                node = self.nodeList2[lastIndex]
                path.append([node.x, node.y,node.theta1,[self.nodeList2[node.parent].x,self.nodeList2[node.parent].y,self.nodeList2[node.parent].theta1],node.fromVel])
                lastIndex = node.parent
            lastIndex = len(self.nodeList) - 1
            if self.nodeList2[0].x==self.start.x:  
                path.append([self.start.x, self.start.y,self.start.theta1,[self.start.x, self.start.y,self.start.theta1],""])
            elif self.nodeList2[0].x==self.end.x:
                path.append([self.end.x, self.end.y,self.end.theta1,[self.end.x, self.end.y,self.end.theta1],""])
            path1=list(reversed(path))
            while self.nodeList[lastIndex].parent is not None:
                node = self.nodeList[lastIndex]
                path1.append([node.x, node.y,node.theta1,[self.nodeList[node.parent].x,self.nodeList[node.parent].y,self.nodeList[node.parent].theta1],node.fromVel])
                lastIndex = node.parent
            if self.nodeList2[0].x==self.end.x:  
                path1.append([self.start.x, self.start.y,self.start.theta1,[self.start.x, self.start.y,self.start.theta1],""])
                print(list(reversed(path1)))
                return list(reversed(path1))                
            elif self.nodeList2[0].x==self.start.x:
                path1.append([self.end.x, self.end.y,self.end.theta1,[self.end.x, self.end.y,self.end.theta1],""])
                print(path1)
                return path1
            #return [[self.nodeList[len(self.nodeList)-1].x,self.nodeList[len(self.nodeList)-1].y,self.nodeList[len(self.nodeList)-1].theta1]]

          
    #to get the nearest node in the tree
    def GetNearestListIndex(self, nodeList, rnd):      
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 + (node.theta1-rnd[2]) for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind
    
    #to extend the tree towards the randompoint to find the new node
    def Extend(self,rnd,nodelist):
        # Find nearest node
        nind = self.GetNearestListIndex(nodelist, rnd)
        # expand tree
        nearestNode = nodelist[nind]
        a=nearestNode.x
        b=nearestNode.y
        c=nearestNode.theta1
        #print("nearest a b c of Ta: ",a,b,c)
        boo,a,b,c,u1= self.integrate(a,b,c,rnd)
        if boo:
            return "Trapped",[]
        else:
            newNode=Node(a,b,c)
            newNode.parent = nind
            newNode.fromVel=u1
            print("new node is",newNode.x,newNode.y,newNode.theta1,u1)
            if closeness([a,b,c],rnd):
                return "Reached",newNode
            else:
                return "Advanced",newNode
    
    # Using Runge-Kutta method to integrate towards the random point
    def integrate(self,a,b,c,rnd):
        x,y,teta, hs = a,b,c,0.05
        x1,y1,teta1, hs = a,b,c,0.05
        #collisionChecker = collide.WorldCollider(self.world)
        iscollideP=False
        iscollideN=False
        #itera=20000
        for i in range(100):
            if not iscollideP:
                x,y,teta = rK3(x,y,teta, fx, fy, f0, hs,rnd)
            if not iscollideN:
                x1,y1,teta1 = rK3(x1,y1,teta1, fx1, fy1, f0, hs,rnd)
            #print("integratefunction x y teta :",x,y,teta)
            node1=x,y,teta
            qc=x,y,teta	
            colli_flag,obsName=collisionchecking(qc,self.obs)
            if colli_flag:
                #print("collision in integratefunction")
                iscollideP=True
                    #return True,x,y,teta
            #qlis.append(qc)
            px1,rx1=dist(node1,rnd)
            
            node2=x1,y1,teta1
            qc=x1,y1,teta1
            colli_flag,obsName=collisionchecking(qc,self.obs)
            if colli_flag:
                #print("collision in integratefunction")
                iscollideN=True
                    #return True,x1,y1,teta1
            #qlis.append(qc)
            if iscollideP and iscollideN:
                return True,1,1,1,"N"
            px2,rx2=dist(node2,rnd)
            if not iscollideP:
                if closeness(node1,rnd):
                    print("close to rnd1")
                    return False,x,y,teta,"P"
            if not iscollideN:
                if closeness(node2,rnd):
                    print("close to rnd2")
                    return False,x1,y1,teta1,"N"
        #print("2000 iterations finished adding new node")
        
        if px1+rx1>px2+rx2:
            print("close to rnd2")
            return False,x1,y1,teta1,"N"
        else:
            print("close to rnd1")
            return False,x,y,teta,"P"

    def simulate(self,final):
        print("inside simulate")
        fqlist=[]
        for i in range(len(final)-1):
            if not [final[i][0],final[i][1],final[i][2]]==final[i][3]:
                if [final[i-1][0],final[i-1][1],final[i-1][2]]==final[i][3]:
                    fqlist.append(self.integratefinal(final[i-1][0],final[i-1][1],final[i-1][2],[final[i][0],final[i][1],final[i][2]],final[i][4]))
                elif [final[i+1][0],final[i+1][1],final[i+1][2]]==final[i][3]:
                    fqlist.append(list(reversed(self.integratefinal(final[i+1][0],final[i+1][1],final[i+1][2],[final[i][0],final[i][1],final[i][2]],final[i][4]))))
        return fqlist

    def integratefinal(self,a,b,c,rnd,u1):
        #print("inside integratefinal")
        x,y,teta,hs = a,b,c,0.05
        qlist=[]
        qlist.append([a,b,c])
        #itera=20000
        for i in range(100):
            if u1=="P":
                x,y,teta = rK3(x,y,teta, fx, fy, f0, hs,rnd)
            elif u1=="N":
                x,y,teta = rK3(x,y,teta, fx1, fy1, f0, hs,rnd)
            #print("integratefunction x y teta :",x,y,teta)
            node1=x,y,teta
            if closeness(node1,rnd):
                print("close to rnd")
                qlist.append(node1)
                #print(qlist)
                return qlist
            qlist.append(node1)
        #print(qlist)
        return qlist

#creating a node class representing vertices in the tree
class Node():
    def __init__(self, x, y,theta1):
        self.x = x
        self.y = y
        self.theta1 = theta1
        self.parent = None
        self.fromVel=""

#set a limit of 0.05 to check if the two points are close enough and 0.1 
#to check the orientation simalirity 
def closeness(node1,node2):
    px=math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1])** 2)
    rx=abs(node1[2]-node2[2])
    if px<=0.05 and rx<=0.1:
        return True
    else:
        return False

#gives the distabce between the two points    
def dist(node1,node2):
    px=math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1])** 2)
    rx=abs(node1[2]-node2[2])
    return px,rx

#Here in Runge-kutta method two scenarios are considered one is with a positive 
#velocity(moving forward) and other is for negative velocity(moving backward) 
def rK3(x, y, teta, fx, fy, f0, hs,rnd):
    a1 = fx(x, y, teta)*hs
    b1 = fy(x, y, teta)*hs
    c1 = f0(x, y, teta,rnd)*hs
    ak = x + a1*0.5
    bk = y + b1*0.5
    ck = teta + c1*0.5
    a2 = fx(ak, bk, ck)*hs
    b2 = fy(ak, bk, ck)*hs
    c2 = f0(ak, bk, ck,rnd)*hs
    ak = x + a2*0.5
    bk = y + b2*0.5
    ck = teta + c2*0.5
    a3 = fx(ak, bk, ck)*hs
    b3 = fy(ak, bk, ck)*hs
    c3 = f0(ak, bk, ck,rnd)*hs
    ak = x + a3
    bk = y + b3
    ck = teta + c3
    a4 = fx(ak, bk, ck)*hs
    b4 = fy(ak, bk, ck)*hs
    c4 = f0(ak, bk, ck,rnd)*hs
    x = x + (a1 + 2*(a2 + a3) + a4)/6
    y = y + (b1 + 2*(b2 + b3) + b4)/6
    teta = teta + (c1 + 2*(c2 + c3) + c4)/6
    return x,y,teta

#positive velocity(moving forward)
def fx(x, y, teta):
    return (0.15*math.cos(teta))
def fy(x, y, teta):
    return (0.15*math.sin(teta))
def f0(x, y, teta,rnd):
    return rnd[2]-teta

#negative velocity(moving backward)
def fx1(x, y, teta):
    return (-0.15*math.cos(teta))
def fy1(x, y, teta):
    return (-0.15*math.sin(teta))

#Collision Checking
def collisionchecking(node,obstacleList):
    for (ox, oy, a,b,tr) in obstacleList:
        dx = ox - node[0]
        dy = oy - node[1]
        d = ((dx * dx)/(a*a) + (dy * dy)/(b*b))
        if d <= 1+0.04:
            return True,tr
    if abs(node[0])>3 or abs(node[1])>3:
        return True,"Wall"
    return False,""



