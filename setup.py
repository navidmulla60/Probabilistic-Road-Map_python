import random 
import math
import pygame
import numpy as np


class Map:
    def __init__(self,start,goal,MapDimensions,obsdim,obsnum):
        self.start=start
        self.goal=goal
        self.MapDimensions=MapDimensions
        self.Maph,self.Mapw=self.MapDimensions

        self.MapWindowName='PRM Path Planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map=pygame.display.set_mode((self.Mapw,self.Maph))
        self.map.fill((255,255,255))
        self.nodeRad=2
        self.nodeThickness=0
        self.edgeThickness=1

        self.obstacles=[]
        self.obsdim=obsdim
        self.obsNumber=obsnum

        # define colour

        self.grey=(70,70,70)
        self.Blue=(0,0,255)
        self.Green=(0,255,0)
        self.Red=(255,0,0)
        self.white=(255,255,255)
        self.light_grey=(194, 202, 204)
        self.light_blue=(61, 173, 252)

    def drawMap(self,obstacles):
        pygame.draw.circle(self.map,self.Red,self.start,self.nodeRad+5,0)
        pygame.draw.circle(self.map,self.Green,self.goal,self.nodeRad+20,1)
        self.drawObs(obstacles)

    def drawObs(self,obstacles):
        obstaclesList=obstacles.copy()
        while(len(obstaclesList)>0):
            obstacle=obstaclesList.pop(0)
            pygame.draw.rect(self.map,self.grey,obstacle)

    def drawPath(self,path):
        print(path)
        for node in path:
            pygame.draw.circle(self.map, self.Green, node, self.nodeRad+3, 0)
        for i in range(len(path)-1):
            node1=path[i]
            node2=path[i+1]
            x1,y1=node1[0],node1[1]
            x2,y2=node2[0],node2[1]
            # if(self.start[0]<self.goal[0]) and (self.start[1]<self.goal[1]):
            #     print("Goal is on right side down")
            # if(x1<x2)and(y1<y2):
            #     if not Graph.crossObstacle(self,x1,x2,y1,y2):
            #         pygame.draw.line(self.map, self.Red, (x1,y1), (x2,y2), self.edgeThickness+5)
            pygame.draw.line(self.map, self.Red, (x1,y1), (x2,y2), self.edgeThickness+5)
            # if not Graph.crossObstacle(self,x1,x2,y1,y2):
            #     pygame.draw.line(self.map, self.Red, (x1,y1), (x2,y2), self.edgeThickness+5)



    def goal_wrt_start(self,x1,y1,x2,y2):
        if(self.start[0]<self.goal[0]) and (self.start[1]<self.goal[1]):
            print("Goal is on right side down")
            if(x1<=x2)and(y1<=y2):
                if not Graph.crossObstacle(self,x1,x2,y1,y2):
                    pygame.draw.line(self.map, self.Red, (x1,y1), (x2,y2), self.edgeThickness+5)

        elif (self.start[0]<self.goal[0]) and (self.start[1]>self.goal[1]):
            print("Goal is on right side up")

        elif (self.start[0]>self.goal[0]) and (self.start[1]<self.goal[1]):
            print("Goal is on Left side down")
        elif (self.start[0]>self.goal[0]) and (self.start[1]>self.goal[1]):
            print("Goal is on Left side up")   
    
            

class Graph:
    def __init__(self,start,goal,MapDimensions,obsdim,obsnum):
        (x,y)=start
        self.start=start
        self.goal=goal
        self.goalFlag=False
        self.Maph,self.Mapw=MapDimensions
        self.nodeRad=2

        self.x=[]
        self.y=[]
        self.parent=[]

        #starting point of tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        

        self.sp=[]  #sp=SamplePoints

        #for Obstacles

        self.obstacles=[]
        self.obsDim=obsdim
        self.obsNum=obsnum

        #path
        self.goalstate=None
        self.path=[]


    def makeRandomRect(self):
        uppercornerx=int(random.uniform(0,self.Mapw-self.obsDim))
        uppercornery=int(random.uniform(0,self.Maph-self.obsDim))
        return (uppercornerx, uppercornery)  

    def makeObs(self):
        obs=[]
        for i in range(0,self.obsNum):
            rectangle=None
            starttogoalcol=True
            while starttogoalcol:
                upper =self.makeRandomRect()
                rectangle=pygame.Rect(upper,(self.obsDim,self.obsDim))
                if rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal):
                    starttogoalcol=True
                else:
                    starttogoalcol=False
            obs.append(rectangle)
        self.obstacles=obs.copy()
        return obs

    def samples(self):

        x=int(random.uniform(0,self.Mapw))
        y=int(random.uniform(0,self.Maph))

        return x,y

    def number_of_node(self):
        return len(self.x)


    def isFree(self,node):
        
        (x,y)=(node[0],node[1])
        obs=self.obstacles.copy()
        # print(obs)
        while len(obs)>0:
            rectangle=obs.pop(0)
            if rectangle.collidepoint(x,y):
                # self.remove_node(node)
                return False
        return True

    def crossObstacle(self, x1,x2,y1,y2):
        obs=self.obstacles.copy()
        while(len(obs)>0):
            rectangle=obs.pop(0)
            for i in range(0,101):
                u=i/100
                x=x1*u + x2*(1-u)
                y=y1*u + y2*(1-u)
                if rectangle.collidepoint(x,y):
                    return True
        return False

    # def distance(self,x1,y1,x2,y2):
    #     px=(float(x1)-float(x2))**2
    #     py=(float(y1)-float(y2))**2
    #     return (px+py)**(0.5)

    def remove_node(self,n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self,parent,child):
        self.parent.insert(child,parent)

    def distance(self,n1,n2):
        (x1,y1)=(n1[0],n1[1])
        (x2,y2)=(n2[0],n2[1])
        px=(float(x1)-float(x2))**2
        py=(float(y1)-float(y2))**2
        return (px+py)**(0.5)

    def connect(self, n1,n2):
        (x1,y1)= (n1[0],n1[1])
        (x2,y2)= (n2[0],n2[1])
        if self.crossObstacle(x1,x2,y1,y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True

    def isitGoal(self,n):
        x,y= n[0],n[1]
        # print(x,y)
        # print(self.goal[0],self.goal[1])
        # d=((x-self.goal[0])**2)+((y-self.goal[1])**2)
        # print(d)
        # print((self.nodeRad+20)**2)
        if((x-self.goal[0])**2)+((y-self.goal[1])**2)<((self.nodeRad+20)**2):
            self.goalFlag=True
        # print(self.goalFlag)
        return self.goalFlag


    def findpath(self,sp):    #sp= Sample points
        self.sp=sp
        self.path.append(self.start)
        # print(self.sp)
        n=0
        while(not self.isitGoal(self.path[len(self.path)-1])):
            Distance=[]
            for i in range(len(self.sp)):   #len(self.sp)
                n1=self.path[len(self.path)-1]
                n2=self.sp[i]
                # print(n1)
                x1,y1= n1[0],n1[1]
                x2,y2= n2[0],n2[1]
                if(self.start[0]<self.goal[0]) and (self.start[1]<self.goal[1]):
                    # print("Goal is on right side down")
                    if(self.start[0]<=x2)and(self.start[1]<=y2):
                
                        Distance.append(self.distance(x1,y1,x2,y2))
            
            node_to_connect=np.argmin(Distance)

            # print(self.sp[node_to_connect])
            # print(Distance[np.argmin(Distance)])
            if self.sp[node_to_connect] not in self.path:
                self.path.append(self.sp[node_to_connect])
            self.sp.pop(node_to_connect)
            
            n+=1
            Distance.clear()
        # print(self.path)
        return self.path




        # print(np.argmin(Distance))
        # print(Distance[np.argmin(Distance)])


            # dist.append(Distance)
        #     if dist>Distance:
        #         dist=Distance
        #         self.path.append(n2)
        #         print(self.path)
        #     dist=0
        # print("After Looop...path is")
        # print(self.path)
        # print(n1[0])
        # print(n1[1])
        # x1,y1= n1[0],n1[1]
        # print(x1,y1)
        # print(self.sp[:5])
        # print(self.sp[0])
        


        