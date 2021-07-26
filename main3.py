from random import sample
from numpy.core.numeric import count_nonzero
import pygame
from pygame.draw import circle, line
from setup import Graph, Map
from pygame.locals import *
import sys
import random
import numpy as np


def main():

    sampleNodes=150
    edge_size=100
    dimension=(500,500)
    start=(20,50)
    goal=(410,410)
    obsdim=30           
    obsnum=10
    path={}
    # parent=[]
    # child=[]

    pygame.init()

    map=Map(start,goal,dimension,obsdim,obsnum)
    graph=Graph(start,goal,dimension,obsdim,obsnum)

    obstacles=graph.makeObs()
    map.drawMap(obstacles)
    nodes=[]
    
    #------------Generating Random Nodes-------------------#
   
    for i in range(sampleNodes):
        Newnode=graph.samples()
        # if graph.distance(Newnode,nodes[len(nodes)-1])<edge_size:
        if(graph.isFree(Newnode)):
            nodes.append(Newnode)
            # pygame.draw.circle(map.map, map.Red, (Newnode[0],Newnode[1]), map.nodeRad, map.nodeThickness)

    nodes=sorted(nodes) 

    for i in range(len(nodes)-1):
        if(graph.isFree(Newnode)):
            pygame.draw.circle(map.map, map.Red, (Newnode[0],Newnode[1]), map.nodeRad, map.nodeThickness)





    #-------------------------Creating Edges between the nodes------------------------------#
    for n in nodes:
        for k in nodes:
            if n!=k:
                dist=graph.distance(n,k)
                if dist<edge_size:
                    # graph.connect(n,k)
                    if not graph.crossObstacle(n[0],k[0],n[1],k[1]):
                        pygame.draw.line(map.map, map.light_grey, [n[0],n[1]],[k[0],k[1]], map.edgeThickness)
                        if n in list(path):
                            path[n].append(k)
                        path[n]=[k]
    
    # print(path)
    


    
    #------ Attaching start node and Goal to the RoadMap (network of vertext and edges)-----#

    Distance_startNode=[]
    Distance_GoalNode=[]

    for node in nodes:
        meanDist_start=graph.distance(node,start)
        Distance_startNode.append(meanDist_start)
        meanDist_Goal=graph.distance(node,goal)
        Distance_GoalNode.append(meanDist_Goal)
    start_inx=np.argmin(Distance_startNode)
    goal_idx=np.argmin(Distance_GoalNode)

    snode=nodes[start_inx]
    gnode=nodes[goal_idx]
    pygame.draw.line(map.map, map.Red, [start[0],start[1]],[snode[0],snode[1]], map.edgeThickness+2)
    pygame.draw.line(map.map, map.Red, [goal[0],goal[1]],[gnode[0],gnode[1]], map.edgeThickness+2)
    nodes.pop(start_inx)
   



    #----- calculating path----#

    
    distance_from_start_to_goal=graph.distance(snode,gnode)
    final_path=[]
    final_path.append(start)
    final_path.append(snode)

    distance=[]
    node_to_remove=None
    node_to_be_remove=[]
    newNodeTopath=None

    # print("----Lenght of nodes------")
    # print(len(nodes))
    # print("----Lenght of nodes------")

    #--------------adding nodes to path if new node is nearest to existing node 
    # -------------and if its distance (from new node to goal) is minimum compare to the distance from (previous ndoes to goal)---------------#

    while(not(newNodeTopath==gnode)):
        extendable_node=final_path[len(final_path)-1]

        for node in nodes:
            dist=graph.distance(extendable_node,node)
            distance.append(dist)
        idx=np.argmin(distance)
        # print(idx)
        newNodeTopath=nodes[idx]
        distance_from_newPoint_to_Goal=graph.distance(newNodeTopath,gnode)
        if(distance_from_newPoint_to_Goal<distance_from_start_to_goal):
            if not graph.crossObstacle(extendable_node[0],newNodeTopath[0],extendable_node[1],newNodeTopath[1]):
                pygame.draw.line(map.map, map.light_blue, [extendable_node[0],extendable_node[1]],[newNodeTopath[0],newNodeTopath[1]], map.edgeThickness+2)
                final_path.append(newNodeTopath)
                distance_from_start_to_goal=distance_from_newPoint_to_Goal
        nodes.pop(idx)
        distance.clear()


    #-------------------------------Optimization of Path----------------------------------------------------#

    # i`m taking 3 nodes from the final_path (a,b,c) 
    # and checking obstacte between a and c... if no obstacle then remove b and add edge between a and c
    # if obstacle present then check for a and b and add edge between a and b 

    startcount=0
    endcount=3
    opti_path=[]
    elements=len(final_path)
    ele_to_be_added=[]
    
    print(elements % 3)
    print(elements)
    if not (elements % 3==0):
        e=elements % 3
        a=0
        while(a<e):
            node_to_be_added=final_path.pop(final_path.index(final_path[elements-(a+1)]))
            ele_to_be_added.append(node_to_be_added)

            a+=1
        ele_to_be_added.reverse()
        elements=len(final_path)
        # print("Inside IF")

    while(elements>0):
        # print(endcount,elements)
        # print(startcount,endcount)
        if(endcount<elements):
            [a,b,c]=final_path[startcount:endcount]
            if not graph.crossObstacle(a[0],c[0],a[1],c[1]):
                pygame.draw.line(map.map, map.Green, [a[0],a[1]],[c[0],c[1]], map.edgeThickness+1)
                opti_path.append(a)
                opti_path.append(c)
                id_a,id_b,id_c=final_path.index(a),final_path.index(b),final_path.index(c)
                final_path.pop(id_a)
                final_path.pop(id_b)
                final_path.pop(id_c)

                startcount=endcount
                endcount=startcount+3

                elements=len(final_path)

            elif not graph.crossObstacle(a[0],b[0],a[1],b[1]):
                pygame.draw.line(map.map, map.Green, [a[0],a[1]],[b[0],b[1]], map.edgeThickness+1)
                id_a,id_b=final_path.index(a),final_path.index(b)
                opti_path.append(a)
                opti_path.append(b)
                opti_path.append(c)
                final_path.pop(id_a)
                final_path.pop(id_b)
                print("Inside AB edge")
                startcount=endcount-1
                endcount=startcount+3

                elements=len(final_path)
           
        else:
            break
    opti_path.extend(ele_to_be_added)
    opti_path.append(goal)
    # print([start]+final_path[:])
    opti_path=[start]+opti_path


    for i in range(len(opti_path)-1):
        oa=opti_path[i]
        ob=opti_path[i+1]
        if not graph.crossObstacle(oa[0],ob[0],oa[1],ob[1]):
                pygame.draw.line(map.map, map.Green, [oa[0],oa[1]],[ob[0],ob[1]], map.edgeThickness+3)
        pygame.display.update()
        pygame.event.clear()
        pygame.event.wait(0)       

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


    # Randpoints=graph.samples(sampleNodes)

    # n=graph.number_of_node()
   
    # connected_edge=[]
 


if __name__ == '__main__':
   
    main()
    print("-------------------------------")
    print("----------Try Again------------")
    print("-------------------------------")

    