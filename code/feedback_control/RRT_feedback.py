from scipy.spatial import KDTree
import numpy as np
import random
import math
import time
import copy

class Node(object):
    def __init__(self, x, y, parent=-1):
        self.x = x
        self.y = y
        self.parent = parent

class RRT_star(object):
    def __init__(self,expand_dis=200):
        self.expand_dis=expand_dis
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 200
        self.avoid_dist = 200
    def plan(self,vision, start_x, start_y, goal_x, goal_y):
        path_found=False
        # Obstacles
        obstacle_x = [-999999]
        obstacle_y = [-999999]
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacle_x.append(robot_blue.x)
                obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacle_x.append(robot_yellow.x)
                obstacle_y.append(robot_yellow.y)
        obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        self.node_list=[Node(start_x,start_y)]
        while True:
            rand_x = (random.random() * (self.maxx - self.minx)) + self.minx
            rand_y = (random.random() * (self.maxy - self.miny)) + self.miny
            if random.random()<=0.2:
                rand_x=goal_x
                rand_y=goal_y
            rand_Node=Node(rand_x,rand_y)
            nearest_id=self.get_nearest_node_index(rand_Node)
            nearest_node=self.node_list[nearest_id]
            #print(nearest_id)
            #new node
            theta=math.atan2(rand_y-nearest_node.y,rand_x-nearest_node.x)
            new_node=copy.deepcopy(nearest_node)
            new_node.x+=self.expand_dis*math.cos(theta)
            new_node.y+=self.expand_dis*math.sin(theta)
            new_node.parent=nearest_id
            #print(new_node.x,new_node.y)
            if not self.check_obs(nearest_node,theta,obstree):
                continue 
            self.node_list.append(new_node)
            #print(new_node.x,new_node.y)
            dx=new_node.x-goal_x
            dy=new_node.y-goal_y
            d=math.hypot(dx,dy)
            if d<self.expand_dis:
                print("find goal")
                path_found=True
                break
        path_x, path_y = [goal_x], [goal_y]
        if path_found:
            path_x.append(new_node.x)
            path_y.append(new_node.y)
            parent =new_node.parent
            while parent != -1:
                p_parent=self.node_list[parent].parent
                if p_parent!=-1:
                    if math.hypot(path_x[-1]-self.node_list[p_parent].x,path_y[-1]-self.node_list[p_parent].y)<400:
                        path_x.append(self.node_list[p_parent].x)
                        path_y.append(self.node_list[p_parent].y)
                        parent=self.node_list[p_parent].parent
                        continue
                path_x.append(self.node_list[parent].x)
                path_y.append(self.node_list[parent].y)
                parent =self.node_list[parent].parent

        return path_x,path_y

    def check_obs(self,nearest_node,theta,obstree):
            a=1
            avoid_size=self.robot_size+self.avoid_dist
            step=round(self.expand_dis/(avoid_size))
            x=nearest_node.x
            y=nearest_node.y
            for i in range(step): 
                dis,index=obstree.query(np.array([x,y]))
                if dis<=avoid_size:
                    a=0
                    break
                x+=step*math.cos(theta)
                y+=step*math.sin(theta)
            dis,index=obstree.query(np.array([x,y]))
            if dis<=avoid_size:
                a=0
            return a
    def get_nearest_node_index(self,rand_Node ):
        dlist = [(node.x - rand_Node.x)**2 + (node.y - rand_Node.y)**2
                 for node in self.node_list]
        minind = dlist.index(min(dlist))

        return minind   


'''def check_obs(self,nearest_node,theta,obstree):
        a=1
        avoid_size=self.robot_size+self.avoid_dist
        step=round(self.expand_dis/(avoid_size))
        x=nearest_node.x
        y=nearest_node.y
        for i in range(step): 
            dis,index=obstree.query[np.array([x,y])]
            if dis<=avoid_size:
                a=0
                break
            x+=step*math.cos(theta)
            y+=step*math.sin(theta)'''