from sys import path
from scipy.spatial import KDTree
import numpy as np
import math
import time
from debug import Debugger
from r_dwa import DWA

class Node(object):
    def __init__(self, x, y, parent_node):
        self.x = x
        self.y = y #position
        self.parent = parent_node 
        self.G = 0 
        self.H = 0 


class Astar_DWA(object):

    def __init__(self, MAX_EDGE_LEN=5000, LIMIT_TRIAL=500):

        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.LIMIT_TRIAL = LIMIT_TRIAL #trial limit 
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        #the shape of the robot
        self.robot_size = 200
        self.avoid_dist = 200
        self.r = 50
        self.obstree = ()
        self.step_lenth = 60 #the steplenth of robot
        self.openlist = []
        self.closelist = []
        self.dwaPlanner = DWA()
        self.pathDebugger = Debugger()
        self.pathPointInterval = 3 # A_star中取的路径点数量


    def plan(self, vision, start_x, start_y, start_angle, goal_x, goal_y, goal_angle):
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
        #init      
        flag = 0
        # Obstacle KD Tree
        # print(np.vstack((obstacle_x, obstacle_y)).T)
        self.obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        path = self.A_star(start_x, start_y, goal_x, goal_y)
        path_x = []
        path_y = []
        
        if path:
            flag =1
            path = np.array(path)       
            path_x = path[:,0]
            path_y = path[:,1]
            self.useDwa(path_x, path_y, start_angle, goal_angle, vision)

        return path_x, path_y, flag
    
    def useDwa(self, path_x, path_y, start_angle, goal_angle, vision):
        # get split path as new path
        path_len = len(path_x)
        new_path_x = [path_x[path_len-1]]
        new_path_y = [path_y[path_len-1]]
        if path_len>self.pathPointInterval:
            step = path_len//self.pathPointInterval
        else:
            step = 1
        for i in range(path_len-2, 0, -step):
            new_path_x.append(path_x[i])
            new_path_y.append(path_y[i])
        new_path_x.append(path_x[0])
        new_path_y.append(path_y[0])
        self.pathDebugger.draw_specialPoints(new_path_x,new_path_y)
        disReduis = 400
        # use dwa
        start_x = new_path_x[0];  start_y = new_path_y[0]
        for i in range(0, len(new_path_x)-1,1):
            if i==len(new_path_x)-2:
                disReduis = 100
            else:
                disReduis = 400
            self.dwaPlanner.plan(start_x,start_y,start_angle,new_path_x[i+1],new_path_y[i+1],goal_angle,vision,disReduis) # goal_angle no use
            start_x, start_y, start_angle = vision.my_robot.x,vision.my_robot.y,vision.my_robot.orientation
            


    def search_path(self, node, goal_x, goal_y):
        
        # if it has a collision, ignore
        if self.check_obs(node.x, node.y, self.obstree):
            return
        # if it's in closelist, ignore
        if self.isCloseList(node.x, node.y):
            return
        #calculate G，H
        node.G = node.parent.G + self.step_lenth
        node.H = np.abs(node.x-goal_x)+np.abs(node.y-goal_y)
        #relpace if its f < (minf of openlist)
        point = self.isOpenList(node.x, node.y)
        if point:
            if (node.G + node.H) <= (point.G + point.H):
                point = node
        else:
            self.openlist.append(node)

    #check if collision
    def check_obs(self, node_x, node_y, obstree):
        x = node_x
        y = node_y
        tree = obstree
        dis, index = tree.query(np.array([x, y]))
        if dis > self.MAX_EDGE_LEN:
            return True
        step_size = self.robot_size + self.avoid_dist
        steps = round(dis/step_size)
        for i in range(steps):
            if dis <= self.robot_size + self.avoid_dist:
                return True
        return False

    def A_star(self, start_x, start_y, goal_x, goal_y):
        # 当前节点
        # debugger = Debugger()
        start_position = Node(start_x, start_y,-1)
        self.openlist = [start_position]
        path_list = []

        # 调试
        x1 = [];  y1 = [];  x2 = [];  y2 = []
        for i in range(self.LIMIT_TRIAL):
            #if goal is in closelist
            #print(self.openlist)
            point = self.is_final(goal_x, goal_y)
            if point:
                path_list = [[point.x ,point.y]]
                while point.parent!= -1 :
                    point = point.parent
                    path_list.append([point.x, point.y])
                #reverse the list
                #path_list.reverse()
                print("The path is found!")
                return path_list
            if not self.openlist:
                return False
                
            #select the point with minF from the openlist
            point_minF = self.ifFmin()
             # 调试（查看实时树）
            parent = point_minF.parent
            if parent != -1 :
                x1.append(point_minF.x)
                y1.append(point_minF.y)
                x2.append(parent.x)
                y2.append(parent.y)

            #将minF移出poenlist和移入closelist
            self.closelist.append(point_minF)
            self.openlist.remove(point_minF)
            # Search Path,the order:←, ↓, →, ↑,且将该节点作为父节点
            node1 = Node(point_minF.x - self.step_lenth, point_minF.y, point_minF)
            node2 = Node(point_minF.x - self.step_lenth, point_minF.y - self.step_lenth, point_minF)
            node3 = Node(point_minF.x, point_minF.y - self.step_lenth, point_minF)
            node4 = Node(point_minF.x + self.step_lenth, point_minF.y - self.step_lenth, point_minF)            
            node5 = Node(point_minF.x + self.step_lenth, point_minF.y, point_minF)
            node6 = Node(point_minF.x + self.step_lenth, point_minF.y + self.step_lenth, point_minF)
            node7 = Node(point_minF.x, point_minF.y + self.step_lenth, point_minF)
            node8 = Node(point_minF.x - self.step_lenth, point_minF.y + self.step_lenth, point_minF)

            self.search_path(node1, goal_x, goal_y)
            self.search_path(node2, goal_x, goal_y)
            self.search_path(node3, goal_x, goal_y)
            self.search_path(node4, goal_x, goal_y) 
            self.search_path(node5, goal_x, goal_y)  
            self.search_path(node6, goal_x, goal_y)  
            self.search_path(node7, goal_x, goal_y)  
            self.search_path(node8, goal_x, goal_y)  

            
        
            #print(self.openlist)
            #x2.append(node1.x)
            #y2.append(node1.y)
            # 调试
            # debugger.draw_debug(x1, y1, x2, y2)
        if i == self.LIMIT_TRIAL-1:
            print("cannot find path!")

    #select the node with min f(n)
    def ifFmin(self):
        Fmin = self.openlist[0]
        for node in self.openlist:
            if (node.G + node.H) <= (Fmin.G+Fmin.H) :
                Fmin = node
        return Fmin
        
    def isOpenList(self,x,y):
        for node in self.openlist:
            if node.x == x and node.y == y:
                return node
        return False
       
    def is_final(self, goal_x, goal_y):
        for node in self.closelist:
            dx = node.x-goal_x
            dy = node.y-goal_y
            if math.hypot(dx, dy) <= self.r :
                return node   
        return False
    
    
    def isCloseList(self,goal_x,goal_y):
        for node in self.closelist:
            if node.x == goal_x and node.y == goal_y:
                return node   
        return False
    