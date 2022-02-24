from scipy.spatial import KDTree
import numpy as np
import random
import math
import time
#调试
from debug import Debugger

class Node(object):
    def __init__(self, x, y, parent):
        self.x = x
        self.y = y
        self.parent = parent


class RRT(object):
    def __init__(self, MAX_EDGE_LEN=5000, MAX_TRY_COUNT=5000):
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.MAX_TRY_COUNT = MAX_TRY_COUNT # RRT最多尝试次数，超过则认为找不到
        self.minx = -4500 # 地图边界
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 200 # 机器人参数
        self.avoid_dist = 200
        self.StepLength = [800, 300, 100, 20] # 机器人执行能力

    def plan(self, vision, start_x, start_y, goal_x, goal_y,debugger):
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
        # Obstacle KD Tree
        obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        # init
        tree = [Node(start_x,start_y,-1)]
        edges = []
        node_x = [start_x]
        node_y = [start_y]
        flag = 0 # 0 means not found, 1 means found goal
        # 调试
        x1 = [];  y1 = [];  x2 = [];  y2 = []

        # search
        for i in range(self.MAX_TRY_COUNT):
            node_len = len(node_y)
            # get one random point
            sample_x, sample_y = self.sampling(obstree, node_x[node_len-1], node_y[node_len-1], goal_x, goal_y)
            cur_dis = math.hypot(node_x[node_len-1]-goal_x, node_y[node_len-1]-goal_y)
            if cur_dis < 200:
                length_index = 3
            elif cur_dis<500:
                length_index = 2
            elif cur_dis<1200:
                length_index = 1
            else:
                length_index = 0
            # update tree
            searchTree =  KDTree(np.vstack((node_x, node_y)).T)
            parent_index, new_x, new_y = self.search_NewNode(searchTree, sample_x, sample_y, node_x, node_y,length_index)
            # check collision
            if not self.check_obs(node_x[parent_index], node_y[parent_index], new_x, new_y, obstree):
                node_x.append(new_x)
                node_y.append(new_y)
                tree.append(Node(new_x, new_y, parent_index))

                # 调试（查看实时树）
                x1.append(node_x[parent_index])
                y1.append(node_y[parent_index])
                x2.append(new_x)
                y2.append(new_y)
                debugger.draw_debug(x1, y1, x2, y2)

                if math.hypot(new_x-goal_x, new_y-goal_y)<100:
                    flag = 1
                    break

        if flag == 0:
            path_x = []
            path_y = []
            print("Sadly, road plan has failed")
        else:
            print("Nice! Road plan has successed.")
            path_x, path_y = self.getPath(tree)

        return path_x, path_y, flag

    # 找从当前点到目标点方向的120范围内的点
    def sampling(self, obstree, cur_x, cur_y, goal_x, goal_y):
        while True:
            maxx=max(cur_x, goal_x)+200;  minx=min(cur_x, goal_x)-200
            maxy=max(cur_y,goal_y)+200;  miny=min(cur_y,goal_y)-200
            maxx=min(maxx, self.maxx);  minx=max(minx,self.minx)
            maxy=min(maxy,self.maxy);  miny=max(miny,self.miny)
            tx = (random.random() * (maxx - minx)) + minx
            ty = (random.random() * (maxy - miny)) + miny

            distance, index = obstree.query(np.array([tx, ty]))

            if distance >= self.robot_size + self.avoid_dist:
                if math.fabs(math.atan2(goal_y-cur_y, goal_x-cur_x)-math.atan2(goal_y-ty, goal_x-tx))<1:
                    sample_x = tx
                    sample_y = ty
                    break

        return sample_x, sample_y

    # 这里还需做轨迹规划（差分驱动）
    def search_NewNode(self, searchTree, sample_x, sample_y, node_x, node_y, length_index):
        distance, index = searchTree.query(np.array([sample_x, sample_y])) # find one nearest point
        nx = node_x[index]
        ny = node_y[index]
        angle =  math.atan2(ny-sample_y,nx-sample_x)
        new_x = nx + (self.StepLength[length_index])*math.cos(angle)
        new_y = ny + (self.StepLength[length_index])*math.sin(angle)
        return index, new_x, new_y
    
    # check collision
    def check_obs(self, ix, iy, nx, ny, obstree):
        x = ix
        y = iy
        dx = nx - ix
        dy = ny - iy
        angle = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        if dis > self.MAX_EDGE_LEN:#distance too big
            return True

        step_size = self.robot_size + self.avoid_dist
        steps = round(dis/step_size)#four out five in
        for i in range(steps):
            distance, index = obstree.query(np.array([x, y])) #the distance between nearest obstacle point and random point(start)
            if distance <= self.robot_size + self.avoid_dist:#there exists obstacle,can't go
                return True
            x += step_size * math.cos(angle)#go forward
            y += step_size * math.sin(angle)

        # check for goal point
        distance, index = obstree.query(np.array([nx, ny])) #the distance between nearest obstacle point and random point(end)
        if distance <= self.robot_size + self.avoid_dist:
            return True

        return False

    def getPath(self, tree):
        path_x = []
        path_y = []
        index=len(tree)-1
        while tree[index].parent != -1:
            path_x.append(tree[index].x)
            path_y.append(tree[index].y)
            index=tree[index].parent
        # add start point
        path_x.append(tree[index].x)
        path_y.append(tree[index].y)
        
        return path_x, path_y

