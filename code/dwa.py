"""parameters
最大线速度3m/s 最大线加速度3m/s^2
最大角速度5rad/s 最大角加速度5rad/s^2
"""
import math
from action import Action
import time
import sys
#调试
from debug import Debugger

class Robot(object):
    def __init__(self):
        self.robot_size = 150 # 机器人参数 200
        self.avoid_dist = 150
        self.StepLength = 200 # 机器人执行能力
        self.TimeInterval = 0.15 # 机器人控制时间间隔
        self.PredictTime = 0.45 # 选取(v,w)时机器人路径的预测时间
        self.maxVel = 1200.0 # 机器人最大线速度 3000  1200
        self.maxAddVel = 3000.0 # 机器人最大线加速度 3000
        self.maxW = 3.0 # 机器人最大角速度 5.0   3.0
        self.maxAddW = 5.0 # 机器人最大角加速度 5.0
        self.ToGoalCostDis = 60 # 评价函数中朝向目标点的权重
        self.ToGoalCostAngle = 120
        self.ToGoalCostV = 0 # here needs to be changed
        self.ToGoalCostW = 0 # here needs to be changed
        self.ToGoalCostFinalAngle = 0 # here needs to be changed
        self.AwayfromObstacle = 1000 # 评价函数中远离障碍物的权重 1000
        self.MaxVelW = 0.001 # 评价函数中速度最大化的权重
        self.RobotStuckLimitV = 10 # 判断机器人是否被卡住
        self.RobotStuckLimitW = 0.8
        self.VelAddInterval = 10 # no use
        self.WAddInerval = 0.001 # no use
        self.MaxDis = 5000 # 评价函数计算中用来归一化 no use

class DWA(object):
    def __init__(self, MAX_EDGE_LEN=5000, MAX_TRY_COUNT=5000):
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.MAX_TRY_COUNT = MAX_TRY_COUNT # DWA最多尝试次数，超过则认为找不到
        self.minx = -4500 # 地图边界
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.action = Action()
        self.debugger = Debugger()
        self.robotPara = Robot()

    def floatrange(self,start,stop,steps):
        ans = [start]
        while ans[-1]<stop:
            ans.append(ans[-1]+steps)
        return ans

    def plan(self, start_x, start_y, start_angel, goal_x, goal_y, goal_angle, vision):
        #initialize: cur_x, cur_y, cur_angle, cur_vx, cur_vw
        cur = [start_x, start_y, start_angel, 0, 0]
        flag = 0
        path_x = [start_x]
        path_y = [start_y]
        time_start = time.time()
        #print("start_x: "+str(start_x)+" start_y: "+str(start_y))
        for i in range(1, self.MAX_TRY_COUNT):
            new_v, new_w = self.stepPlan(cur, goal_x, goal_y, goal_angle, vision)
            # prevent the robot to be stuck
            if abs(new_v)<self.robotPara.RobotStuckLimitV and abs(new_w)<self.robotPara.RobotStuckLimitW:
                new_w = self.robotPara.maxW
                new_v = -self.robotPara.maxVel
            time_end = time.time()
            time_interval = time_end-time_start
            if time_interval<self.robotPara.TimeInterval:
                time.sleep(self.robotPara.TimeInterval-time_interval)
            # send command
            if cur[0]<-4500.0 or cur[0]>4500.0 or cur[1]<-3000.0 or cur[1]>3000.0:
                self.action.sendCommand(vx=0, vy=0, vw=0)
            else:
                self.action.sendCommand(vx=new_v, vy=0, vw=new_w) # 由于差分驱动，所以vy=0
            # print("vx: "+str(new_v)+" vw: "+str(new_w))

            # termination condition
            # x,y不太准，因为前面公式好像就不完全准，所以用vision重新读到的x,y,angle代替
            cur_x, cur_y, cur_angle = vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation
            if cur_angle<0:
                cur_angle += 2*math.pi
            distance = math.hypot(cur_x-goal_x, cur_y-goal_y)
            #print("i= "+str(i)+" cur_pos: "+ str(cur_x)+" , "+str(cur_y)+" , "+str(cur_angle)+" distace: "+str(distance))
            if distance<100:
                flag = 1
                self.action.sendCommand(vx=0, vy=0, vw=0)
                break
            cur = [cur_x,cur_y,cur_angle,new_v,new_w]
        
        if flag==1:
            print("Nice! We have got the goal.")
        else:
            print("Sadly, we fail to get the goal.")
            
    def stepPlan(self, cur, goal_x, goal_y, goal_angle, vision):
        MinV, MaxV, MinW, MaxW = self.getVelWSpace(cur)
        minEvaluation = float("inf")
        bestChoose = [cur[3],cur[4]]
        #print("MinV: "+str(MinV)+" MaxV: "+str(MaxV)+" MinW: "+str(MinW)+ " MaxW: "+str(MaxW))
        Interval_v = (MaxV-MinV)/10
        Interval_w = (MaxW-MinW)/10
        for v in self.floatrange(MinV, MaxV, Interval_v):
            for w in self.floatrange(MinW, MaxW, Interval_w):
                # get predict trajectory
                trajectory = self.predictTrajectory(cur, v, w)
                # get obstacles
                obstacle_x, obstacle_y = self.getCurrentObstacles(vision)
                # get cost
                dif_angle, dif_distance,dif_v, dif_w, dif_finalAngle = self.EGoalHeading(trajectory[-1], goal_x, goal_y, goal_angle)
                #print("dif_distance:" + str(dif_distance))
                ToGoalEvaluation = self.robotPara.ToGoalCostDis*dif_distance+self.robotPara.ToGoalCostAngle*dif_angle+\
                                self.robotPara.ToGoalCostV*dif_v+self.robotPara.ToGoalCostW*dif_w+self.robotPara.ToGoalCostFinalAngle*dif_finalAngle
                AwayObstacleEvaluation = self.robotPara.AwayfromObstacle*self.EObstacleDist(trajectory,obstacle_x,obstacle_y)
                MaxVelEvaluation = self.robotPara.MaxVelW*(self.robotPara.maxVel-trajectory[-1][3])/self.robotPara.maxVel
                # print("v: "+str(v)+" w: "+str(w)+" Goal: "+str(ToGoalEvaluation)+" Obs: "+str(AwayObstacleEvaluation)+" Vel: "+str(MaxVelEvaluation))
                SumEvaluation = ToGoalEvaluation+AwayObstacleEvaluation+MaxVelEvaluation

                if minEvaluation>SumEvaluation:
                    minEvaluation = SumEvaluation
                    bestChoose = [v,w]
        # prevent the robot to be stuck
        if minEvaluation == float('inf'):
            bestChoose[0] = -self.robotPara.maxVel
        #print("v: "+str(bestChoose[0])+" w: "+str(bestChoose[1]))
        return bestChoose[0], bestChoose[1]



    def EGoalHeading(self, trajectory, goal_x, goal_y, goal_angle):
        x = trajectory[0]
        y = trajectory[1]
        angle = trajectory[2]
        v = trajectory[3]
        w = trajectory[4]
        error_angle = math.atan2(goal_y-y, goal_x-x)
        if error_angle<0:
            error_angle += 2*math.pi
        differ_angle = abs(error_angle-angle)
        # differ_distance = math.hypot(x-goal_x, y-goal_y)/math.hypot(self.minx-self.maxx,self.miny-self.maxy)
        differ_distance = math.hypot(x-goal_x, y-goal_y)
        
        differ_v = (1/differ_distance)*abs(v)
        differ_w = (1/differ_distance)*abs(w)
        if math.hypot(x-goal_x, y-goal_y)<300:
            differ_finalAngle = abs(angle-goal_angle)
        else:
            differ_finalAngle = 0
        #print("curX: "+str(x)+" curY: "+str(y)+" goal_x: "+str(goal_x)+" goal_y: "+str(goal_y))
        #print("differ_distance:" + str(differ_distance))
        return differ_angle, differ_distance, differ_v, differ_w, differ_finalAngle

    def EObstacleDist(self, trajectory, obstacle_x, obstacle_y):
        minDis = float("inf")
        for i in range(0, len(trajectory)):
            x=trajectory[i][0];  y=trajectory[i][1]
            for j in range(0, len(obstacle_x)):
                dis = math.hypot(x-obstacle_x[j], y-obstacle_y[j])
                if dis<self.robotPara.robot_size+self.robotPara.avoid_dist:
                    return float("inf")
                if (x<self.minx or x>self.maxx) and (y>self.maxy or y<self.miny):
                    return float('inf')
                minDis = min(minDis, dis)
        return 1.0/minDis*(math.hypot(self.minx-self.maxx,self.miny-self.maxy))

    def predictTrajectory(self, cur, v, w):
        trajectory = [cur]
        for t in self.floatrange(self.robotPara.TimeInterval, self.robotPara.PredictTime, self.robotPara.TimeInterval):
            cur = self.robotMoveModel(cur, v, w, self.robotPara.TimeInterval)
            if cur[0]<-4500.0 or cur[0]>4500.0 or cur[1]<-3000.0 or cur[1]>3000.0:
                break
            #print("cur[0]: "+str(cur[0])+" cur[1]: "+str(cur[1]))
            trajectory.append(cur)
        return trajectory

    def getCurrentObstacles(self, vision):
        obstacle_x = []
        obstacle_y = []
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacle_x.append(robot_blue.x)
                obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacle_x.append(robot_yellow.x)
                obstacle_y.append(robot_yellow.y)
        return obstacle_x, obstacle_y

    def robotMoveModel(self, cur, v, w, t):
        new_angle = cur[2] + w*t
        while new_angle>=2*math.pi:
            new_angle -= 2*math.pi
        while new_angle<=0:
            new_angle += 2*math.pi
            
        new_x = cur[0] + v*math.cos(new_angle)*t
        new_y = cur[1] + v*math.sin(new_angle)*t
        return [new_x, new_y, new_angle, v, w]

    def getVelWSpace(self, cur):
        v_left = cur[3] - self.robotPara.TimeInterval*self.robotPara.maxAddVel
        v_right = cur[3] + self.robotPara.TimeInterval*self.robotPara.maxAddVel
        w_left = cur[4] - self.robotPara.TimeInterval*self.robotPara.maxAddW
        w_right = cur[4] + self.robotPara.TimeInterval*self.robotPara.maxAddW
        Vd = [v_left, v_right, w_left, w_right]
        Vs = [-self.robotPara.maxVel, self.robotPara.maxVel, -self.robotPara.maxW, self.robotPara.maxW]
        VSpace = [max(Vd[0], Vs[0]), min(Vd[1], Vs[1]), max(Vd[2], Vs[2]), min(Vd[3],Vs[3])]
        return VSpace[0], VSpace[1], VSpace[2], VSpace[3]




    
