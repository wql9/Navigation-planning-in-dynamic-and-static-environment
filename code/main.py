from vision import Vision
from action import Action
from debug import Debugger
from prm import PRM
from rrt import RRT
from dwa import DWA
from rrt_dwa import RRT_DWA
from A_star_dwa import Astar_DWA
from action import Action
import time
import math
import sys

def getAngle(x1, y1, x2, y2):
    angle = math.atan2(y1-y2, x1-x2)
    # transform from -pi-pi to 0-2pi
    if angle<0:
        angle += 2*math.pi
    return angle


if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()

    time.sleep(0.1) # 否则是-999999
    start_x, start_y, start_angle= vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation
    #print("start_x: "+str(start_x)+" start_y: "+str(start_y))
    #sys.exit(0)
    goal_x, goal_y = -2400, -1500
    # draw the start and end points
    # debugger.draw_specialPoints([start_x,goal_x],[start_y,goal_y])
    goal_angle = getAngle(start_x, start_y, goal_x, goal_y)
    
    # try A_star+dwa
    planner = Astar_DWA()
    for i in range(1,6):
        path_x, path_y, flag= planner.plan(vision=vision, start_x=start_x, start_y=start_y, start_angle=start_angle,
                                goal_x=goal_x, goal_y=goal_y, goal_angle=goal_angle)
        # turn back
        MaxW = 5
        action.sendCommand(vx=0, vy=0, vw=MaxW)
        while True:
            cur_angle = vision.my_robot.orientation
            if cur_angle<0:
                cur_angle += 2*math.pi
            if abs(cur_angle - goal_angle)<math.pi/4:
                break
            time.sleep(math.pi/MaxW/6)
        action.sendCommand(vx=0, vy=0, vw=0)
        goal_x = start_x;  goal_y = start_y
        start_x, start_y, start_angle = vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation
        goal_angle = getAngle(start_x, start_y, goal_x, goal_y)

    """ # try rrt+dwa
    planner = RRT_DWA()
    for i in range(1,6):
        path_x, path_y = planner.plan(start_x, start_y, start_angle, goal_x, goal_y, goal_angle, vision)
        # turn back
        MaxW = 5
        action.sendCommand(vx=0, vy=0, vw=MaxW)
        while True:
            cur_angle = vision.my_robot.orientation
            if cur_angle<0:
                cur_angle += 2*math.pi
            if abs(cur_angle - goal_angle)<math.pi/4:
                break
            time.sleep(math.pi/MaxW/6)
        action.sendCommand(vx=0, vy=0, vw=0)
        goal_x = start_x;  goal_y = start_y
        start_x, start_y, start_angle = vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation
        goal_angle = getAngle(start_x, start_y, goal_x, goal_y) """

    """ # use dwa directly
    planner = DWA()
    debugger.draw_specialPoints([start_x,goal_x],[start_y,goal_y])
    for i in range(1,6):
        planner.plan(start_x, start_y, start_angle, goal_x, goal_y,goal_angle, vision)
        # turn back
        MaxW = 5
        action.sendCommand(vx=0, vy=0, vw=MaxW)
        while True:
            cur_angle = vision.my_robot.orientation
            if cur_angle<0:
                cur_angle += 2*math.pi
            if abs(cur_angle - goal_angle)<math.pi/4:
                break
            time.sleep(math.pi/MaxW/6)
        action.sendCommand(vx=0, vy=0, vw=0)
        goal_x = start_x;  goal_y = start_y
        start_x, start_y, start_angle = vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation
        goal_angle = getAngle(start_x, start_y, goal_x, goal_y) """
    
        
    """ #prm
    planner = PRM()
    path_x, path_y, road_map, sample_x, sample_y = planner.plan(vision=vision, 
        start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y) 
    debugger.draw_all(sample_x, sample_y, road_map, path_x, path_y)"""

    """#rrt
    planner = RRT()
    debugger1 = Debugger()
    debugger2 = Debugger()
    path_x, path_y, road_flag = road_planner.plan(vision=vision, start_x=start_x, start_y=start_y, 
        goal_x=goal_x, goal_y=goal_y,debugger=debugger2)
    if road_flag == 0:
        sys.exit(0)
    #draw path
    debugger1.draw_onlyPath(path_x, path_y)"""

