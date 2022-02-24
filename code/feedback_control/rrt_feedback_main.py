import sys
sys.path.append("../")
from vision import Vision
from action import Action
from fdebug import Debugger
from RRT_feedback import RRT_star
from feedback import *
import time

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    planner = RRT_star()

    #while True:
        # 1. path planning & velocity planning
    
    goal_x, goal_y = -2400, -1500
    time.sleep(0.01)
    
    start_x, start_y = vision.my_robot.x, vision.my_robot.y
    path_x, path_y= planner.plan(vision=vision, start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)
    path_x.reverse()
    path_y.reverse()
    #print(len(path_x))
    # 2. send command
    x=[]
    y=[]
    for i in range(int(len(path_x)/3)):
        x.append(path_x[i*3])
        y.append(path_y[i*3])
    x.append(path_x[-1])
    y.append(path_y[-1])
    debugger.draw_finalpath(x, y)
    for i in range(5):
        velcontrol(x,y,vision,action)
    #debugger.draw_finalpath(final_x,final_y)
        action.sendCommand(vx=0, vy=0, vw=0)
        x.reverse()
        y.reverse()

    # 3. draw debug msg
    

    time.sleep(0.01)
