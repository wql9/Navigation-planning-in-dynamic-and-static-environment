import math
import time
def velcontrol(path_x,path_y,vision,action):
    robot=vision.my_robot
    tmax=1000
    pi=math.pi
    dt=0.01
    kr=3
    kb=1.5
    ka=8
    final_x=[]
    final_y=[]
    for i in range(len(path_x)-1):
        start_x,start_y=path_x[i],path_y[i]
        goal_x, goal_y = path_x[i+1], path_y[i+1]
        x=[start_x]
        y=[start_y]
        t=robot.orientation
        if t<0:
            t+=2*pi
        theta=[t]
        j=0
        while True:
            if j>=1000:
                break
            rho=math.hypot(x[j]-goal_x,y[j]-goal_y)
            beta=math.atan2(goal_y-y[j],goal_x-x[j])
            alpha=beta-theta[j]    
            if abs(alpha)>pi:
                alpha+=2*pi
            v=kr*rho
            w=ka*alpha+kb*beta
            #print(v,w)
            if v>1500:
                v=1500
            if math.hypot(x[j]-goal_x,y[j]-goal_y)<100.0:
                print("find track")
                break    
            if alpha>-math.pi/2 and alpha<=math.pi/2:
                pass
            else:
                v=-v
            #print(alpha)
            # if wd<0:
            #     wd+=2*math.pi       
            action.sendCommand(vx=v, vy=0, vw=w)
            time.sleep(dt)
            x.append(robot.x)#x[i]+dt*v*math.cos(theta[i]))
            y.append(robot.y)#y[i]+dt*v*math.sin(theta[i]))
            #print(x[i+1],y[i+1])
            j+=1
            t=robot.orientation
            if t<0:
                t+=2*math.pi
            theta.append(t)
        # action.sendCommand(vx=0, vy=0, vw=0)



    
    