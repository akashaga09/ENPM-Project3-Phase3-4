import math
import sys
import matplotlib.pyplot as plt
import numpy as np


x_start = float(input("Enter x coordinate for start node:- "))
y_start = float(input("Enter y coordinate for start node:- "))
start_node=[x_start,y_start]
theta_i = float(input("Enter initial robot orientation:- "))

x_goal = float(input("Enter x coordinate for goal node:- "))
y_goal = float(input("Enter y coordinate for goal node:- "))
goal_node=[x_goal,y_goal]

clearance = float(input("Enter clearance value:- "))
rpm1 = int(input("Left Wheel RPM:- "))
rpm2 = int(input("Right Wheel RPM:- "))


radius = 0.08/2
length = 0.23
d = 0.16
margin = d + clearance


def hanging_line(point1, point2):
    a = (point2[1] - point1[1])/(np.cosh(point2[0]) - np.cosh(point1[0]))
    b = point1[1] - a*np.cosh(point1[0])
    x = np.linspace(point1[0], point2[0], 100)
    y = a*np.cosh(x) + b
    return (x,y)

def check_obstacle(node):
    x = node[0]
    y = node[1]
    c=0
    
    #square1
    l1 = y - 2.25 + margin
    l2 = x + 1.25 - margin
    l3 = y - 3.75 - margin
    l4 = x + 2.75 + margin

#square2
    l5 = y - 0.75 - margin
    l6 = y + 0.75 + margin
    l7 = x + 3.25 - margin
    l8 = x + 4.75 + margin

#square3
    l9 = y - 0.75 - margin
    l10 = y + 0.75 + margin
    l11 = x - 3.25 + margin
    l12 = x - 4.75 - margin

#circles
    c1 = (x ** 2) + (y ** 2) - (1 + margin) ** 2
    c2 = (x - 2.2) ** 2 + (y - 3) ** 2 - (1 + margin) ** 2
    c3 = (x - 2.2) ** 2 + (y + 3.2) ** 2 - (1 + margin) ** 2
    c4 = (x + 2) ** 2 + (y + 3.2) ** 2 - (1 + margin) ** 2
    
    #if margin > 0:
        #if (y - 5 + margin >= 0) or (x - 5 + margin >= 0) or (y + 5 - margin <= 0) or (x + 5 - margin <= 0):
         #   c = 1

    if (l1>=0 and l3<=0 and l2<=0 and l4>=0) or (l5<=0 and l6>=0 and l7<=0 and l8>=0) or (l9<=0 and l10>=0 and l11>=0 and l12<=0):
        c = 1
    if c1<=0 or c2<=0 or c3<=0 or c4<=0:
            #or c2<=0 or c3<=0 or c4<=0:
        c = 1
    return c

if (check_obstacle(goal_node)==1 or check_obstacle(start_node)==1):
    sys.exit("Either start node or goal node lies inside the obstacle space or outside the workspace")

def drange(start, stop, step):
    rn = start
    final = []
    while rn <= stop:
        rn = rn + step
        final.append(rn)
    return final

def create_map():
    x_obs = []
    y_obs = []
    for x in drange(-5,5,0.01):
        for y in drange(-5,5,0.01):
            if check_obstacle([x,y]) == 1:
                x_obs.append(x)
                y_obs.append(y)
    return x_obs, y_obs


def check_goal(node):
    c=0
    if (node[0]-goal_node[0])**2+(node[1]-goal_node[1])**2<(0.4)**2:
        c=1
    return c

def action_set(node,ul,ur,theta):
    new_node = [0,0]
    x = node[0]
    y = node[1]
    
    for i in range(0,400):
        d_theta = (radius/length)*(ur-ul)*0.005
        dx = (radius/2)*(ul+ur)*(math.cos(theta))*0.005
        dy = (radius/2)*(ul+ur)*(math.sin(theta))*0.005
        x += dx
        y += dy
        theta += d_theta
        
    new_node[0]=(new_node[0]+x)
    new_node[1]=(new_node[1]+y)
    new_node = [ ((math.floor(new_node[0] * 100)) / 100.0), ((math.floor(new_node[1] * 100)) / 100.0) ]
    return new_node,theta


def distance(point1 , point2):
    dist = math.sqrt((point1[0] - point2[0])**2 +  (point1[1] - point2[1])**2)
    return dist

def check_duplicate(node, points):
    for point in points:
        if (((node[0] - point[0])**2 + (node[1] - point[1])**2) - 0.1**2 < 0):
            return True
    return False


parent_node = [start_node]
child_node = [start_node]
theta = [theta_i]
cost_to_come = [distance(start_node,start_node)]
cost_to_go = [distance(goal_node,start_node)]
action_steps = [[0,0]]
v_parent_node = []
v_child_node = []
v_theta = []
v_cost_to_go = []
v_cost_to_come = []
final_action_steps = []
current_node = start_node
flag=0
i=0
while(flag!=1):

    new_node,new_theta = action_set(current_node, 0,rpm1,theta[i])
    if (check_obstacle(new_node)!=1):
        if check_duplicate(new_node, v_child_node)!= True:
            check=0
            for j in range(0,len(child_node)):
                if(new_node == child_node[j]):
                    check=1
                    if(cost_to_come[j]>=(cost_to_come[i]+distance( current_node , new_node ))):
                        parent_node[j]=current_node
                        cost_to_come[j] = cost_to_come[i]+distance( current_node , new_node )
                        cost_to_go[j] = cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node )
                        action_steps[j] = [0,rpm1]
                        theta[j] = new_theta
                        break
            if check!=1:
                parent_node.append(current_node)
                child_node.append(new_node)
                theta.append(new_theta)
                cost_to_come.append(cost_to_come[i] + distance( current_node , new_node ))
                cost_to_go.append(cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node ))
                action_steps.append([0,rpm1])
                
    new_node,new_theta=action_set(current_node, 0,rpm2,theta[i])
    if (check_obstacle(new_node)!=1):
        if check_duplicate(new_node, v_child_node)!= True:
            check=0
            for j in range(0,len(child_node)):
                if(new_node == child_node[j]):
                    check=1
                    if(cost_to_come[j]>=(cost_to_come[i]+distance( current_node , new_node ))):
                        parent_node[j]=current_node
                        cost_to_come[j] = cost_to_come[i]+distance( current_node , new_node )
                        cost_to_go[j] = cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node )
                        action_steps[j] = [0,rpm2]
                        theta[j] = new_theta
                        break
            if check!=1:
                parent_node.append(current_node)
                child_node.append(new_node)
                theta.append(new_theta)
                cost_to_come.append(cost_to_come[i] + distance( current_node , new_node ))
                cost_to_go.append(cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node ))
                action_steps.append([0,rpm2])
                
    new_node,new_theta=action_set(current_node, rpm1,rpm2,theta[i])
    if (check_obstacle(new_node)!=1):
        if check_duplicate(new_node, v_child_node)!= True:
            check=0
            for j in range(0,len(child_node)):
                if(new_node == child_node[j]):
                    check=1
                    if(cost_to_come[j]>=(cost_to_come[i]+distance( current_node , new_node ))):
                        parent_node[j]=current_node
                        cost_to_come[j] = cost_to_come[i]+distance( current_node , new_node )
                        cost_to_go[j] = cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node )
                        action_steps[j] = [rpm1,rpm2]
                        theta[j] = new_theta
                        break
            if check!=1:
                parent_node.append(current_node)
                child_node.append(new_node)
                theta.append(new_theta)
                cost_to_come.append(cost_to_come[i] + distance( current_node , new_node ))
                cost_to_go.append(cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node ))
                action_steps.append([rpm1,rpm2])
                
    new_node,new_theta=action_set(current_node, rpm1,rpm1,theta[i])
    if (check_obstacle(new_node)!=1):
        if check_duplicate(new_node, v_child_node)!= True:
            check=0
            for j in range(0,len(child_node)):
                if(new_node == child_node[j]):
                    check=1
                    if(cost_to_come[j]>=(cost_to_come[i]+distance( current_node , new_node ))):
                        parent_node[j]=current_node
                        cost_to_come[j] = cost_to_come[i]+distance( current_node , new_node )
                        cost_to_go[j] = cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node )
                        action_steps[j] = [rpm1,rpm1]
                        theta[j] = new_theta
                        break
            if check!=1:
                parent_node.append(current_node)
                child_node.append(new_node)
                theta.append(new_theta)
                cost_to_come.append(cost_to_come[i] + distance( current_node , new_node ))
                cost_to_go.append(cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node ))
                action_steps.append([rpm1,rpm1])
                
    new_node,new_theta=action_set(current_node, rpm2,rpm2,theta[i])
    if (check_obstacle(new_node)!=1):
        if check_duplicate(new_node, v_child_node)!= True:
            check=0
            for j in range(0,len(child_node)):
                if(new_node == child_node[j]):
                    check=1
                    if(cost_to_come[j]>=(cost_to_come[i]+distance( current_node , new_node ))):
                        parent_node[j]=current_node
                        cost_to_come[j] = cost_to_come[i]+distance( current_node , new_node )
                        cost_to_go[j] = cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node )
                        action_steps[j] = [rpm2,rpm2]
                        theta[j] = new_theta
                        break
            if check!=1:
                parent_node.append(current_node)
                child_node.append(new_node)
                theta.append(new_theta)
                cost_to_come.append(cost_to_come[i] + distance( current_node , new_node ))
                cost_to_go.append(cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node ))
                action_steps.append([rpm2,rpm2])
                
    new_node,new_theta=action_set(current_node, rpm2,rpm1,theta[i])
    if (check_obstacle(new_node)!=1):
        if check_duplicate(new_node, v_child_node)!= True:
            check=0
            for j in range(0,len(child_node)):
                if(new_node == child_node[j]):
                    check=1
                    if(cost_to_come[j]>=(cost_to_come[i]+distance( current_node , new_node ))):
                        parent_node[j]=current_node
                        cost_to_come[j] = cost_to_come[i]+distance( current_node , new_node )
                        cost_to_go[j] = cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node )
                        action_steps[j] = [rpm2,rpm1]
                        theta[j] = new_theta
                        break
            if check!=1:
                parent_node.append(current_node)
                child_node.append(new_node)
                theta.append(new_theta)
                cost_to_come.append(cost_to_come[i] + distance( current_node , new_node ))
                cost_to_go.append(cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node ))
                action_steps.append([rpm2,rpm1])
                
    new_node,new_theta=action_set(current_node, rpm2,0,theta[i])
    if (check_obstacle(new_node)!=1):
        if check_duplicate(new_node, v_child_node)!= True:
            check=0
            for j in range(0,len(child_node)):
                if(new_node == child_node[j]):
                    check=1
                    if(cost_to_come[j]>=(cost_to_come[i]+distance( current_node , new_node ))):
                        parent_node[j]=current_node
                        cost_to_come[j] = cost_to_come[i]+distance( current_node , new_node )
                        cost_to_go[j] = cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node )
                        action_steps[j] = [rpm2,0]
                        theta[j] = new_theta
                        break
            if check!=1:
                parent_node.append(current_node)
                child_node.append(new_node)
                theta.append(new_theta)
                cost_to_come.append(cost_to_come[i] + distance( current_node , new_node ))
                cost_to_go.append(cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node ))
                action_steps.append([rpm2,0]) 
                
    new_node,new_theta=action_set(current_node, rpm1,0,theta[i])
    if (check_obstacle(new_node)!=1):
        if check_duplicate(new_node, v_child_node)!= True:
            check=0
            for j in range(0,len(child_node)):
                if(new_node == child_node[j]):
                    check=1
                    if(cost_to_come[j]>=(cost_to_come[i]+distance( current_node , new_node ))):
                        parent_node[j]=current_node
                        cost_to_come[j] = cost_to_come[i]+distance( current_node , new_node )
                        cost_to_go[j] = cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node )
                        action_steps[j] = [rpm1,0]
                        theta[j] = new_theta
                        break
            if check!=1:
                parent_node.append(current_node)
                child_node.append(new_node)
                theta.append(new_theta)
                cost_to_come.append(cost_to_come[i] + distance( current_node , new_node ))
                cost_to_go.append(cost_to_come[i] + distance( current_node , new_node ) + distance( goal_node , new_node ))
                action_steps.append([rpm1,0]) 
                
    v_parent_node.append(parent_node.pop(i))
    v_child_node.append(child_node.pop(i))
    v_theta.append(theta.pop(i))
    v_cost_to_go.append(cost_to_go.pop(i))
    final_action_steps.append(action_steps.pop(i))
    v_cost_to_come.append(cost_to_come.pop(i))

    if(flag==0 and child_node==[]):         
        sys.exit("Path not found")
        
    if(check_goal(v_child_node[-1])==1):
        flag=1
        
    if(flag!=1):
        i=cost_to_go.index(min(cost_to_go))
        current_node=child_node[i][:]


action_sequence=[final_action_steps[-1]]
x=v_parent_node[-1]
i=1
while(x!= start_node):
    if(v_child_node[-i]==x):
        action_sequence.append(final_action_steps[-i])
        x=v_parent_node[-i]
    i=i+1     

path=[]
path.append(v_child_node[-1])
path.append(v_parent_node[-1])
x=v_parent_node[-1]
i=1
while(x!=start_node):
    if(v_child_node[-i]==x):
        path.append(v_parent_node[-i])
        x=v_parent_node[-i]
    i=i+1

'''x_obs, y_obs = create_map()
plt.plot(x_obs,y_obs,".k")

for i in v_child_node:
    plt.axis([-5,5,-5,5])
    plt.plot(i[0],i[1],'.b')
    plt.pause(0.0000000000000000000000000005)
        
    
x_path = [i[0] for i in path]
y_path = [i[1] for i in path]
plt.plot(x_path,y_path,"r")

plt.show()'''


try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    time = 0

    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
    r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_streaming)

    path_speeds= action_sequence[::-1]

    for k in path_speeds:
        time = 0
        err_code1 = 1
        err_code2 = 2
        while(err_code1 != 0 and err_code2 != 0):
            err_code1 = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], vrep.simx_opmode_streaming)

            err_code2 = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], vrep.simx_opmode_streaming)

        r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

        while(time<2.4):

            r, signalValue2 = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

            time = signalValue2 - signalValue

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')