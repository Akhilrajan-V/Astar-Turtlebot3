#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import numpy as np
import heapq
import time
import math
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1
c = 20
msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

def map_gen():
    map = np.zeros((1000,1000))
    c=20
    for y in range(1,map.shape[0]+1):
        for x in range(1, map.shape[1]+1):
            if x>25-c and x<175+c and y<500+75+c and y>500-75-c:
                map[y-1][x-1]=1000
            if x > 500-125-c and x < 500+125+c and y < 500 + 75+c and y > 500 - 75-c:
                map[y - 1][x - 1] = 10000
            if x > 800-75-c and x < 800+75+c and y < 700 + 100 +c and y > 700 - 100-c:
                map[y - 1][x - 1] = 10000
            if (x-200)**2+(y-200)**2<=(100+c)**2:
                map[y-1][x-1]=10000
            if (x-200)**2+(y-800)**2<=(100+c)**2:
                map[y-1][x-1]=10000
    return map

def rpm_to_xy(UL,UR,r,L,x,y,Theta,dt):
    UL = UL*2*3.14/60
    UR = UR*2*3.14/60
    Delta_Xn = 0.5 * r * (UL + UR) * math.cos(Theta*np.pi/180) * dt
    Delta_Yn = 0.5 * r * (UL + UR) * math.sin(Theta*np.pi/180) * dt
    Theta += ((r / L) * (UR - UL) * dt)*180/np.pi
    if Theta<0:
        Theta = 360-Theta
    x = x + Delta_Xn
    y = y + Delta_Yn
    #print(x,y,Theta)
    return x, y, Theta

def Move_dir(dup_map,map, action, state, index, Parent_cost,i):
    x,y,theta = state[0], state[1], state[2]
    r = 3.8
    L = 35.4
    dt = 1
    x_,y_,theta_ = rpm_to_xy(action[0], action[1], r, L, x, y, theta, dt)
    cost=0

    if x_>=c and x_<1000-c and y_>=c and y_<1000-c:
        if map[math.floor(y_)][math.floor(x_)] == 0 and map[math.ceil(y_)][math.ceil(x_)] == 0: #and dup_map[math.floor(2*y_)][math.floor(2*x_)][math.floor(((theta_)%360)/30)]==0:
            #dup_map[y_][x_][theta_] = 1
            cost = Parent_cost+((x_-x)**2 + (y_-y)**2)**0.5
            index=index+1
            x = x_
            y = y_
            theta = theta_
            return dup_map,map, x, y, theta, cost, index, 1
    return dup_map,map, x, y, theta, cost, index, 0

def New_node(rpm1, rpm2, map, OpenList, ClosedList, Goal_node_x, Goal_node_y, index, dup_map, exp):
    M = heapq.heappop(OpenList)
    ClosedList.append(M)

    y_ = M[3][1]
    if y_ % 1 >= 0.5:
        y_ = math.floor(y_) + 0.5
    else:
        y_ = math.floor(y_)
    x_ = M[3][0]
    if x_ % 1 >= 0.5:
        x_ = math.floor(x_) + 0.5
    else:
        x_ = math.floor(x_)

    theta_ = int((M[3][2] % 360) / 30)
    theta1 = M[3][2]*np.pi/180 #theta1 is in radians

    y_ = int(2 * y_)
    x_ = int(2 * x_)

    if ((M[3][0]-Goal_node_x)**2+(M[3][1]-Goal_node_y)**2)**0.5 <=15:
        return dup_map, map, OpenList, ClosedList, index, 1, exp
    elif dup_map[y_][x_][theta_]==2:
        return dup_map, map, OpenList, ClosedList, index, 0, exp

    dup_map[y_][x_][theta_] = 2

    Action_space = ((0,rpm1), (rpm1,0), (0,rpm2), (rpm2,0), (rpm1,rpm1), (rpm1,rpm2), (rpm2,rpm1), (rpm2,rpm2))
    for i in range(0,8):
        dup_map, map, new_node_x, new_node_y, new_node_angle, C2C, index, t = Move_dir(dup_map, map, Action_space[i], M[3], index, M[4], i)
        y_1 = new_node_y
        if y_1 % 1 >= 0.5:
            y_1 = math.floor(y_1) + 0.5
        else:
            y_1 = math.floor(y_1)
        x_1 = new_node_x
        if x_1 % 1 >= 0.5:
            x_1 = math.floor(x_1) + 0.5
        else:
            x_1 = math.floor(x_1)

        theta_1 = int((new_node_angle % 360) / 30)
        # print(x_1,y_1,theta_1)

        y_1 = int(2 * y_1)
        x_1 = int(2 * x_1)
        count=0
        if t==1:
            # for p in range(y_1-1,y_1+1):
            # 	for q in range(x_1-1,x_1+1):
            # 		if dup_map[p][q][theta_1] == 2:
            # 			count=1
            # 			break
            if dup_map[y_1][x_1][theta_1] != 2:
                new_node = (((Goal_node_x-new_node_x)**2+(Goal_node_y-new_node_y)**2)**0.5+C2C, index, M[1], (new_node_x, new_node_y, new_node_angle), C2C, Action_space[i])
                #print(new_node)
                exp.append([[M[3][0], M[3][1]], [new_node[3][0], new_node[3][1]]])
                # print(exp)
                heapq.heappush(OpenList, new_node)

    return dup_map, map, OpenList, ClosedList, index, 0, exp


def generate_path(lst, ClosedList, idx):
    for i in range(0, len(ClosedList)):
        if ClosedList[i][1] == idx:
            idx = i
            break
    if ClosedList[idx][2] ==-1:
        lst.append(ClosedList[idx][5])
        return lst
    else:
        lst.append(ClosedList[idx][5])
        generate_path(lst, ClosedList, ClosedList[idx][2])

def astar():
    map = map_gen()
    flag=1

    while(flag):
        print('Enter the Starting Coordinates')
        start_node_x = int(input("Enter the X coordinate of start position :"))
        start_node_y = int(input("Enter the Y coordinate of start position :"))
        start_angle = int(input("Enter theta of start position :"))
        print('Enter the Goal Coordinates')
        goal_node_x = int(input("Enter the X coordinate of goal position :"))
        goal_node_y = int(input("Enter the Y coordinate of goal position :"))
        rpm1 = int(input("Enter RPM1 :"))
        rpm2 = int(input("Enter RPM2 :"))
        if start_node_x < c or start_node_y < c or start_node_x > 1000 or start_node_y > 1000 or goal_node_x < c or goal_node_y < c or goal_node_x > 1000 or goal_node_y > 1000:
            print("Coordinates entered are outside the map\n")
        elif start_node_x==goal_node_x and start_node_y==goal_node_y:
            print('Starting node and Goal node are same. Please enter different sets of matrices for each.\n')
        elif map[start_node_y][start_node_x] == 10000 or map[goal_node_y][goal_node_x] == 10000:
            print("Coordinates are inside the Obstacle\n")
        elif start_angle % 30 != 0:
            print("Start Theta value is wrong\n")
        else:
            flag = 0

    print('Starting Node: ', (start_node_x, start_node_y, start_angle))
    print('Goal Node: ', (goal_node_x, goal_node_y))

    # total cost, current index, parent index, (x, y, theta), cost to come
    start_node = (0, 0, -1, (start_node_x, start_node_y, start_angle), 0, (0,0))
    OpenList = [start_node]
    ClosedList = []
    heapq.heapify(OpenList)
    goal_reached = 0
    dup_map = np.zeros((2000,2000,12))

    index = 0
    i=0
    exp = []
    while len(OpenList) and not goal_reached:
        dup_map, map, OpenList, ClosedList, index, goal_reached, exp = New_node(rpm1, rpm2, map, OpenList, ClosedList, goal_node_x, goal_node_y,index, dup_map, exp)
        if len(OpenList) ==0 and not goal_reached:
            print('Solution Not Found')
            quit()
        i+=1

    print('Map explored')

    lst = []
    lst.append(ClosedList[len(ClosedList) - 1][5])
    generate_path(lst, ClosedList, ClosedList[len(ClosedList) - 1][2])

    print('Map explored and optimal path found.')

    return lst

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)


    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('path_plan1')
    rate = rospy.Rate(1)


    turtlebot3_model = rospy.get_param("model", "burger")
    control_linear_vel = 0.0
    control_angular_vel = 0.0
    path = astar()

    try:
        i=len(path)-1
        while not rospy.is_shutdown() and i>=1:

            # print(path[i][0],path[i][1])
            twist = Twist()

            r = 3.8
            L = 35.4
            Ul = path[i][0] * 2 * 3.14 / 60
            Ur = path[i][1] * 2 * 3.14 / 60
            Delta_Xn = 0.5 * r * (Ul + Ur)

            twist.linear.x = Delta_Xn/100; twist.linear.y = 0.0; twist.linear.z = 0.0

            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -(r / L) * (Ur - Ul)
            pub.publish(twist)

            i-=1
            if i==0:
                twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0

                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

            rate.sleep()
            

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)