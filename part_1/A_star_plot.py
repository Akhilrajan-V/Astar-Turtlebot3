#!/usr/bin/env python

"""
ENPM661: Project 2

Akhilrajan Vethirajan (v.akhilrajan@gmail.com)
Vishaal Kanna Sivakumar (vishaal@terpmail.umd.edu)
M.Eng. Student, Robotics
University of Maryland, College Park

"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import time
import cv2
import math

c=10
def map_gen():
	map = np.zeros((250,400))

	for y in range(1,map.shape[0]+1):
		for x in range(1, map.shape[1]+1):
			if x>=165-c and x<=235+c and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(235+c/1.414)))*(x-(235+c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(165-c/1.414)))*(x-(165-c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((165-c/1.414)-(200)))*(x-(200))+map.shape[0]-(60-c)>=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((235+c/1.414)-200))*(x-200)+map.shape[0]-(60-c)>=y:
				map[y-1][x-1]=10000
			if ((map.shape[0]-(210+c/1.414)-map.shape[0]+(185))/((115+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185<=y and ((map.shape[0]-(100-c/1.414)-map.shape[0]+185)/((105+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185>=y and ((map.shape[0]-(210+c/1.414)-map.shape[0]+180)/((115+c/1.414)-(75+c))*(x-(75+c))+map.shape[0]-180>=y or ((map.shape[0]-180-map.shape[0]+(100-c/1.414))/((75+c)-(105+c/1.414)))*(x-(105+c/1.414))+map.shape[0]-(100-c/1.414)<=y):
				map[y-1][x-1]=10000
			if (x-300)**2+(y-map.shape[0]+185)**2<=(40+c)**2:
				map[y-1][x-1]=10000
			#if x>250 and x<275:
			#	map[y-1][x-1]=1
			if x>0 and x<=c:
				map[y - 1][x - 1] = 10000
			if x>400-c and x<=400:
				map[y - 1][x - 1] = 10000
			if y>0 and y<=c:
				map[y - 1][x - 1] = 10000
			if y>250-c and y<=250:
				map[y - 1][x - 1] = 10000
	return map


def rpm_to_xy(UL,UR,r,L,x,y,Theta,dt):
	UL = UL*2*3.14/60
	UR = UR*2*3.14/60
	Delta_Xn = 0.5 * r * (UL + UR) * math.cos(Theta*np.pi/180) * dt
	Delta_Yn = 0.5 * r * (UL + UR) * math.sin(Theta*np.pi/180) * dt
	Theta += (((r / L) * (UR - UL) * dt)*180/np.pi)%360
	if Theta < 0:
		Theta = 360 - Theta
	x = x + Delta_Xn
	y = y + Delta_Yn
	return x, y, (Theta%360)


def Move_dir(dup_map,map, action, state, index, Parent_cost,i):
	x,y,theta = state[0], state[1], state[2]
	r = 0.038
	L = 0.354
	dt = 1
	x_,y_,theta_ = rpm_to_xy(action[0], action[1], r, L, x, y, theta, dt)
	cost=0
	# if y_%1>=0.5:
	# 	y_=math.floor(y+action[1])+0.5
	# else:
	# 	y_ = math.floor(y + action[1])
	# if x_%1>=0.5:
	# 	x_=math.floor(x+action[0])+0.5
	# else:
	# 	x_ = math.floor(x + action[0])
	#
	# theta_ = ((theta+(i-2)*30)%360)%12
	#
	# y_ = int(2 * y_)
	# x_ = int(2 * x_)

	if x_>=c and x_<400-c and y_>=c and y_<250-c:
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
	# theta1 = theta_ * np.pi/180
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
		#print(x_1,y_1,theta_1)

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
				new_node = (((Goal_node_x-new_node_x)**2+(Goal_node_y-new_node_y)**2)**0.5+C2C, index, M[1],
				(new_node_x, new_node_y, new_node_angle), C2C, Action_space[i])
				# print(new_node)
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
		lst.append([ClosedList[idx][3], ClosedList[idx][5]])
		return lst
	else:
		lst.append([ClosedList[idx][3], ClosedList[idx][5]])
		generate_path(lst, ClosedList, ClosedList[idx][2])


def plot_curve(map,Xi, Yi, Thetai, UL, UR):
	t = 0
	r = 0.038
	L = 0.354
	dt = 1
	Xn = Xi
	Yn = Yi
	Thetan = 3.14 * Thetai / 180
	UL = UL*2*3.14/60
	UR = UR*2*3.14/60
	# Xi, Yi,Thetai: Input point's coordinates
	# Xs, Ys: Start point coordinates for plot function
	# Xn, Yn, Thetan: End point coordintes
	while t < 1:
		t = t + dt
		Xs = Xn
		Ys = Yn
		Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
		Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
		Thetan += (r / L) * (UR - UL) * dt
		cv2.line(map,(int(Xs),int(Ys)),(int(Xn),int(Yn)),[255, 255, 255], 1)
	cv2.imshow('A*',map)
	cv2.waitKey(1)
	return map


def main():
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
		rpm1 = int(input("Enter the RPM 1 :"))
		rpm2 = int(input("Enter the RPM 2 :"))
		if start_node_x < 0 or start_node_y < 0 or start_node_x > 399 or start_node_y > 249 or goal_node_x < 0 or goal_node_y < 0 or goal_node_x > 399 or goal_node_y > 249:
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
	start_node = (0, 0, -1, (start_node_x, start_node_y, start_angle), 0, (0, 0))
	OpenList = [start_node]
	ClosedList = []
	heapq.heapify(OpenList)
	goal_reached = 0
	dup_map = np.zeros((500,800,12))
	map_color_r = np.zeros((250, 400))
	map_color_b = np.zeros((250, 400))
	map_color_g = np.zeros((250, 400))
	map_color1 = np.zeros((250, 400,3))
	index = 0
	i=0
	exp = []
	while len(OpenList) and not goal_reached:
		dup_map, map, OpenList, ClosedList, index, goal_reached, exp = New_node(rpm1, rpm2, map, OpenList, ClosedList, goal_node_x, goal_node_y,index, dup_map, exp)
		if len(OpenList) == 0 and not goal_reached:
			print('Solution Not Found')
			quit()
		i+=1

	print('Goal Node Reached')

	lst = []
	lst.append([ClosedList[len(ClosedList) - 1][3], ClosedList[len(ClosedList) - 1][5]])
	generate_path(lst, ClosedList, ClosedList[len(ClosedList) - 1][2])

	print('Map explored and optimal path found.')

	map = map_gen()

	map_color = map
	map_color_r[map_color == 0] = 1
	map_color_b[map_color == 0.7] = 1
	map_color_g[map_color == 10000] = 1
	map_color1[:, :, 0] = map_color_r * 255
	map_color1[:, :, 1] = map_color_b * 255
	map_color1[:, :, 2] = map_color_g * 255

	for i in range(0,len(ClosedList)-1):
		map_color1 = plot_curve(map_color1, ClosedList[i][3][0], ClosedList[i][3][1], ClosedList[i][3][2], ClosedList[i+1][5][0], ClosedList[i+1][5][1])

	map = map_gen()

	map_color = map
	map_color_r[map_color == 0] = 1
	map_color_b[map_color == 0.7] = 1
	map_color_g[map_color == 10000] = 1
	map_color1[:, :, 0] = map_color_r * 255
	map_color1[:, :, 1] = map_color_b * 255
	map_color1[:, :, 2] = map_color_g * 255

	for i in range(0, len(lst)-1):
		cv2.line(map_color1, (int(lst[len(lst)-1-i][0][0]), int(lst[len(lst)-1-i][0][1])), (int(lst[len(lst)-i - 1][0][0]), int(lst[len(lst)-i-1][0][1])), [0, 255, 0], 2)
		cv2.imshow("A*", map_color1)
		cv2.waitKey(100)


if __name__ == '__main__':
	main()


fig, ax = plt.subplots()



