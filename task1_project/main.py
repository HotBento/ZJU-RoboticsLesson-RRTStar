from vision import Vision
from action import Action
from debug import Debugger
import RRT_STAR
import time
import math

if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
	planner = RRT_STAR.RRT_Star()
	time.sleep(5)
	while True:
		start_x, start_y = vision.my_robot.x, vision.my_robot.y
		goal_x, goal_y = -2400, -1500
		sample_x, sample_y, road_map, path_x, path_y= planner.plan(vision, start_x, start_y, goal_x, goal_y)
		# 合并倾角相近的相邻路径，防止机器人在一段近乎直线的路径上反复加减速
		while True:
			flag = True
			for i in range(1,len(path_x)-1):
				if abs(math.atan2(path_y[i+1]-path_y[i], path_x[i+1]-path_x[i])-math.atan2(path_y[i]-path_y[i-1], path_x[i]-path_x[i-1]))<0.2:
					del path_x[i], path_y[i]
					flag = False
					break
			if flag:
				break
		# action.sendCommand(vx=100, vy=0, vw=0)
		# debugger.draw_circle(vision.my_robot.x, vision.my_robot.y)
		debugger.draw_all(sample_x, sample_y, road_map, path_x, path_y)
		# time.sleep(5)
