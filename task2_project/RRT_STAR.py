from scipy.spatial import KDTree
import numpy as np
import random
import math
import time


class RRT_Star():
    '''RRT*算法得到从起始点到目标点的路径'''

    def __init__(self, STEP=1000):
        self.STEP = STEP
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 150
        self.avoid_dist = 300

    def plan(self, vision, start_x, start_y, goal_x, goal_y):
        '''路径规划，输入机器人起始位置与目标位置，输出路径与节点'''

        # 障碍物输入
        obstacle_x = list()
        obstacle_y = list()
        # time.sleep(0.05)
        for blue_robot in vision.blue_robot:
            if blue_robot.visible and blue_robot.id > 0:
                obstacle_x.append(blue_robot.x)
                obstacle_y.append(blue_robot.y)
        for yellow_robot in vision.yellow_robot:
            if yellow_robot.visible:
                obstacle_x.append(yellow_robot.x)
                obstacle_y.append(yellow_robot.y)
        # 创建障碍物KDTree
        obstacle_tree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        # 取点并生成树
        start_node, goal_node, node_x, node_y = self.GetTree(
            start_x, start_y, goal_x, goal_y, obstacle_tree)
        path_x, path_y = self.GetPath(start_node, goal_node, node_x, node_y)
        return path_x, path_y
    def GetTree(self, start_x, start_y, goal_x, goal_y, obstacle_tree):
      start_node = RRTNode(start_x, start_y, 0)
      goal_node = RRTNode(goal_x, goal_y, 999999)
      node_x = [start_x]
      node_y = [start_y]
      node_dict = dict()
      node_dict[(start_x, start_y)] = start_node
      node_dict[(goal_x,goal_y)] = goal_node
      node_tree = KDTree(np.vstack((node_x, node_y)).T) # 节点KD树
      while goal_node.parent_node == None:
        if len(node_x) >150:
          print('No path found!')
          break
        rand_x = (random.random() * (self.maxx - self.minx)) + self.minx
        rand_y = (random.random() * (self.maxy - self.miny)) + self.miny
        node_tree = KDTree(np.vstack((node_x, node_y)).T)
        distance, index = node_tree.query(np.array([rand_x, rand_y]))
        nearest_x = node_x[index]
        nearest_y = node_y[index]
        nearest_node = node_dict[(nearest_x, nearest_y)]
        angle = math.atan2(rand_y - nearest_y, rand_x - nearest_x)
        new_x = math.cos(angle) * self.STEP + nearest_x
        new_y = math.sin(angle) * self.STEP + nearest_y

        if len(node_x) > 1:
          # 终点检测
          distance, index = node_tree.query(np.array([goal_node.x, goal_node.y]), min(len(node_x), 5))
          for (dis, ind) in zip(distance, index):
            if dis <self.STEP and self.ObstacleFree(goal_node.x, goal_node.y, node_x[ind], node_y[ind], obstacle_tree):
              new_x = goal_x
              new_y = goal_y
              nearest_x = node_x[ind]
              nearest_y = node_y[ind]
              nearest_node = node_dict[(nearest_x, nearest_y)]
              break

        if self.ObstacleFree(new_x, new_y, nearest_x, nearest_y, obstacle_tree):
          distance, index = node_tree.query(np.array([new_x, new_y]), min([len(node_x), 4]))
          if type(distance) != np.ndarray: # 检测返回的是否只有一个数
            distance = [distance]
            index = [index]
          node_x.append(new_x)
          node_y.append(new_y)
          x_min = nearest_node
          c_min = nearest_node.cost + math.hypot(new_x - nearest_x, new_y - nearest_y)
          # 寻找最佳父节点
          for i in index:
            near_node = node_dict[(node_x[i], node_y[i])]
            if self.ObstacleFree(new_x, new_y, near_node.x, near_node.y, obstacle_tree) and near_node.cost + math.hypot(new_x - near_node.x, new_y - near_node.y) < c_min:
              x_min = near_node
              c_min = near_node.cost + math.hypot(new_x - near_node.x, new_y - near_node.y)
          if new_x == goal_x and new_y ==goal_y:
            new_node = goal_node
            new_node.parent_node = x_min
          else:
            new_node = RRTNode(new_x, new_y, c_min, parent_node=x_min)
          x_min.next_node.append(new_node)
          node_dict[(new_node.x, new_node.y)] = new_node
          # 更新路径
          for i in index:
            near_node = node_dict[(node_x[i], node_y[i])]
            if self.ObstacleFree(new_x, new_y, near_node.x, near_node.y, obstacle_tree) and new_node.cost + math.hypot(new_x - near_node.x, new_y - near_node.y) < near_node.cost:
              near_node.parent_node.next_node.remove(near_node)
              near_node.parent_node = new_node
              new_node.next_node.append(near_node)
              near_node.cost = new_node.cost + math.hypot(new_x - near_node.x, new_y - near_node.y)
      return start_node, goal_node, node_x, node_y
    def ObstacleFree(self, x1, y1, x2, y2, obstacle_tree):
      '''检测两点间是否有障碍物'''
      dx = x2 - x1
      dy = y2 - y1
      angle = math.atan2(dy, dx)
      dis = math.hypot(dx, dy)

      step_size = self.robot_size + self.avoid_dist
      steps = round(dis/step_size)
      for i in range(steps+1):
        distance, index = obstacle_tree.query(np.array([x1, y1]))
        if distance <= self.robot_size + self.avoid_dist:
          return False
        x1 += step_size * math.cos(angle)
        y1 += step_size * math.sin(angle)
      return True
    def GetPath(self, start_node, goal_node, node_x, node_y):
      # road_map = self.GetRoadMap(start_node, node_list)
      path_x = [goal_node.x]
      path_y = [goal_node.y]
      parent = goal_node.parent_node
      while parent != None:
        path_x.insert(0, parent.x)
        path_y.insert(0, parent.y)
        parent = parent.parent_node
      return path_x, path_y
    def GetRoadMap(self, start_node, node_list):
      road_map = []
      if start_node.next_node == []:
        return []
      else:
        for i in start_node.next_node:
          road_map += self.GetRoadMap(i, node_list)
          road_map += [[node_list.index((start_node.x, start_node.y)), node_list.index((i.x, i.y))]]
        return road_map


class RRTNode():
  '''RRT_STAR的节点'''
  def __init__(self, x, y, cost, parent_node=None):
    self.x = x
    self.y = y
    self.cost = cost
    self.next_node = []
    self.parent_node = parent_node
