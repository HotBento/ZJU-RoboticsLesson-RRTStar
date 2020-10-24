import time
import math
import numpy as np
from scipy.spatial import KDTree
from action import Action
from vision import Vision


class Run(object):
  def __init__(self, MAX_V=1500, MAX_A=1000, MAX_W=1, MAX_ALPHA=1, THRE=150, K_RHO=1, K_ALPHA=2):
    self.MAX_V = MAX_V
    self.MAX_A = MAX_A
    self.MAX_W = MAX_W
    self.MAX_ALPHA = MAX_ALPHA
    self.THRE = THRE
    self.K_RHO = K_RHO
    self.K_ALPHA = K_ALPHA
    self.vi = 0
    self.wi = 0
  def act(self, action, vision, path_x, path_y):
    for i in range(1, len(path_x)):
      self.PointToPoint(action, vision, path_x[i], path_y[i])
    # 在终点减速到0
    vi = self.vi
    wi = self.wi
    while vi > 0.00001 or abs(wi) > 0.00001:
      vi = self.vi
      wi = self.wi
      vf = 0
      wf = 0
      v_min = max(-self.MAX_V, vi-self.MAX_A/60)
      w_max = min(self.MAX_W, wi+self.MAX_ALPHA/60)
      w_min = max(-self.MAX_W, wi-self.MAX_ALPHA/60)
      if vf < v_min:
        vf = v_min
      if wf > w_max:
        wf = w_max
      elif wf < w_min:
        wf = w_min
      action.sendCommand(vf, 0, wf)
      time.sleep(0.02)
      self.vi = vf
      self.wi = wf
  def PointToPoint(self, action, vision, path_xf, path_yf):
    '''轨迹规划及简单的避障规划
    轨迹规划采用反馈控制法
    避障规划比较暴力，但在这种场景下挺实用'''
    while math.hypot(vision.my_robot.x-path_xf, vision.my_robot.y-path_yf)>self.THRE:
      xi = vision.my_robot.x
      yi = vision.my_robot.y
      wi = self.wi
      vi = self.vi
      theta = vision.my_robot.orientation

      dx = path_xf - xi
      dy = path_yf - yi
      beta = math.atan2(dy, dx)
      alpha = beta - theta
      while alpha > math.pi:
        alpha -= 2*math.pi
      while alpha < -math.pi:
        alpha += 2*math.pi
      rho = math.hypot(dx, dy)*np.sign(math.cos(alpha))
      v_max = min(self.MAX_V, vi+self.MAX_A/60)
      v_min = max(-self.MAX_V, vi-self.MAX_A/60)
      w_max = min(self.MAX_W, wi+self.MAX_ALPHA/60)
      w_min = max(-self.MAX_W, wi-self.MAX_ALPHA/60)
      if rho > 0:
        vf = self.K_RHO*rho + 300 # 在接近节点时以一个较小的速度前进
      else:
        vf = 0
      wf = self.K_ALPHA*alpha
      if vf > v_max:
        vf = v_max
      elif vf < v_min:
        vf = v_min
      if wf > w_max:
        wf = w_max
      elif wf < w_min:
        wf = w_min
      if -math.pi/2 < alpha < -math.pi/6 or math.pi/6 < alpha < math.pi/2:
        vf = min(vf, 150) # 防止机器人离轨道过远，限定其在alpha角过大时的速度
      
      # 障碍物检测，简单粗暴的避障规划，检测到距离较近的障碍物就刹车并转向
      # 同时遇到多个障碍物时有可能避障失败
      for blue_robot in vision.blue_robot:
        if blue_robot.visible and blue_robot.id > 0 and math.hypot(blue_robot.x-xi, blue_robot.y-yi)<400:
          dx_obstacle = blue_robot.x - xi
          dy_obstacle = blue_robot.y - yi
          beta_obstacle = math.atan2(dy_obstacle, dx_obstacle)
          alpha_obstacle = beta_obstacle - theta
          while alpha_obstacle > math.pi:
            alpha_obstacle -= 2*math.pi
          while alpha_obstacle < -math.pi:
            alpha_obstacle += 2*math.pi
          if -math.pi/2 < alpha_obstacle <0:
            wf = w_max
            vf = v_min
          elif 0 <= alpha_obstacle <math.pi/2:
            wf = w_min
            vf = v_min
          elif -math.pi <= alpha_obstacle <= -math.pi/2:
            wf = w_max
            vf = v_max
          elif math.pi/2 <= alpha_obstacle <= math.pi:
            wf = w_min
            vf = v_max
      for yellow_robot in vision.yellow_robot:
        if yellow_robot.visible and math.hypot(yellow_robot.x-xi, yellow_robot.y-yi)<400:
          dx_obstacle = yellow_robot.x - xi
          dy_obstacle = yellow_robot.y - yi
          beta_obstacle = math.atan2(dy_obstacle, dx_obstacle)
          alpha_obstacle = beta_obstacle - theta
          while alpha_obstacle > math.pi:
            alpha_obstacle -= 2*math.pi
          while alpha_obstacle < -math.pi:
            alpha_obstacle += 2*math.pi
          if -math.pi/2 < alpha_obstacle <0:
            wf = w_max
            vf = v_min
          elif 0 <= alpha_obstacle <math.pi/2:
            wf = w_min
            vf = v_min
          elif -math.pi <= alpha_obstacle <= -math.pi/2:
            wf = w_max
            vf = v_max
          elif math.pi/2 <= alpha_obstacle <= math.pi:
            wf = w_min
            vf = v_max
      
      action.sendCommand(vf, 0, wf)
      time.sleep(0.02)
      self.vi = vf
      self.wi = wf