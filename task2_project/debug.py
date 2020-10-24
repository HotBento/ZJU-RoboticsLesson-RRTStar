import socket
import sys
import time

from zss_debug_pb2 import Debug_Msgs, Debug_Msg, Debug_Arc

class Debugger(object):
	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.debug_address = ('localhost', 20001)

	def draw_line(self, package, x1, y1, x2, y2):
		msg = package.msgs.add()
		msg.type = Debug_Msg.LINE
		msg.color = Debug_Msg.WHITE
		line = msg.line
		line.start.x = x1
		line.start.y = y1
		line.end.x = x2
		line.end.y = y2
		line.FORWARD = True
		line.BACK = True
	def draw_points(self, package, tx, ty):
		msg = package.msgs.add()
		msg.type = Debug_Msg.Points
		msg.color = Debug_Msg.BLUE
		for x,y in zip(tx,ty):
			points = msg.points.point.add()
			points.x = x
			points.y = y
	def draw_roadmap(self, package, sample_x, sample_y, road_map):
		for i in road_map:
			self.draw_line(package, sample_x[i[0]], sample_y[i[0]], sample_x[i[1]], sample_y[i[1]])
	def draw_finalpath(self, package, path_x, path_y):
		for i in range(len(path_x)-1):
			msg = package.msgs.add()
			msg.type = Debug_Msg.LINE
			msg.color = Debug_Msg.RED
			line = msg.line
			line.start.x = path_x[i]
			line.start.y = path_y[i]
			line.end.x = path_x[i+1]
			line.end.y = path_y[i+1]
			line.FORWARD = True
			line.BACK = True
		
	def draw_circle(self, package, x, y):
		msg = package.msgs.add()
		msg.type = Debug_Msg.ARC
		msg.color = Debug_Msg.WHITE
		arc = msg.arc
		radius = 300
		arc.rectangle.point1.x = x - radius
		arc.rectangle.point1.y = y - radius
		arc.rectangle.point2.x = x + radius
		arc.rectangle.point2.y = y + radius
		arc.start = 0
		arc.end = 360
		arc.FILL = True
	def draw_all(self, path_x, path_y):
		package = Debug_Msgs()
		# self.draw_points(package, sample_x, sample_y)
		# self.draw_roadmap(package, sample_x, sample_y, road_map)
		self.draw_finalpath(package, path_x, path_y)
		self.sock.sendto(package.SerializeToString(), self.debug_address)

if __name__ == '__main__':
	debugger = Debugger()
	while True:
		debugger.draw_circle(x=100, y=200)
		time.sleep(0.02)
