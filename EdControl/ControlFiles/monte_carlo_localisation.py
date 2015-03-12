import sys
from math import pi
from params import *
from utils import *   

def main():
  robot_util = RobotUtility() 
  
  draw = True
  interval = 10

  robot_util.walls.draw()
  robot_util.set_location(27.5, 15)
  robot_util.move_to_waypoint(15, 15, interval, draw)
  robot_util.move_to_waypoint(15, 33, interval, draw)
  robot_util.move_to_waypoint(50, 33, interval, draw)
  robot_util.move_to_waypoint(150, 33, interval, draw)


if __name__ == "__main__":
  main()

