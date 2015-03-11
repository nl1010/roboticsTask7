import sys
from params import *
from utils import * 
from math import pi  
import random  

import time 

def main():
  SQUARE_LOG_FILE_NAME = "square"
  
  robot_util = RobotUtility() 
  
  draw=False
  verbose=False

  robot_util.move_to_distance_from(20,draw,verbose)
  robot_util.rotate(-pi/2,draw,verbose)
  robot_util.rotate_head(-pi/2)
  robot_util.follow(100,20,draw,verbose)

if __name__ == "__main__":
  main()
