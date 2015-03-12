

import sys
from params import *
from ed_utils import * 
from math import pi  
import random  

import time 

def main():
  
  robot = RobotUtility() 
  
  draw=False
  verbose=False
  #absolute head angle: look straight forward 
  ##must MANUALLY turn robot head forward 
  abs_head_angle = robot.get_abs_head_angle_ed()
  print "absolute head angle setted! ==> " + str(abs_head_angle)
  
  #rotate and get sensor values , store 
  #robot.set start position for sensing
  robot.turn_head_to_angle_ed(-pi,True)
  #robot.reset_head_ed(abs_head_angle,True)
  environment_buffer = robot.environment_scan(2*pi,abs_head_angle,True)

  #robot.turn_head_to_angle_ed(2*pi,True)
  robot.reset_head_ed(abs_head_angle,True)
  
  '''data should be saved'''
  print environment_buffer
  '''santitize by threashhold'''
  threashold_min = 57.5
  threashold_max = 62.5
  santitized_buffer = []
  for data in environment_buffer :
      angle = data[0]
      depth = data[1] 
      if (depth > threashold_min) and (depth < threashold_max) :
          santitized_buffer.append([angle,depth])
  print "santitized_buffer = " + str(santitized_buffer)

# #v2 
#   angle_arr = []
#   for data in santitized_buffer:
#       angle_arr.append(data[0])
#   angle_avg = sum(angle_arr) / len(angle_arr)
#   target_angle = - angle_avg
#   print target_angle
#   robot.rotate(target_angle)
# #v2 end


#v1
  depth_arr = []
  min_angle_arr = []
  for data in santitized_buffer:
      depth_arr.append(data[1])
  print "depth arr = " + str(depth_arr)
  if depth_arr :
   min_depth = min(depth_arr)
   print "min depth = " + str(min_depth)
   for data in santitized_buffer:
    if data[1] == min_depth:
       min_angle_arr.append(data[0])

   print "min_angle_arr = " + str(min_angle_arr)
   target_angle = -(min_angle_arr[0]+min_angle_arr[-1])/2
   c = 0
   target_angle = target_angle + c 
   print "target angle = " + str(target_angle)
   robot.rotate(target_angle)
#v1 end

   #identification process start
   robot.force_move(35) 
   robot.turn_head_to_angle_ed(pi/2) #turn right first ! 
   left_sonar_reading = robot.read_sensor_ed()
   print "left sonar: " + str(left_sonar_reading)
   robot.turn_head_to_angle_ed(-pi) #then turn full left
   right_sonar_reading = robot.read_sensor_ed()
   print "right sonar: " + str(right_sonar_reading)
   robot.reset_head_ed(abs_head_angle,True)
   
   #decide which location
   if left_sonar_reading > 40 and right_sonar_reading > 40 : 
       print "I AM IN POINT !!!! 2 <MIDDLE>!!!!"
   elif right_sonar_reading > left_sonar_reading :
    print "I AM IN POINT !!!! 1 <RIGHT>!!!!"
   elif left_sonar_reading > right_sonar_reading :
    print "I AM IN POINT !!!! 3 <LEFT>!!!!"







##store santitized angle data into array 
  # angle_arr = []
  # temp = 0
  # for data in santitized_buffer: 
  #     angle_arr.append(data[0])

##work out target angle by using the array 
  # if angle_arr:
  #   interval = abs(angle_arr[0]) + abs(angle_arr[-1])  
  #   print interval 
  #   minimal = min(angle_arr)
  #   if interval > minimal :
  #       target_angle = -
  #   target_angle = -(interval/2)
  #   print target_angle
  #   robot.rotate(target_angle)
   # if angle_arr:
   #     min_angle_arr = []
   #  interval = abs(angle_arr[0]) + abs(angle_arr[-1])
   #  target_angle = -(interval/2)

   #  temp_angle_arr = []
   #  for elem in angle_arr:
   #      temp 

   #  minimal = min(angle_arr[1])

   #  for elem in angle_arr:
   #      if elem = minimal:
   #          min_angle_arr.append(elem)
   #  target_angle = angle_arr[0] + angle_arr[-1]
   #  print target_angle
   #  robot.rotate(target_angle)
    
    





  #chop all data blow the threashhold 

  #get minimum data 





if __name__ == "__main__":
  main()












