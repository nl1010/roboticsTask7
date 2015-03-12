import brickpi
import math
import params
import random
import sys
import place_rec_bits
import time

#Provided by ajd + modified START

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
  def __init__(self,map_size=210):
    self.map_size    = map_size    # in cm;
    self.canvas_size = 768         # in pixels;
    self.margin      = 0.05*map_size
    self.scale       = self.canvas_size/(map_size+2*self.margin)

  def drawLine(self,line):
    x1 = self.__screenX(line[0])
    y1 = self.__screenY(line[1])
    x2 = self.__screenX(line[2])
    y2 = self.__screenY(line[3])
    print "drawLine:" + str((x1,y1,x2,y2))

  def drawParticles(self,data):
    display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data]
    print "drawParticles:" + str(display)

  def __screenX(self,x):
    return (x + self.margin)*self.scale

  def __screenY(self,y):
    return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
  def __init__(self, practical=7):
    if (practical == 7):
      self.set_walls_practical7()
    elif (practical == 5):
      self.set_walls_practical5()
    else:
      self.set_walls_debug()

  def set_walls_debug(self):
    self.a = (0, 0, 0, 48)
    self.b = (0, 48, 171, 48)
    self.c = (171, 48, 171, 0)
    self.d = (0, 0, 171, 0)
    self.walls = [self.a, self.b, self.c, self.d]

  def set_walls_practical5(self): 
    self.a = (0,0,0,168)        # a
    self.b = (0,168,84,168)     # b
    self.c = (84,126,84,210)    # c
    self.d = (84,210,168,210)   # d
    self.e = (168,210,168,84)   # e
    self.f = (168,84,210,84)    # f
    self.g = (210,84,210,0)     # g
    self.h = (210,0,0,0)        # h

    self.walls = [self.a, self.b, self.c, self.d, self.e, self.f, self.g, self.h]

  def set_walls_practical7(self): 
    self.a = (0,0,0,84)        # OA
    self.b = (0,84,42,84)      # AB
    self.c = (42,84,42,42)     # BC
    self.d = (42,42,252,42)    # CD
    self.e = (252,42,252,84)   # DE
    self.f = (252,84,294,84)   # EF
    self.g = (294,84,294,42)   # FG 
    self.h = (294,42,504,42)   # GH
    self.i = (504,42,504,84)   # HI
    self.j = (504,84,546,84)   # IJ
    self.k = (546,84,546,0)    # JK
    self.o = (546,0,0,0)       # KO

    self.walls = [self.a, self.b, self.c, self.d, self.e, self.f, self.g, self.h, self.i, self.j, self.k, self.o]

  def add_wall(self,wall):
    self.walls.append(wall)

  def clear(self):
    self.walls = []

  def draw(self):
    for wall in self.walls:
     s_wall = tuple(params.DRAW_SCALE * i for i in wall)
     print "drawLine:" + str(s_wall) 
      
  def which_wall(self, particle, verbose=False):
    # Given a particle, find out which wall it is looking at.
    minimum_m = sys.maxint
    best_wall = None
    incident_angle = 0.0
    for wall in self.walls:
      A_x = wall[0]
      A_y = wall[1]
      B_x = wall[2]
      B_y = wall[3]

      if verbose:
        print("Wall A: ({},\t {})\t - \tWall B: ({},\t {}).".format(A_x, A_y, B_x, B_y))
      
      denom = ( (B_y - A_y) * math.cos(particle.theta) - (B_x - A_x) * math.sin(particle.theta) )
      
      if denom == 0:
        print "division by zero avoided!"
        continue
        
      m = ( (B_y - A_y) * (A_x - particle.x) - (B_x - A_x) * (A_y - particle.y) ) / denom

      if verbose:
        print("M for this wall: {}".format(m))

      x_max = max(A_x, B_x)
      x_min = min(A_x, B_x)
      y_max = max(A_y, B_y)
      y_min = min(A_y, B_y)

      hit_point = (particle.x + m * math.cos(particle.theta), particle.y + m * math.sin(particle.theta))

      if verbose:
        print(hit_point)

      if hit_point[0] >= x_min - 1 and hit_point[0] <= x_max + 1 and hit_point[1] >= y_min - 1 and hit_point[1] <= y_max + 1:
        if m < minimum_m and m >= 0:
          minimum_m = m
          best_wall = wall
          incident_angle = math.acos(( math.cos(particle.theta) * (A_y - B_y) + math.sin(particle.theta) * (B_x - A_x) ) / (math.sqrt((A_y - B_y) ** 2 + (B_x - A_x) ** 2)) )
      
    if best_wall == None:
      print incident_angle
      print minimum_m
      print hit_point

    return best_wall, minimum_m, incident_angle, hit_point

  # Provided by ajd + modified END
 
class Particle:
  num_particles = 100
  def __init__(self, x=0, y=0, theta=0):
    self.x = x
    self.y = y
    self.theta = theta
    self.weight = 1.0 / Particle.num_particles

class RobotUtility:
  def __init__(self):
    interface = brickpi.Interface()
    interface.initialize()
    self.ACCEPT_THRESHOLD = 0.25 # number of readings we get that incidences angle is too large
    self.WHEEL_RADIUS = 2.85
    self.ROBOT_RADIUS = 5.5 # centre of rotation to the wheel touching the ground
    self.WAYPOINT_RADIUS = 4 # distance we allow away from waypoint
    self.ROBOT_LENGTH = 20 # used when hit the wall to move back
    self.PERC_MOD_WHEEL = 1.025 # error in the wheel
    self.PERC_MOD_BOOST = 1.05 # extra move boost (extra distance) 
    self.ANGLE_BOOST = 1.14  # extra rotation boost (extra angle)
    self.walls = Map('DEBUG') # world representation
    self.particles = [Particle() for i in range(Particle.num_particles)] # (x, y, theta)
    self.sigma = 0.1325 # error in distance to update x and y in particles 
    self.error_theta = 0.5 * math.pi / 180 # error in rotation in angle to update theta in particles
    self.x = 0.0 # robot mean x location
    self.y = 0.0 # robot mean y location
    self.theta = 0.0 # robot mean theta angle (from x-axis) 0 - 2pi (TODO: Check)
    self.sonar_sigma = 0.4 # distance error in the sonar in cm
    self.interface = interface
    self.motorParams = params.setup(interface)    
    self.motors = [params.MOTOR_LEFT,params.MOTOR_RIGHT,params.MOTOR_HEAD] 
    self.draw_particles = []
    
    # enable all motors (0..2)
    for motor in self.motors: 
      interface.motorEnable(motor)
      interface.setMotorAngleControllerParameters(motor, self.motorParams[motor])
      
    # enable sensors TODO: enable light
    interface.sensorEnable(params.PORT_LEFT_TOUCH, brickpi.SensorType.SENSOR_TOUCH)
    interface.sensorEnable(params.PORT_RIGHT_TOUCH, brickpi.SensorType.SENSOR_TOUCH)
    interface.sensorEnable(params.PORT_ULTRASONIC, brickpi.SensorType.SENSOR_ULTRASONIC)   
   
    # set head values 
    self.HEAD_ANGLE_CENTRE = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    self.HEAD_ANGLE_LEFT = self.HEAD_ANGLE_CENTRE - math.pi 
    self.HEAD_ANGLE_RIGHT = self.HEAD_ANGLE_CENTRE + math.pi 
    self.HEAD_STEP = 15 * math.pi / 180    
    self.HEAD_WAIT = 0.25


  '''Returns object Map which contains walls'''
  def getWalls(self):
    return self.walls


  '''Change calibration values'''
  def set_drift_callibration(self, mod_wheel, boost, degs):
    self.PERC_MOD_WHEEL = mod_wheel
    self.PERC_MOD_BOOST = boost 
    self.ANGLE_BOOST = degs

  
  '''Sets robot location to (x,y,theta)'''
  def set_location(self, x, y, theta=0):
   self.x = x
   self.y = y
   self.theta = theta
   for particle in self.particles:
     particle.x = x
     particle.y = y
     particle.theta = theta

  
  '''Update weights for particles based on sonar readings'''
  def update_weights(self):
    sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)[0]

    new_weights = []
    num_acceptable = 0
    
    for particle in self.particles: 
      likelihood, acceptable = self.calculate_likelihood(self.walls, particle, sonar_reading)
      new_weights.append(particle.weight * likelihood)
      if acceptable:
        num_acceptable = num_acceptable + 1

    if num_acceptable > len(self.particles) * self.ACCEPT_THRESHOLD:
      for i in range(len(self.particles)):
        self.particles[i].weight = new_weights[i] / sum(new_weights)
    
   
  '''Takes angle returns it with respect to coordinate system 0 - 2pi '''
  def wrap_angle(self, angle):
    full_circle = math.pi  * 2
    if abs(angle) > full_circle:
      if angle > 0:
        return (angle % full_circle)
      else:
        return -(angle % full_circle)
    else:
      return angle
  

  '''Rotate radians anti-clockwise'''
  def rotate(self, radians, draw=False, verbose=False):
   
    # optimal number of radians for roation
    optimal_radians = self.wrap_angle(radians)
    
    if optimal_radians > math.pi:
      optimal_radians =  -(2 * math.pi - optimal_radians)

    print "rotating by ", optimal_radians
        
    # angle to rotate motor 
    motor_angle = self.ROBOT_RADIUS * (optimal_radians * self.ANGLE_BOOST) / self.WHEEL_RADIUS
        
    motors = [params.MOTOR_LEFT, params.MOTOR_RIGHT] 
 
    self.interface.increaseMotorAngleReferences(motors, [-motor_angle, motor_angle * 1.00])
   
    while not self.interface.motorAngleReferencesReached(motors):
      motorAngles = self.interface.getMotorAngles(motors)
      if motorAngles and verbose:
        print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
   
    # update theta for particles 
    for particle in self.particles:
      particle.theta = self.wrap_angle(particle.theta + optimal_radians + random.gauss(0, self.error_theta))
        
    self.update_weights()
    self.resample()

    if draw:
      # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
      for particle in self.particles:
        self.draw_particles.append((particle.x * params.DRAW_SCALE +
          params.DRAW_OFFSET_X, particle.y * params.DRAW_SCALE +
          params.DRAW_OFFSET_Y, particle.theta))
      print "drawParticles:" + str(self.draw_particles)

    # get total theta (normalised) and update self.theta
    total_theta = 0
    for particle in self.particles:
      total_theta = total_theta + particle.theta * particle.weight
    self.theta = self.wrap_angle(total_theta)    
    
    sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)[0]
    print "I am ", sonar_reading, " cm away from the wall"
    w, _, _, _ = self.walls.which_wall(self)
    
    print "I am looking at wall: ", w
   
  # Head methods START
   
  '''Puts head to the nearest default position, either looking left or looking right
     Returns koefficient to be used for direction of rotation'''
  def reset_head(self, verbose=False):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]    
    # coefficient to rotate head clockwise(1) anticlockwise (-1)
    direction_k = 1
    if (verbose):
      print 'Head before reset', current_head_angle
    if ( abs(current_head_angle - self.HEAD_ANGLE_LEFT) < abs(current_head_angle - self.HEAD_ANGLE_RIGHT)):
      self.look_left(verbose)
    else:
      self.look_right(verbose)
      direction_k = -1
    return direction_k


  '''Move head to look left'''
  def look_left(self, verbose=False):
    self.interface.setMotorAngleReference(params.MOTOR_HEAD, self.HEAD_ANGLE_LEFT)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      if verbose:
        print "Head angle: ", self.interface.getMotorAngle(params.MOTOR_HEAD)


  '''Move head to look right'''
  def look_right(self, verbose=False):
    self.interface.setMotorAngleReference(params.MOTOR_HEAD, self.HEAD_ANGLE_RIGHT)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      if verbose:
        print "Head angle: ", self.interface.getMotorAngle(params.MOTOR_HEAD)


  '''Move head to look straight'''
  def look_straight(self, verbose=False):
    self.interface.setMotorAngleReference(params.MOTOR_HEAD, self.HEAD_ANGLE_CENTRE)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      if verbose:
        print "Head angle: ", self.interface.getMotorAngle(params.MOTOR_HEAD)


  '''Rotate head radians clockwise. Return the last head angle and last sonar reading'''
  def rotate_head(self, radians, draw=False, verbose=False):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    
    if (current_head_angle + radians < self.HEAD_ANGLE_LEFT):
      radians = - current_head_angle + self.HEAD_ANGLE_LEFT
    elif (current_head_angle + radians > self.HEAD_ANGLE_RIGHT):
      radians = - current_head_angle + self.HEAD_ANGLE_RIGHT 

    self.interface.increaseMotorAngleReference(params.MOTOR_HEAD, radians)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]): 
      current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
      head_angle_offset = self.get_head_offset()
      if verbose:
        print "Angle from initial head angle: ", (head_angle_offset) * 180 / math.pi
   
    end_sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)
    
    print "Head offset ",head_angle_offset
    print "Theta + head offset", (head_angle_offset + self.theta)

    if draw:
      robot_mean_particle = Particle(self.x, self.y, (self.theta + head_angle_offset))
      best_wall, minimum_m, incident_angle, hit_point = self.walls.which_wall(robot_mean_particle, verbose)
      print "drawLine:" + str((self.x,self.y,hit_point[0], hit_point[1]))
   
    return current_head_angle, end_sonar_reading


  ''' Prints head angle'''
  def print_head_angle(self):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]    
    print 'Head angle', current_head_angle


  '''Looks arounds between left and right head angles.
     Returns array of tuples with angles and sonar readings'''
  def look_around(self, draw=False, verbose=False):
    direction_k = self.reset_head()
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)     
    angles_distances = [(current_head_angle, sonar_reading)]

    while (current_head_angle >= self.HEAD_ANGLE_LEFT
           and current_head_angle <= self.HEAD_ANGLE_RIGHT) :
      current_head_angle, sonar_reading = self.rotate_head(direction_k * self.HEAD_STEP, draw, verbose)
      angles_distances.append((self.get_head_offset(), sonar_reading))
      if verbose:
        print "Head angle: ", current_head_angle, "Ultrasonic: ", sonar_reading
      time.sleep(self.HEAD_WAIT)

    return angles_distances


  ''' Get offset angle of head from centre in radians '''
  def get_head_offset(self, verbose=False):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    return self.HEAD_ANGLE_CENTRE - current_head_angle

  # Methods for head END


  # Methods for move START

  '''Brute move distance cm forward ignoring everything'''
  def force_move(self, distance, verbose=False):
    self.interface.increaseMotorAngleReferences(self.motors,[(distance * self.PERC_MOD_BOOST)/self.WHEEL_RADIUS, self.PERC_MOD_WHEEL * distance * self.PERC_MOD_BOOST/self.WHEEL_RADIUS])
    
    while not self.interface.motorAngleReferencesReached(self.motors) :
      motorAngles = self.interface.getMotorAngles([0,1])
      if motorAngles and verbose:
        print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
 

  '''Move distance cm forward with conditions
     Returns'''
  def move(self, distance, draw=False, verbose=False):
    bumper_hit_left = 0
    bumper_hit_right = 0
    
    self.interface.increaseMotorAngleReferences(self.motors,[(distance * self.PERC_MOD_BOOST)/self.WHEEL_RADIUS, self.PERC_MOD_WHEEL * distance * self.PERC_MOD_BOOST/self.WHEEL_RADIUS])

    while not self.interface.motorAngleReferencesReached(self.motors) and not bumper_hit_left and not bumper_hit_right:
      motorAngles = self.interface.getMotorAngles([0,1])
      bumper_hit_left = self.interface.getSensorValue(params.PORT_LEFT_TOUCH)[0]
      bumper_hit_right = self.interface.getSensorValue(params.PORT_RIGHT_TOUCH)[0]
      sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)      
     
      if bumper_hit_left or bumper_hit_right:
        print "Hit bumper?"
        print bumper_hit_left
        print bumper_hit_right
        self.reset_motors()
        self.recover()
        if verbose:
          print "Hit bumper"
        return
       
      if verbose:
        if motorAngles:
          print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]   
        if bumper_hit_left or bumper_hit_right: 
          print "Left: " + str(bumper_hit_left) + " , right: " + str(bumper_hit_right) 
        if sonar_reading:
          print "Ultrasonic: " + str(sonar_reading)
        else:
          print "Failed to read"

    # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
    prev_x = self.x
    prev_y = self.y
    
    # Update particles pose (already moved) 
    for particle in self.particles:
      e = random.gauss(0, self.sigma) * distance
      particle.x = particle.x + (distance + e) * math.cos(particle.theta)
      particle.y = particle.y + (distance + e) * math.sin(particle.theta)
      particle.theta = particle.theta + random.gauss(0, self.error_theta)
      

    self.update_weights()
    self.resample() 
   
    # Update robot's mean location (totals are nomalised)
    total_x = 0
    total_y = 0  
    for particle in self.particles:
      total_x = total_x + particle.x * particle.weight
      total_y = total_y + particle.y * particle.weight
    self.x = total_x
    self.y = total_y


    # Draw particles     
    if draw:
      for particle in self.particles:
        self.draw_particles.append((-(particle.x * params.DRAW_SCALE +
          params.DRAW_OFFSET_X), -(particle.y * params.DRAW_SCALE +
          params.DRAW_OFFSET_Y), particle.theta))
      print "drawParticles:" + str(self.draw_particles)
      print "drawLine:"+str((params.DRAW_SCALE * prev_x + params.DRAW_OFFSET_X,
                             params.DRAW_SCALE * prev_y + params.DRAW_OFFSET_Y,
                             params.DRAW_SCALE * self.x + params.DRAW_OFFSET_X,
                             params.DRAW_SCALE * self.y + params.DRAW_OFFSET_Y))
   
    # Reset motors to 0 to avoid angle reference problem
    self.reset_motors()
    
    sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)[0]
    print "I am ", sonar_reading, " cm away from the wall"
    w, _, _, _ = self.walls.which_wall(self)
    
    print "I am looking at wall: ", w
    
    return bumper_hit_left, bumper_hit_right, sonar_reading


  ''' Follow the wall for distance '''
  def follow(self, distance, distanceFrom, draw=False, verbose=False):
    self.interface.increaseMotorAngleReferences(self.motors,[(distance * self.PERC_MOD_BOOST)/self.WHEEL_RADIUS, self.PERC_MOD_WHEEL * distance * self.PERC_MOD_BOOST/self.WHEEL_RADIUS])

    acceleration, speed, forwardGain = params.getSpeedParams(self.interface,0)
    directon_k = 1 #looking left 1, looking right -1
    old_sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)[0] 
 
    while not self.interface.motorAngleReferencesReached(self.motors):
      motorAngles = self.interface.getMotorAngles([0,1])
      sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)[0]      
      if (self.get_head_offset() > 0):
        direction_k = -1
      if (sonar_reading > distanceFrom):
        print "Sonnar > distance"
        if (sonar_reading > max(1.5*old_sonar_reading,255)):
          self.reset_motors()
          return
        params.setSpeedParams(self.interface, 0, acceleration, 0, forwardGain)
      elif (sonar_reading < distanceFrom):         
        print "Sonnar < distance"
        if (sonar_reading < min(0.5*old_sonar_reading,0)):
          self.reset_motors()
          return
        params.setSpeedParams(self.interface, 0, acceleration, 0, forwardGain)
      old_sonar_reading = sonar_reading

  ''' Move until certain distance from the wall '''
  def move_to_distance_from(self, distanceFrom, draw=False, verbose=False):     
    sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)[0]
    distance = sonar_reading - distanceFrom
    self.move(distance, draw, verbose)


  ''' Recovers position of robot after hitting the wall '''
  def recover(self):
    self.move(-self.ROBOT_LENGHT) 


  ''' Terminates interface '''
  def terminate_interface(self):
    self.interface.terminate()

  
  ''' Finds a particle index at which the cumulative weight is in the right
  bucket for randomly generated weight '''
  def get_particle(self, rnd_weight, cp_arr):
    split = len(cp_arr) / 2

    if rnd_weight <= cp_arr[split]:
      if split == 0:
        return 0
      elif rnd_weight > cp_arr[split - 1]:
        return split
      else:
        return self.get_particle(rnd_weight, cp_arr[:split])
    else:
      return self.get_particle(rnd_weight, cp_arr[split:]) + split
     

  ''' Generate new particles whose weights are all equal to 1 / N but whose
  spatial distribution represents the previous weighted distribution.'''
  def resample(self):
    # cumulative probability array  
    cpa = []
    new_particles = []
    # total probability
    total_p = 0

    for i in range(len(self.particles)): 
      total_p = total_p + self.particles[i].weight
      cpa.append(total_p)
     
    for i in range(len(self.particles)):
      rnd = random.uniform(0, 1)
      index = self.get_particle(rnd, cpa) 
      p = Particle() 
      p.x = self.particles[index].x
      p.y = self.particles[index].y
      p.theta = self.particles[index].theta
      p.weight = 1.0 / len(self.particles)
      new_particles.append(p)
    
    self.particles = new_particles


  ''' Move to a waypoint from current location with or without intervals'''
  def move_to_waypoint(self, x, y, interval=0, draw=False, verbose=False):
    new_theta = self.theta 
    if verbose:
     print "Aheading to the way point ",x,",",y
     print "Current position and heading params: ",self.x, ",",self.y,",",self.theta

    delta_x = x - self.x
    delta_y = y - self.y
    
    beta = math.atan(delta_y / delta_x)
     
    abs_beta = abs(beta)     
   
    if (delta_x == 0):
      if (delta_y < 0):
        new_theta = 3 * math.pi / 2
      elif (delta_y > 0):
        new_theta = math.pi / 2
    elif (delta_y == 0):
      if (delta_x < 0):
        new_theta = math.pi
      elif (delta_x > 0):
        new_theta = 0
    elif (x < self.x and y < self.y):
      new_theta = math.pi + abs_beta
    elif (x > self.x and y < self.y):
      new_theta = 2 * math.pi - abs_beta
    elif (x < self.x and y > self.y):
      new_theta = math.pi - abs_beta
    elif (x > self.x and y > self.y):
      new_theta = abs_beta
  
    print "I'm currently facing ", self.theta, ", I want to get to ", new_theta, " so i'm going to turn by ", (new_theta - self.theta)
    turn_angle = (new_theta - self.theta)
    
    if verbose:
       print "new_theta : " , new_theta
       print "turn_angle : " , turn_angle

    # calculating distance should move 
    move_distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
        
    if interval > 0:
     if move_distance > self.WAYPOINT_RADIUS:
        self.rotate(turn_angle,draw,verbose) 
        self.move(min(interval,move_distance), draw, verbose) 
        self.move_to_waypoint(x,y,interval,draw,verbose)
    else:
      self.rotate(turn_angle,draw,verbose) 
      self.move(move_distance,draw,verbose)
      print "waypoint (", x, ", ", y, ") reached"

  # move methods END

  '''Clears rotation commands on all motors.'''
  def reset_motors(self, reset_left=True, reset_right=True, reset_head=False): 
   if reset_left:
     self.interface.motorDisable(params.MOTOR_LEFT)
     self.interface.motorEnable(params.MOTOR_LEFT)
   if reset_right:
     self.interface.motorDisable(params.MOTOR_RIGHT)
     self.interface.motorEnable(params.MOTOR_RIGHT)
   if reset_head:
     self.interface.motorDisable(params.MOTOR_HEAD)
     self.interface.motorEnable(params.MOTOR_HEAD) 

  '''Calculate likelihood (for updating weights) '''
  def calculate_likelihood(self, walls, particle, z):
    self.LIKELIHOOD_OFFSET = 0.01
    wall, m, angle, hp = walls.which_wall(particle)
    if angle > 15 * math.pi / 180:
      return 1, False
    else:
      return self.LIKELIHOOD_OFFSET + math.exp(-((z - m) ** 2) / (2 * (self.sonar_sigma ** 2))), True




#####ed#####

  # '''Rotate head radians clockwise. Return the last head angle and last sonar reading'''
  # def environment_scan(self, radians, draw=False, verbose=False):
  #   current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    
  #   if (current_head_angle + radians < self.HEAD_ANGLE_LEFT):
  #     radians = - current_head_angle + self.HEAD_ANGLE_LEFT
  #   elif (current_head_angle + radians > self.HEAD_ANGLE_RIGHT):
  #     radians = - current_head_angle + self.HEAD_ANGLE_RIGHT 

  #   self.interface.increaseMotorAngleReference(params.MOTOR_HEAD, radians)
  #   while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]): 
  #     current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
  #     head_angle_offset = self.get_head_offset()
  #     if verbose:
  #       print "Angle from initial head angle: ", (head_angle_offset) * 180 / math.pi
   
  #   end_sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)
    
  #   print "Head offset ",head_angle_offset
  #   print "Theta + head offset", (head_angle_offset + self.theta)

  #   if draw:
  #     robot_mean_particle = Particle(self.x, self.y, (self.theta + head_angle_offset))
  #     best_wall, minimum_m, incident_angle, hit_point = self.walls.which_wall(robot_mean_particle, verbose)
  #     print "drawLine:" + str((self.x,self.y,hit_point[0], hit_point[1]))
   
  #   return current_head_angle, end_sonar_reading


##mark down current head angle and return 
  def get_abs_head_angle_ed(self):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    return current_head_angle




  def reset_head_ed(self,abs_angle,verbose):
    self.interface.setMotorAngleReference(params.MOTOR_HEAD,abs_angle)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      pass


    # self.interface.setMotorAngleReference(params.MOTOR_HEAD,self.)

    # self.interface.setMotorAngleReference(params.MOTOR_HEAD, self.HEAD_ANGLE_RIGHT)
    # while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
    #   if verbose:
    #     print "Head angle: ", self.interface.getMotorAngle(params.MOTOR_HEAD)


  def turn_head_to_angle_ed(self, radians, verbose=False):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    
    if (current_head_angle + radians < self.HEAD_ANGLE_LEFT):
      radians = - current_head_angle + self.HEAD_ANGLE_LEFT
    elif (current_head_angle + radians > self.HEAD_ANGLE_RIGHT):
      radians = - current_head_angle + self.HEAD_ANGLE_RIGHT 

    self.interface.increaseMotorAngleReference(params.MOTOR_HEAD, radians)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      pass



      ## envrinoment scan 
  def environment_scan(self, radians, abs_head_angle,verbose=False):

    environment_data_buffer =[]

    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    
    if (current_head_angle + radians < self.HEAD_ANGLE_LEFT):
      radians = - current_head_angle + self.HEAD_ANGLE_LEFT
    elif (current_head_angle + radians > self.HEAD_ANGLE_RIGHT):
      radians = - current_head_angle + self.HEAD_ANGLE_RIGHT 

    self.interface.increaseMotorAngleReference(params.MOTOR_HEAD, radians)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
      relative_head_angle = current_head_angle - abs_head_angle
      sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)[0]
      environment_data_buffer.append([relative_head_angle,sonar_reading]) #append to data buffer for store
      #print "My head is in :" + str(relative_head_angle) + "with deepth" + str(sonar_reading) 
      time.sleep(0.04)
      pass

    #print environment_data_buffer for DEBUG
    self.save_environment(environment_data_buffer)

    return environment_data_buffer

  def save_environment(self,environment_data_buffer): 
    f = open('environment_output.txt', 'w') 
    for data in environment_data_buffer:
      f.write("%s %s\n" % (float(data[0]),float(data[1])))



















