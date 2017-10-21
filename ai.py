#Purpose of this function is to provide state control for the robot, 
#Inputs to this module are: 
#Array ball_loc[x,z] (x,y should be null if no ball visible)
#Array goal_loc[x,z] (x,y should be null if no Goal  visible)
#Array obs_loc[x,z] (x,y should be null if no Obstacle  visible)

#Structure -> CV -> State controller -> state -> pathfinding -> motor controller

import math
import numpy as np
# import matplotlib.pyplot as plt
import random
import cProfile
import re 
####
#search_counter = 0
#self.virt_balll = 180
#ai_config = config['ai']
####

#TODO: Rmake a number of the for loops to utilize Numpy, as opposed to 
#TODO: REMOVE ALL DEGREE TO RADIAN CONVERSIONS
#TODO: USE SIMX TO LATCH BALL ONTO ROBOTT
#Using the built in python libraries
#Change objects to obstacles

class AI:
    def __init__(self, config):
        self.config = config['ai']
        self.search_counter = 0
        self.virt_ball = 180
        self.forward_vel = self.config['forwardVelocity']
        self.rotate_vel = self.config['rotationalVelocity']
        self.virt_goal = 180
        self.status = {}

       # 
    #----------------------------------------------------------------
    #Primary State controller function, handles calling all 
    #other functions. RIP OOP
    #---------------------------------------------------------------
    def state_controller(self,ball, objects, goal, wall):
        #Set up parameters
        sub_state = ''
        sum_field = []

        if ball is not None and ball[0][1] is None:
            ball = None
        if goal is not None and goal[0][1] is None:
            goal = None

        wall = [point for point in wall if point[0][1] is not None]
        objects = [point for point in objects if point[0][1] is not None]

        state = self.determine_state(ball,objects,goal)
        #print("Rolling with state number:",state,"Counter value is:",self.search_counter)

        desired_heading = self.config['initHeading']
        velocity = 0 #Units for Velocity is m/s

        if (state == 1): #State 1 (grabbing ball)
            sub_state = self.determine_sub_state(ball,objects,goal, state)
            #print('state is:',state,'sub_state is:',sub_state)
            if (sub_state == 'finding_ball'):
                #Need to rotate for a random count between 1 and 5
                #Generate a random ball location
                if self.search_counter >= self.config['ballSearchMoveCount'] and self.search_counter <= self.config['ballSearchSpinCount']:
                    print("Spinning on point, search counter:",self.search_counter)
                    self.search_counter +=1
                    #INSERT CODE TO PERFORM 360 DEGREE SPIN
                    desired_heading = 270
                    velocity = 0

                elif self.search_counter >= self.config['ballSearchSpinCount'] or self.search_counter == 0:
                    self.virt_ball = self.generate_virtual_ball_random_loc()
                    self.search_counter = 1
                    print('Searching for ball, placing virtual ball at:', self.virt_ball)
                
                elif (self.search_counter == self.config['virtballturncount']):
                    self.virt_ball = self.generate_virtual_ball_infront()
                    self.search_counter += 1
                    print('Have finished turning for virt ball, now moving foward', self.virt_ball)
                    velocity = self.forward_vel
              
                else:
                    print("Moving towards virtual ball at heading:", self.virt_ball)
                    self.search_counter +=1
                    attract_field = self.create_attraction_field(self.virt_ball)
                    repulse_field = self.create_repulsion_field(objects) + self.create_repulsion_field_wall(wall)
                    sum_field = attract_field - repulse_field 
                    desired_heading = sum_field.argmax(axis = 0) - 180
                    velocity = self.forward_vel
                    #print('heading towards: ',desired_heading, "At velocity:",velocity)

            elif (sub_state == 'moving_to_ball'):
                self.search_counter = 0;
                attract_field =  self.create_attraction_field(ball)
                repulse_field = self.create_repulsion_field(objects) + self.create_repulsion_field_wall(wall)
                sum_field = attract_field - repulse_field 
                desired_heading = (sum_field.argmax(axis = 0) - 180)
                #print(sum_field)

                velocity = self.forward_vel
                print('found ball heading towards: ',desired_heading)
                #print('----------------------------')

        elif (state == 2): #Aligning ball with goal

            sub_state = self.determine_sub_state(ball,objects,goal, state)
            if (sub_state ==  'Looking for Goal'):
                #Create a virtual goal location, move towards it
                #Then do a spin to try and spot it.
                if self.search_counter >= 20 and self.search_counter <= 30:
                    print("Spinning on point, looking for goal, search counter:",self.search_counter)
                    self.search_counter +=1
                    #INSERT CODE TO PERFORM 360 DEGREE SPIN
                    desired_heading = 270
                    velocity = 0

                elif self.search_counter >= 30 or self.search_counter == 0:
                    self.virt_goal = self.generate_virtual_ball_infront()
                    self.search_counter = 1
                    print('Searching for Goal, placing virtual goal at:',self.virt_goal)    
                
                
                elif (self.search_counter == self.config['virtballturncount']):
                    self.virt_goal= self.generate_virtual_ball_infront()
                    self.search_counter += 1
                    print('Have finished turning for virt ball, now moving foward',self.virt_goal)
                    velocity = self.forward_vel
              

                else:
                    print("Moving towards virtual goal at heading:", self.virt_goal)
                    self.search_counter +=1
                    attract_field = self.create_attraction_field(self.virt_goal)
                    repulse_field = self.create_repulsion_field(objects) + self.create_repulsion_field_wall(wall)
                    sum_field = attract_field - repulse_field 
                    desired_heading = sum_field.argmax(axis = 0) - 180
                    velocity = self.forward_vel
                    #print('heading towards: ',desired_heading, "At velocity:",velocity)


            elif(sub_state == 'Found Goal'):
                print("found goal, moving towards it")
                self.search_counter = 0
                attract_field = self.create_attraction_field(goal)
                repulse_field = self.create_repulsion_field(objects) + self.create_repulsion_field_wall(wall)
                sum_field = attract_field - repulse_field
                desired_heading = sum_field.argmax(axis = 0) - 180
                velocity = self.forward_vel
            
        print('----------------------------')
        print("State:",state,"Substate:",sub_state)
        #print(attract_field);
        desired_heading = np.clip(desired_heading,-90,90)
        desired_rot = (desired_heading/90) 
        print("search counter:",self.search_counter)
        #print("DESIRED HEADING:",desired_rot)
        #self.plot_state(ball,objects,goal,wall,desired_heading,self.virt_balll)
        heading = 90
        
        #Conversion shit for sim:
        desired_rot = desired_rot * -self.rotate_vel
        #print("movement:",vrep.simxGetObjectPosition(clientID, robot,-1, vrep.simx_opmode_blocking)[1])
        #return (heading, desired_rot, velocity)
        # if len(sum_field) != 0:
        #     plt.cla()
        #     plt.plot(sum_field)
        #     plt.plot([desired_heading + 180] * 2, [0,1], 'r--')
        #     plt.plot([(math.degrees(ang) + 180 ) for (ang, _), _, _ in wall], np.ones(len(wall)), 'og')
        #     plt.show(block=False)
        #     plt.pause(0.01)
        self.status = {
            'state': state,
            'substate': sub_state,
            'desired_rot': float(desired_rot),
            'desired_velocity': float(velocity)
        }
        #
        # return (desired_rot, velocity,state)
        #May not work right if we are subctracting 90 from the heading
        #If shit doesn't work, start by changing this
        #return (math.radians(desired_heading),velocity)


    #----------------------------------------------------------------
    #Convers the given locations, and returns angles relative to the 
    #robot's reference fram
    #----------------------------------------------------------------
    def return_angles(self,ball_loc, goal_loc,obj_loc,robot_rot):
        ball_ang = math.degrees(math.atan2(ball_loc[1],ball_loc[0]))
        goal_ang = math.degrees(math.atan2(goal_loc[1],goal_loc[0]))
        obj_ang = math.degrees(math.atan2(obj_loc[1],obj_loc[0]))
        return ball_ang, goal_ang, obj_ang


    #----------------------------------------------------------------
    #Detrmines whether or not the robot can "see" the ball.
    #Only useful in the simulation
    #----------------------------------------------------------------
    def determine_can_see(self,ball_ang,goal_ang,obj_ang):
        see_ball = abs(ball_ang) <= 45
        see_goal = abs(goal_ang) <= 45
        see_obj = abs(obj_ang) <= 45

        return see_ball, see_goal, see_obj

    #----------------------------------------------------------------
    #Creates an attraction field, based entirely on the relative
    #angle given to it
    #----------------------------------------------------------------
    def create_attraction_field(self,ball): 
       ballpol,_,_ = ball
       ramp = np.linspace(0, 1, num=180, endpoint=False)
       ramp = ramp[1:180]
       base_field = np.concatenate([[1], np.flip(ramp, axis=0), [0], ramp])
       ball_ang = int(round(math.degrees(ballpol[0]) + 180)) % 360
       rotated_field = np.roll(base_field, ball_ang, axis=0)
       return np.reshape(rotated_field, (-1, 1))
       

    #----------------------------------------------------------------
    #Creates a repulsion based on both the object location
    #and angle
    #----------------------------------------------------------------
    def create_repulsion_field(self,objects):
        #print("REPULSION")
        #Initial set up
        robot_radius = 0.18/2
        obj_width = 2
        obj_fields = []
        if len(objects) == 0:
            return np.zeros((360,1))
        for obj in objects:
            obj_pol,_,_ = obj
            obj_ang,obj_dist= obj_pol
            obj_ang = int(math.degrees(obj_ang) + 180)
            try:
                obj_width_ang = int(math.degrees(math.asin(0.5*obj_width/obj_dist)))
            except ValueError as e:
                obj_width_ang = 45
                print("object 2 large 4 sin")
            cur_field = np.zeros((360,1))

            left_ang = self.clip_angle_360(obj_ang - obj_width_ang)
            right_ang = self.clip_angle_360(obj_ang + obj_width_ang)
            
            if left_ang < right_ang:
                cur_field[left_ang:right_ang] = 1
            else:
                cur_field[left_ang:] = 1
                cur_field[:right_ang] = 1

            
            
            #cur_field[obj_ang - obj_width_ang : obj_ang + obj_width_ang] = 1
            obj_fields.append(cur_field)

        field_matrix = np.concatenate(obj_fields, axis=1)
        field_max = np.max(field_matrix, axis=1)
        return np.reshape(field_max, (-1,1))

        #return rep_field


    #----------------------------------------------------------------
    #Creates a repulsion based on both the wall location
    #and angle
    #----------------------------------------------------------------
    def create_repulsion_field_wall(self,wall): 
    ##print("REPULSION")
        #Initial set up
        obj_fields = []
        if len(wall) == 0:
            return np.zeros((360,1))
        for obj in wall:
            obj_pol,_,_ = obj
            obj_ang,obj_dist= obj_pol
            obj_ang = int(math.degrees(obj_ang) + 180)
            obj_dist = min(2,obj_dist)
            #print("Creating obj_wall, obj_ang:",obj_ang)
            cur_field = np.zeros((360,1))
            if (obj_ang < 360):
                cur_field[obj_ang] = 1 - (obj_dist/2)
            obj_fields.append(cur_field)

        field_matrix = np.concatenate(obj_fields, axis=1)
        field_max = np.max(field_matrix, axis=1)
        #print(field_max)
        return np.reshape(field_max, (-1,1))

    #----------------------------------------------------------------
    #Convers the desired heading and ball_angle into commands for the
    #Motor controller to use
    #----------------------------------------------------------------
    def motor_controller(self,desired_heading, ball_ang):
        max_vel = 0.1
        max_rot = 0.8
        goal_p = 0.05
        angle_delta = abs(180 - desired_heading)
        #print('angle_delta is' , angle_delta)
        #print('goal_ang is', ball_ang, 'desired_heading is',desired_heading)
        #print('currently using angle_delta as:', angle_delta * goal_p)
        desired_rot = min(max_rot, angle_delta * goal_p)
        #print('direction to head is', 180 - desired_heading)
        #print('the desired rot vel is',desired_rot)
        wheel_base = 0.2
        time_step = 0.05
        r_acc = 1 * time_step
        if (180 - desired_heading < 0 ):
            desired_rot *= -1
        vr = max_vel + desired_rot * wheel_base / 2
        vl = max_vel - desired_rot * wheel_base / 2
        return vr,vl

    #----------------------------------------------------------------
    #Angle for trimming an angle into a format within 0-360,
    #useful for when an angle is <0 or >360
    #Note: Does NOT work for angle greater than 720 or less than
    #-360
    #----------------------------------------------------------------
    def clip_angle_360(self,angle):
        negative = angle < 0
        angle = abs(angle) % 360
        if negative:
            return 359 - angle
        return angle


    #----------------------------------------------------------------
    #Used for determining substates.
    #----------------------------------------------------------------
    #def determine_sub_state(see_b, see_g, see_obj, state):
    def determine_sub_state(self,ball,objects,goal, state):
        if (state == 1): #Looking for the ball, is it visible?
            if ball is None:
                sub_state = 'finding_ball'
                #print(sub_state)
            else:
                sub_state = 'moving_to_ball'
                #print(sub_state)
        elif (state == 2): #Have ball in dribbler, align with the goal
            if goal is None:
                sub_state = 'Looking for Goal'
                print('substate set to:', sub_state)
            else:
                sub_state = 'Found Goal'
                print('substate set to:', sub_state)
        return sub_state

    #----------------------------------------------------------------
    #Used for determining states.
    #----------------------------------------------------------------
    def determine_state(self,ball,objects,goal):
        
        if ball is None:
            return 1
            
        polar, cartesian, reliable = ball
        bearing, range = polar
        #print("range", range)
        if range >= self.config['distToGrab']:
            state = 1
        else:
            state = 2
        return state

    #----------------------------------------------------------------
    #Generate a random location for the ball
    #----------------------------------------------------------------
    def generate_virtual_ball_random_loc(self):
        #Create ball at random angle (0-360), at distance 0.5, 
        vball_abs_dist = 2
        vball_ang_deg = random.randint(0,360) - 180
        vball_ang_rad = math.radians(vball_ang_deg)
        vball_pol = (vball_ang_rad,vball_abs_dist)
        vballxy = (vball_abs_dist*math.cos(vball_ang_rad),vball_abs_dist*math.sin(vball_ang_rad))
        vball = (vball_pol,vballxy,True)
        return vball

 
    #----------------------------------------------------------------
    #Generate a ball infront of robot
    #----------------------------------------------------------------
    def generate_virtual_ball_infront(self):
        vball_abs_dist = 2
        vball_ang_deg = 0
        vball_ang_rad = math.radians(vball_ang_deg)
        vball_pol = (vball_ang_rad,vball_abs_dist)
        vballxy = (vball_abs_dist*math.cos(vball_ang_rad),vball_abs_dist*math.sin(vball_ang_rad))
        vball = (vball_pol,vballxy,True)
        print("creating ball in front, location is:",vball)
        return vball

         

    # #----------------------------------------------------------------
    # #Plot the state
    # #----------------------------------------------------------------
    # def plot_state(self,ball,objects,goal,wall,heading,vball):
    #
    #     heading_vect_length = 1
    #     headingx = heading_vect_length * math.cos(math.radians(90 - heading))
    #     headingy = heading_vect_length * math.sin(math.radians(90 - heading))
    #
    #
    #     #print(headingx,headingy)
    #     plt.cla()
    #
    #     if ball is not None:
    #         _, ballxz, _ = ball
    #         plt.plot(ballxz[0],ballxz[1], 'ro')
    #
    #     if vball is not None:
    #         _, vballxz, _ = vball
    #         plt.plot(vballxz[0],vballxz[1], 'yo')
    #
    #
    #     objectCart = [object[1] for object in objects]
    #     objectX = [point[0] for point in objectCart]
    #     objectZ = [point[1] for point in objectCart]
    #     plt.plot(objectX, objectZ,'bo')
    #
    #     wallCart = [wallObj[1] for wallObj in wall]
    #     wallX = [point[0] for point in wallCart]
    #     wallZ = [point[1] for point in wallCart]
    #     plt.plot(wallX, wallZ,'go')
    #
    #
    #     plt.plot([0,headingx],[0.,headingy])
    #     plt.title('Red = Ball, Blue = objects, Green = Goal')
    #     plt.axis([-2, 2, -2, 2])
    #     plt.grid(True)
    #     plt.show(block=False)
    #     plt.draw()
    #     plt.pause(0.01)


