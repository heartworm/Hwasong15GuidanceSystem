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
import time
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
        self.forward_vel = self.config['forwardVelocity']
        self.rotate_vel = self.config['rotationalVelocity']
        self.status = {}
        self.virt_goal = self.generate_virtual_ball_infront()
        self.virt_ball = self.generate_virtual_ball_infront()
        self.search_start_time = 0
        self.sub_state = ''
        self.field_num = 30
        self.field_length = self.field_num * 2 + 1
        self.field_cent = self.field_num
        self.max_ang = config['vision']['fov'][0] / 2
        self.search_state = ''
        self.found_ball = False
        self.last_ball_left = False
        self.last_goal_left = False
        self.sum_field = []

    def ang_to_nang(self, ang):
        return ang / self.max_ang

    def ang_to_ind(self, ang):
        return self.nang_to_ind(self.ang_to_nang(ang))

    def nang_to_ind(self, nang):
        nang = self.clip_nang(nang)
        return self.field_cent + int(round((nang * self.field_num)))
      
    def ind_to_nang(self, ind):
        return (ind - self.field_cent) / float(self.field_num)
        
    def field_to_nang(self, field):
        nang_list = np.linspace(-1, 1, num=self.field_length, endpoint = True)
        field = np.reshape(field, (-1))
        field = np.maximum(field, 0)
        print("field", field)
        print("sum", np.sum(field))
        center_of_mass = np.sum(np.multiply(nang_list, field)) / np.sum(field)

        print("nang", center_of_mass)

        return center_of_mass
        # return self.ind_to_nang(field.argmax(axis=0))

    def field_master(self, good, obstacles, walls, velocity):
        attract_field = self.create_attraction_field(good)

        print("attraction", np.reshape(attract_field, (-1)))

        # repulsion_field = np.maximum(self.create_repulsion_field(obstacles), self.create_repulsion_field_wall(walls))
        repulsion_field = self.create_repulsion_field(obstacles)
        repulsion_field *= 0.5

        print("repulsion", np.reshape(repulsion_field, (-1)))

        sum_field = attract_field - repulsion_field

        self.sum_field = sum_field

        if np.max(sum_field) <= 0.5:
            print('obscured')
            return 0, 1

        heading = self.field_to_nang(sum_field)
        print("nang", heading)
        return velocity, heading


    #----------------------------------------------------------------
    #Primary State controller function, handles calling all 
    #other functions. RIP OOP
    #---------------------------------------------------------------
    def state_controller(self,ball, objects, goal, wall):
        #Set up parameters
        
        sum_field = np.zeros((self.field_length, 1))

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
            last_sub_state = self.sub_state
            self.sub_state = self.determine_sub_state(ball,objects,goal, state)
            #print('state is:',state,'sub_state is:',sub_state)

            if (self.sub_state == 'finding_ball'):
                elapsed_time = time.time() - self.search_start_time
                # print('elapsed time: ', elapsed_time)

                spin_time = self.config['ballSearchSpinCount']
                full_search_time = spin_time + self.config['ballSearchMoveCount']
                if last_sub_state != 'finding_ball' or elapsed_time >= full_search_time:
                    print('resetting search')
                    self.search_state = 'resetting'
                    self.search_start_time = time.time()
                elif elapsed_time >= spin_time:
                    print('moving forward')
                    self.search_state = 'moving'
                    velocity, desired_heading = self.field_master(None, objects, wall, self.forward_vel)
                else:
                    print('scanning')
                    self.search_state = 'turning'
                    velocity = 0
                    desired_heading = -1 if self.last_ball_left else 1

                    if ((elapsed_time) // 0.5) % 2 == 1:
                        desired_heading = 0
                    
            elif (self.sub_state == 'moving_to_ball'):
                self.search_counter = 0
                velocity, desired_heading = self.field_master(ball, objects, wall, self.forward_vel)
                # print('found ball heading towards: ',desired_heading)
                #print('----------------------------')
        elif (state == 2): #Aligning ball with goal
            last_sub_state = self.sub_state
            self.sub_state = self.determine_sub_state(ball,objects,goal, state)
            if (self.sub_state ==  'Looking for Goal'):

                elapsed_time = time.time() - self.search_start_time

                spin_time = self.config['goalSearchSpinCount']
                full_search_time = spin_time + self.config['goalSearchMoveCount']
                if last_sub_state != 'Looking for Goal' or elapsed_time >= full_search_time:
                    print('resetting search')
                    self.search_state = 'resetting'
                    self.search_start_time = time.time()
                elif elapsed_time >= spin_time:
                    print('moving forward')
                    self.search_state = 'moving'
                    velocity, desired_heading = self.field_master(None, objects, wall, self.forward_vel)
                else:
                    print('scanning')
                    self.search_state = 'turning'
                    velocity = 0
                    desired_heading = -1 if self.last_goal_left else 1
                    if ((elapsed_time) // 0.5) % 2 == 1:
                        desired_heading = 0




            elif(self.sub_state == 'Found Goal'):
                # print("found goal, moving towards it")
                self.search_counter = 0
                # attract_field = self.create_attraction_field(goal)
                # repulse_field = self.create_repulsion_field(objects) + self.create_repulsion_field_wall(wall)
                # sum_field = attract_field - repulse_field
                # desired_heading = self.field_to_nang(sum_field)
                # velocity = self.forward_vel
                velocity, desired_heading = self.field_master(goal, objects, wall, self.forward_vel)

            elif self.sub_state == 'Shoot':
                velocity = 0
                desired_heading = 0
            
        # print('----------------------------')
        # print("State:",state,"Substate:",self.sub_state)

        desired_rot = desired_heading

        # print("desired heading is", desired_rot)


        # velocity_multiplier = ((-1 * abs(desired_rot)) + 0.5) * 2




        velocity_multiplier = ((-1 * abs(desired_rot)) + 1)

        # velocity_multiplier = 1

        new_velocity = velocity * velocity_multiplier
        min_vel = 0.25
        if abs(velocity) >= min_vel and abs(new_velocity) < min_vel:
            if velocity < 0:
                velocity = -min_vel
            else:
                velocity = min_vel
        else:
            velocity = new_velocity

        #Conversion shit for sim:
        desired_rot = desired_rot * -self.rotate_vel
        #print("movement:",vrep.simxGetObjectPosition(clientID, robot,-1, vrep.simx_opmode_blocking)[1])
        #return (heading, desired_rot, velocity)
        # if len(sum_field) != 0:
        #     plt.cla()
        #     plt.plot(np.linspace(-1, 1, self.field_length, endpoint=True), sum_field)
        #     plt.plot([desired_heading] * 2, [0,1], 'r--')
        #     plt.plot([self.ang_to_nang(ang) for (ang, _), _, _ in wall], np.ones(len(wall)), 'og')
        #     plt.show(block=False)
        #     plt.pause(0.01)
        #
        # if (state == 2):
        #     desired_rot = 0
        #     velocity = 0

        self.status = {
            'state': state,
            'subState': self.sub_state,
            'desiredRot': float(desired_rot),
            'desiredVelocity': float(velocity),
            'searchState': self.search_state
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
        if ball is None:
            return np.ones((self.field_length, 1))

        polar,_,_ = ball
        bearing, range = polar
        ramp = np.linspace(0, 1, num=self.field_num + 1, endpoint=False)
        ramp = ramp[1:]
        base_field = np.concatenate([[1], np.flipud(ramp), ramp])
        ball_ang = self.ang_to_ind(bearing)
        rotated_field = np.roll(base_field, ball_ang, axis=0)
        return np.reshape(rotated_field, (-1, 1))

    #----------------------------------------------------------------
    #Creates a repulsion based on both the object location
    #and angle
    #----------------------------------------------------------------
    def create_repulsion_field(self,objects):
        robot_radius = 0.18/2
        obj_width = 2
        obj_fields = [np.zeros((self.field_length, 1))]
        for obj in objects:
            obj_pol,_,_ = obj
            obj_ang,obj_dist= obj_pol
            obj_ind = self.ang_to_ind(obj_ang)
            try:
                obj_width_ang = math.asin(self.config['objWidth'] / obj_dist)
            except ValueError:
                cur_field = np.ones((self.field_length, 1))
            else:
                cur_field = np.zeros((self.field_length,1))

                left_ind = self.ang_to_ind(obj_ang - obj_width_ang)
                right_ind = self.ang_to_ind(obj_ang + obj_width_ang)
            
                # print("creating obj from {} to {}".format(left_ind, right_ind))

                cur_field[left_ind:right_ind] = 1
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
        obj_fields = [np.zeros((self.field_length,1))]
        for obj in wall:
            obj_pol,_,_ = obj
            obj_ang,obj_dist= obj_pol
            obj_ind = self.ang_to_ind(obj_ang)
            obj_dist = min(2,obj_dist)
            cur_field = np.zeros((self.field_length,1))
            cur_field[obj_ind] = 1 - (obj_dist/2)
            obj_fields.append(cur_field)
        field_matrix = np.concatenate(obj_fields, axis=1)
        field_max = np.max(field_matrix, axis=1)
        return np.reshape(field_max, (-1,1))

    #----------------------------------------------------------------
    #Angle for trimming an angle into a format within 0-360,
    #useful for when an angle is <0 or >360
    #Note: Does NOT work for angle greater than 720 or less than
    #-360
    #----------------------------------------------------------------
    def clip_nang(self,nang):
        return max(-1, min(1, nang))


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
                self.last_ball_left = ball[0][0] < 0
                sub_state = 'moving_to_ball'
                #print(sub_state)
        elif (state == 2): #Have ball in dribbler, align with the goal
            if goal is None:
                sub_state = 'Looking for Goal'
                # print('substate set to:', sub_state)
            else:
                if self.config['shoot'] == 1 and goal[0][1] < self.config['shootDist'] and abs(goal[0][0]) < math.radians(self.config['shootAngle']):
                    sub_state = 'Shoot'
                else:
                    sub_state = 'Found Goal'
                self.last_goal_left = goal[0][0] < 0
                    # print('substate set to:', sub_state)
        return sub_state

    #----------------------------------------------------------------
    #Used for determining states.
    #----------------------------------------------------------------
    def determine_state(self,ball,objects,goal):
        if self.found_ball:
            return 2

        if ball is None:
            return 1
            
        polar, cartesian, reliable = ball
        bearing, range = polar

        if range >= self.config['distToGrab']:
            state = 1
        else:
            state = 2
            self.found_ball = True
        return state

    #----------------------------------------------------------------
    #Generate a random location for the ball
    #----------------------------------------------------------------
    def generate_virtual_ball_random_loc(self):
        #Create ball at random angle (0-360), at distance 0.5, 
        vball_abs_dist = 2
        vball_ang_rad = random.uniform(-self.max_ang, self.max_ang)
        vball_pol = (vball_ang_rad, vball_abs_dist)
        vball_cartesian = (vball_abs_dist*math.cos(vball_ang_rad), vball_abs_dist*math.sin(vball_ang_rad))
        vball = (vball_pol,vball_cartesian,True)
        return vball

 
    #----------------------------------------------------------------
    #Generate a ball infront of robot
    #----------------------------------------------------------------
    def generate_virtual_ball_infront(self):
        vball_abs_dist = 2
        vball_ang_rad = 0
        vball_pol = (vball_ang_rad,vball_abs_dist)
        vball_xy = (vball_abs_dist*math.cos(vball_ang_rad),vball_abs_dist*math.sin(vball_ang_rad))
        vball = (vball_pol,vball_xy,True)
        return vball
