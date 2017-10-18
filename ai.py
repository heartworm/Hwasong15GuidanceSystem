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
search_counter = 0
virt_ball = 180

#TODO: Finish search algorithm
#Change objects to obstacles
#----------------------------------------------------------------
#Primary State controller function, handles calling all 
#other functions. RIP OOP
#---------------------------------------------------------------
def state_controller(ball, objects, goal,wall):
    #Set up parameters
    # print(ball, objects, goal, wall)
    objects = [object for object in objects if object[1][0] is not None]
    wall = [w for w in wall if w[1][0] is not None]
    sub_state = ''
    global search_counter
    global virt_ball 

    state = determine_state(ball,objects,goal)
    # print("Rolling with state number:",state,"Counter value is:",search_counter)

    desired_heading = 90
    velocity = 0 #Units for Velocity is m/s

    if (state == 1): #State 1 (grabbing ball)
        sub_state = determine_sub_state(ball,objects,goal, state)
        # print('state is:',state,'sub_state is:',sub_state)
        if (sub_state == 'finding_ball'):
            #Generate a random ball location
            if search_counter >= 20 and search_counter <= 30:
                # print("Spinning on point, search counter:",search_counter)
                search_counter +=1
                #INSERT CODE TO PERFORM 360 DEGREE SPIN
                desired_heading = 270
                velocity = 0

            elif search_counter >= 30 or search_counter == 0:
                virt_ball = generate_virtual_ball()
                search_counter = 1
                # print('Searching for ball, placing virtual ball at:',virt_ball)

            else:
                # print("Moving towards virtual ball at heading:", virt_ball)
                search_counter +=1
                attract_field = create_attraction_field(virt_ball)
                repulse_field = create_repulsion_field(objects) + create_repulsion_field_wall(wall)
                sum_field = attract_field - repulse_field 
                desired_heading = sum_field.argmax(axis = 0) - 180
                velocity = 0.3
                # print('heading towards: ',desired_heading, "At velocity:",velocity)

        elif (sub_state == 'moving_to_ball'):
            search_counter = 0
            attract_field = create_attraction_field(ball)
            repulse_field = create_repulsion_field(objects) + create_repulsion_field_wall(wall)
            sum_field = attract_field - repulse_field 
            desired_heading = sum_field.argmax(axis = 0) - 180
            velocity = 0.3
            # print('heading towards: ',desired_heading)
            # print('----------------------------')

    elif (state == 2): #Aligning ball with goal
        if (sub_state ==  'Looking for Goal'):
            #Create a virtual goal location, move towards it
            #Then do a spin to try and spot it.
            if search_counter >= 20 and search_counter <= 30:
                # print("Spinning on point, looking for goal, search counter:",search_counter)
                search_counter +=1
                #INSERT CODE TO PERFORM 360 DEGREE SPIN
                desired_heading = 270
                velocity = 0

            elif search_counter >= 30 or search_counter == 0:
                virt_goal = generate_virtual_ball()
                search_counter = 1
                # print('Searching for Goal, placing virtual goal at:',virt_goal)

            else:
                # print("Moving towards virtual goal at heading:", goal)
                search_counter +=1
                attract_field = create_attraction_field(virt_goal)
                repulse_field = create_repulsion_field(objects)
                sum_field = attract_field - repulse_field 
                desired_heading = sum_field.argmax(axis = 0) - 180
                velocity = 0.3
                # print('heading towards: ',desired_heading, "At velocity:",velocity)


        elif(sub_state == 'Found Goal'):
            search_counter = 0
            attract_field = create_attraction_field(goal)
            repulse_field = create_repulsion_field(objects) + create_repulsion_field_wall(wall)
            sum_field = attract_field -repulse_field
            desired_heading = sum_field.argmax(axis = 0) - 180
            velocity = 0.3
        
        print('----------------------------')
    # plot_state(ball,objects,goal,wall,desired_heading,virt_ball)
    return (math.radians(desired_heading - 90),velocity)
    #May not work right if we are subctracting 90 from the heading
    #If shit doesn't work, start by changing this
    #return (math.radians(desired_heading),velocity)


#----------------------------------------------------------------
#Convers the given locations, and returns angles relative to the 
#robot's reference fram
#----------------------------------------------------------------
def return_angles(ball_loc, goal_loc,obj_loc,robot_rot):
    ball_ang = math.degrees(math.atan2(ball_loc[1],ball_loc[0]))
    goal_ang = math.degrees(math.atan2(goal_loc[1],goal_loc[0]))
    obj_ang = math.degrees(math.atan2(obj_loc[1],obj_loc[0]))
    #print('ball_loc: ', ball_loc, 'ball angle: ' ,ball_ang)
    #print('goal loc:',goal_loc,' goal angle: ', goal_ang)
    #print('obstacle loc:', obj_loc, ' obj angle', obj_ang)
    return ball_ang, goal_ang, obj_ang


#----------------------------------------------------------------
#Detrmines whether or not the robot can "see" the ball.
#Only useful in the simulation
#----------------------------------------------------------------
def determine_can_see(ball_ang,goal_ang,obj_ang):
    see_ball = abs(ball_ang) <= 45
    see_goal = abs(goal_ang) <= 45
    see_obj = abs(obj_ang) <= 45

    return see_ball, see_goal, see_obj

#----------------------------------------------------------------
#Creates an attraction field, based entirely on the relative
#angle given to it
#----------------------------------------------------------------
def create_attraction_field(ball):
    ball_ang_tup = ball[0]
    ball_ang = math.degrees(ball_ang_tup[0])
    #Create field map
    ball_ang += 180
    ball_ang = int(ball_ang)
    # print('ball angle is:',ball_ang)
    att_field = np.zeros((360,1))
    #print(att_field.shape)
    att_field[ball_ang] = 1
    gradient = 1/180

    for angle in range(1,178): 
        att_field[clip_angle_360(ball_ang - angle)] = 1 - (angle * gradient)
        att_field[clip_angle_360(ball_ang + angle)] = 1 - (angle * gradient)
    return att_field

#----------------------------------------------------------------
#Creates a repulsion based on both the object location
#and angle
#----------------------------------------------------------------
#
def create_repulsion_field(objects):
    #Initial set up
    robot_radius = 0.18/2
    rep_field = np.zeros((360,1))
    obj_width = 2 * robot_radius + 0.1
    numberOfObjects = len(objects)
    rep_field = np.zeros((360,1))
    for i in range (0,numberOfObjects):
        # print("Creating repulse_field for object number:",i ,"Out of: ",numberOfObjects)
        obj = objects[i]
        objPol = obj[0]
        # print("objpol: ",objPol)
        obj_dist = objPol[1]
        obj_ang = math.degrees(objPol[0])
        # print("----------------------")
        # print("Object angle:",obj_ang,"distance: ",obj_dist)
        # print("----------------------")
        if(obj_dist > 0.28):
            obj_width_ang = int(math.degrees(math.asin(obj_width/obj_dist)))
        else:
            # print('applying fix to unfuck the asin error')
            obj_width_ang = 90

        obj_ang_array = 180 + obj_ang
        # print('obj width angle is: ',obj_width_ang)

        obj_effect = max(0, 1 - min(1, (obj_dist - robot_radius*2)))
        # print('creating repulsion_field, effect of the field is: ',obj_effect, 'angle is:', obj_ang)
        for angle in range(0,int(obj_width_ang)):
            rep_field[clip_angle_360(int(obj_ang_array - angle))] += max(rep_field[obj_width_ang - angle],obj_effect)
            rep_field[clip_angle_360(int(obj_ang_array + angle))] += max(rep_field[obj_width_ang + angle],obj_effect)
    #print(rep_field)
    return rep_field


#----------------------------------------------------------------
#Creates a repulsion based on both the wall location
#and angle
#----------------------------------------------------------------
def create_repulsion_field_wall(wall):
#     #Initial set up
     robot_radius = 0.18/2
     rep_field = np.zeros((360,1))
     wall_width = 0.001
     field_amplitude = 0.2
     numberOfPoints = len(wall)
     for i in range (0,numberOfPoints):
         # print("Creating repulse_field for object number:",i ,"Out of: ",numberOfPoints)
         wall_point = wall[i]
         wallPol = wall_point[0]
         # print("wallpol: ",wallPol)
         wall_dist= wallPol[1]
         wall_ang = math.degrees(wallPol[0])
         # print("----------------------")
         # print("Object angle:",wall_ang,"distance: ",wall_dist)
         # print("----------------------")
         if(wall_dist > 0.28):
             wall_width_ang = int(math.degrees(math.asin(wall_width/wall_dist)))
         else:
             # print('applying fix to unfuck the asin error')
             wall_width_ang = 90
#
         wall_ang_array = 180 + wall_ang
         # print('wall width angle is: ',wall_width_ang)
         wall_effect = max(0, 1 - min(1, (wall_dist - robot_radius*2)))
         # print('creating repulsion_field, effect of the field is: ',wall_effect, 'angle is:', wall_ang)
         for angle in range(0,int(wall_width_ang)):
             rep_field[clip_angle_360(int(wall_ang_array - angle))] += max(rep_field[wall_width_ang - angle],wall_effect) * field_amplitude
             rep_field[clip_angle_360(int(wall_ang_array + angle))] += max(rep_field[wall_width_ang + angle],wall_effect) * field_amplitude
      #print(rep_field)
     return rep_field



#----------------------------------------------------------------
#Convers the desired heading and ball_angle into commands for the
#Motor controller to use
#----------------------------------------------------------------
#TODO: UPDATE TO ALLOW INTEGRATION INTO THE OMNI DRIVE SYSTEM
def motor_controller(desired_heading, ball_ang):
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
def clip_angle_360(angle):
    if(angle < 0):
        angle = angle + 360;
    if( angle >= 360):
        angle = angle - 360;
    return angle


#----------------------------------------------------------------
#Used for determining substates.
#----------------------------------------------------------------
#def determine_sub_state(see_b, see_g, see_obj, state):
def determine_sub_state(ball,objects,goal, state):
    if (state == 1): #Looking for the ball, is it visible?
        if ball is None:
            sub_state = 'finding_ball'
            # print(sub_state)
        else:
            sub_state = 'moving_to_ball'
            # print(sub_state)
    elif (state == 2): #Have ball in dribbler, align with the goal
        if goal is None:
            sub_state = 'Looking for Goal'
        else:
            sub_state = 'Found Goal'
    return sub_state

#----------------------------------------------------------------
#Used for determining states.
#----------------------------------------------------------------
def determine_state(ball,objects,goal):
    
    if ball is None:
        return 1
        
    polar, cartesian, reliable = ball
    bearing, range = polar
    # print("range", range)
    if range >= 0.1:
        ##if the ballxz is greater than these
        #Then is is not seated in the dribbler, that must be corrected
        #Also, if the length of the ball tuple is 0, then obviously
        #we should do something about that
        state = 1
    else:
        state = 2
    return state

#----------------------------------------------------------------
#Generate a random location for the ball
#----------------------------------------------------------------
def generate_virtual_ball():
    #Create ball at random angle (0-360), at distance 0.5, 
    vball_abs_dist = 0.5
    vball_ang_deg = random.randint(0, 360) - 180
    vball_ang_rad = math.radians(vball_ang_deg)
    vball_pol = (vball_ang_rad,vball_abs_dist)
    vballxy = (vball_abs_dist*math.cos(vball_ang_rad),vball_abs_dist*math.sin(vball_ang_rad))
    vball = (vball_pol,vballxy,True)
    return vball

 
#
# #----------------------------------------------------------------
# #Plot the state
# #----------------------------------------------------------------
# def plot_state(ball,objects,goal,wall,heading,vball):
#
#     heading_vect_length = 1
#     headingx = heading_vect_length * math.cos(math.radians(90 - heading))
#     headingy = heading_vect_length * math.sin(math.radians(90 - heading))
#     #print(wallpointscart)
#
#
#     #Format for py.plot: (x1x2x3),(y1y2y3)
#     #print(objectInLoopx,objectInLoopz)
#     #print(wallinloopx)
#     print(headingx,headingy)
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



'''
    if (ball_loc[0] == 0):
        #Execute state 1 (Will have it's own dedicated py file
        print('State 1, searching for ball')
        pass
    #State 2 (Found ball, align with the dribbler
    elif (ball_loc[0] != 0) :
        pass
        #Execute state 2
   #State 3 (Have ball in dribbler, but don't have the goal located )
    elif (goal_loc[0] == 0):
        pass
        #Execute state 3
    #State 4 (Have ball in dribbler, have goal located, go for gold!!)
    elif (goal_loc[0] != 0):
        pass


        '''
