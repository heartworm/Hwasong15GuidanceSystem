from motor_controller import MotorController
from communicator import Communicator
from tkinter import *
import time
import math
from itertools import zip_longest
from PIL import Image
from PIL import ImageTk


wheel_base = 0.2

#def translate_to_skid(d_rt,d_f):
    #Converts desired forward and rotational velocities into left/right wheel
    #Rotational velocities, I have yet to decide whether or not to also have the commands sent
    #from this function

    #d_rt = Desired rotational velocity, d_f = Desired forward velocity,
    #Will likely need to expand on this in the future

    #vl =

master = Tk()
w = Canvas(master, width=500, height=2000)
w.pack()

class bball(object):
    def __init__(self):
        self.root = Tk()
        #self.canvas = Canvas(self.root, width=400, height = 400)
        #self.canvas.pack()
        self.bball = w.create_oval(20, 260, 120, 360, outline='white',fill='orange')
        w.pack()
        self.x = 20
        self.y = 20

class robot(object):
    def __init__(self):
        robot.x = 100
        robot.y = 500
        robot.rot = 0

    def fnd_ball():
        print('grabbing coords')
        ball_loc = w.coords(animation.ballcanv)
        ball_x = ball_loc[0]
        ball_y = ball_loc[1]
        print("Found ball at x: %d, y: %d" % (ball_x , ball_y))
        return ball_x,ball_y
        
    def move(x,y):
        w.move(robot.canv,x,y)
        robot.x += x
        robot.y += y

    def create():
        robot.disp = Image.open('hwasong15.png')
        robot.tk = ImageTk.PhotoImage(robot.disp)
        robot.canv = w.create_image(robot.x, robot.y ,image = robot.tk) 

    def rotate(rot):
        print('rotating, old rot', robot.rot, 'rot = ', rot)
        robot.rot += rot
        robot.tk = ImageTk.PhotoImage(robot.disp.rotate(robot.rot))
        robot.canv = w.create_image(robot.x,robot.y,image = robot.tk) 

    def find():
        robot.loc = w.coords(robot.canv)
        print('finding robot, coords are:', robot.loc)

    def update():
        robot.dist_to_ball_pol() 
        robot.rotate(robot.ballang)
        pass

    def dist_to_ball_cart():
        robot.find()
        ball_x, ball_y = robot.fnd_ball()
        print('creating cartesian distance, ball_x = ', ball_x, 'y',ball_y,'robotx',robot.loc[0],'roboty', robot.loc[1])
        robot.dist_x = ball_x - robot.loc[0]
        robot.dist_y = ball_y - robot.loc[1]
        print('distance to ball x,y = ', robot.dist_x, robot.dist_y)

    def dist_to_ball_pol():
        robot.dist_to_ball_cart()
        robot.ballang =  math.atan2(robot.dist_y,robot.dist_x)
        robot.ball_dist_true = math.sqrt(math.pow(robot.dist_x,2) + math.pow(robot.dist_y,2))
        print('ballang  = ',math.degrees(robot.ballang), 'dist to ball', robot.ball_dist_true)
        print('robot rot is: ', robot.rot)

class window():
    def create():
        T = w.create_text(15,15,text = "test")

    def update():
        T = w.create_text(15,40,text = robot.rot)
        #T.pack()
        #T.insert(END, "Just a text Widget\nin two lines\n")

def animation():
        
        #animation.bball_disp = w.create_oval(20, 260, 120, 360, outline='white',fill='orange')

        animation.balldisp = Image.open('ball.png')
        animation.balltk = ImageTk.PhotoImage(animation.balldisp)
        animation.ballcanv = w.create_image(300, 100 ,image = animation.balltk) 


        #robot_disp = w.create_polygon((0, 0, 0, 100, 100, 100, 100, 0), fill="red")
        #print('robot created at' ,w.coords(robot_canv))
        #print('robot moved to' ,w.coords(robot_canv))

        track = 0
        robot.create()
        window.create()
        window.update()
        while True:
            x = 5
            y = 0
            if track == 0:
                for i in range(0,51):
                    time.sleep(0.025)
                    w.move(animation.ballcanv, 5, 0)
                    robot.update()
                    window.update()
                    #T = Text(w, height=2, width=30)
                    #T.pack()
                    #T.insert(END, "Just a text Widget\nin two lines\n")
                    w.update()
                track = 1 
                
            if track == 1:
                for i in range(0,51):
                    time.sleep(0.025)
                    w.move(animation.ballcanv, 0, 5)
                    robot.update()
                    w.update()
                track = 2 
                
            if track == 2:
                for i in range(0,51):
                    time.sleep(0.025)
                    w.move(animation.ballcanv, -5, 0)
                    robot.update()
                    w.update()
                track = 3
            else: 
                for i in range(0,51):
                     time.sleep(0.025)
                     w.move(animation.ballcanv, 0, -5)
                     robot.update()
                     w.update()
                track = 0
                w.update()
                    

#bball()
robot()
animation()





mainloop()
