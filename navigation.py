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
w = Canvas(master, width=1000, height=2000)
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
        robot.rot = -90
        robot.speed = 3

    def fnd_ball():
        #print('grabbing coords')
        ball_loc = w.coords(animation.ballcanv)
        ball_x = ball_loc[0]
        ball_y = ball_loc[1]
        #print("Found ball at x: %d, y: %d" % (ball_x , ball_y))
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
        #print('rotating, old rot', robot.rot, 'rot = ', rot)
        robot.rot += rot
        #I have a bunch of corrections here on the rotate function, don't as me why, I don't know either
        robot.tk = ImageTk.PhotoImage(robot.disp.rotate(-robot.rot - 90))
        robot.canv = w.create_image(robot.x,robot.y,image = robot.tk) 

    def find():
        robot.loc = w.coords(robot.canv)
        #print('finding robot, coords are:', robot.loc)

    def update():
        robot.dist_to_ball_pol() 
        robot.rotate(robot.ballang)
        robot.translate_mov_to_grid()
        #robot.move(1,1)
        pass

    def dist_to_ball_cart():
        robot.find()
        ball_x, ball_y = robot.fnd_ball()
        #print('creating cartesian distance, ball_x = ', ball_x, 'y',ball_y,'robotx',robot.loc[0],'roboty', robot.loc[1])
        robot.dist_x = ball_x - robot.loc[0]
        robot.dist_y = ball_y - robot.loc[1]
        print('distance to ball x,y = ', robot.dist_x, robot.dist_y)

    def dist_to_ball_pol():
        robot.dist_to_ball_cart()
        robot.ballang =  math.degrees(math.atan2(robot.dist_y,robot.dist_x)) - robot.rot
        robot.ball_dist_true = math.sqrt(math.pow(robot.dist_x,2) + math.pow(robot.dist_y,2))
        #print('ballang  = ',robot.ballang, 'dist to ball', robot.ball_dist_true)
        #print('robot rot is: ', robot.rot)

    def translate_mov_to_grid():
        #robot.to_mov_y = -1 * math.ceil(math.cos(robot.rot))
        #robot.to_mov_x = 1 * math.ceil(math.cos(robot.rot))


        robot.to_mov_y = -1 
        robot.to_mov_x = 1
        if robot.dist_x < 0:
            robot.to_mov_x = -robot.to_mov_x
            print('inverting x')

        if robot.dist_y > 0:
            robot.to_mov_y = -robot.to_mov_y
            print('invertin y')

        robot.move(robot.speed * robot.to_mov_x,robot.speed * robot.to_mov_y)
        #print('to_mov_x: ',robot.to_mov_x,'y',robot.to_mov_y)
        pass

class window():
    def create():
        window.box = w.create_text(10, 10, anchor="nw")
        w.itemconfig(window.box, text="this is the text")
        window.box2 = w.create_text(10, 40, anchor="nw")

    def update():
        w.delete(window.box,0,END)
        w.delete(window.box2,0,END)

        window.box = w.create_text(10, 10, anchor="nw")
        window.box2 = w.create_text(10, 40, anchor="nw")

        w.itemconfig(window.box, text="Robot rot:")
        w.insert(window.box,12,int(robot.rot))

        w.itemconfig(window.box2, text="ballang")
        w.insert(window.box2,12,int(math.degrees(robot.ballang)))
        pass

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
        robot.update()
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
            window.update() 
            if track == 1:
                for i in range(0,100):
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
                    window.update()
                    w.update()
                track = 3
            else: 
                for i in range(0,100):
                     time.sleep(0.025)
                     w.move(animation.ballcanv, 0, -5)
                     robot.update()
                     window.update()
                     w.update()
                track = 0
                w.update()
                    

#bball()
robot()
animation()





mainloop()
