from motor_controller import MotorController
from communicator import Communicator
from tkinter import *
import time
import math
from itertools import zip_longest
from PIL import Image
from PIL import ImageTk


wheel_base = 0.2


master = Tk()
w = Canvas(master, width=500, height=2000)
w.pack()

class Ball(object):
    def __init__(self, canvas):
        self.shape = canvas.create_oval(20, 260, 120, 360, outline='white',fill='orange')
        self.x = 20
        self.y = 20
        self.canvas = canvas


class Robot(object):
    def __init__(self, canvas):
        self.x = 100
        self.y = 500
        self.rot = 0
        self.canvas = canvas
        self.img = Image.open('hwasong15.png')
        self.tkImg = Image.open()

    def move(self, x, y, rot):
        if rot != self.rot:
            self.tkImg = ImageTk.PhotoImage(self.img.rotate(rot))
            self.shape = self.canvas.create_image(self.x, self.y, image = self.tkImg)

        self.canvas.move(self.shape, x, y)
        self.x = x
        self.y = y
        self.rot = rot

    def find(self):
        Robot.loc = w.coords(Robot.canv)
        print('finding robot, coords are:', Robot.loc)

    def update(self):
        Robot.dist_to_ball_pol()
        Robot.rotate(Robot.ballang)
        pass

    def dist_to_ball_cart(self):
        Robot.find()
        ball_x, ball_y = Robot.fnd_ball()
        print('creating cartesian distance, ball_x = ', ball_x, 'y', ball_y,'robotx', Robot.loc[0], 'roboty', Robot.loc[1])
        Robot.dist_x = ball_x - Robot.loc[0]
        Robot.dist_y = ball_y - Robot.loc[1]
        print('distance to ball x,y = ', Robot.dist_x, Robot.dist_y)

    def dist_to_ball_pol(self):
        Robot.dist_to_ball_cart()
        Robot.ballang =  math.atan2(Robot.dist_y, Robot.dist_x)
        Robot.ball_dist_true = math.sqrt(math.pow(Robot.dist_x, 2) + math.pow(Robot.dist_y, 2))
        print('ballang  = ', math.degrees(Robot.ballang), 'dist to ball', Robot.ball_dist_true)
        print('robot rot is: ', Robot.rot)

def animation():
        
        #animation.bball_disp = w.create_oval(20, 260, 120, 360, outline='white',fill='orange')

        animation.balldisp = Image.open('ball.png')
        animation.balltk = ImageTk.PhotoImage(animation.balldisp)
        animation.ballcanv = w.create_image(300, 100 ,image = animation.balltk) 


        #robot_disp = w.create_polygon((0, 0, 0, 100, 100, 100, 100, 0), fill="red")
        #print('robot created at' ,w.coords(robot_canv))
        #print('robot moved to' ,w.coords(robot_canv))

        track = 0
        Robot.create()
        while True:
            x = 5
            y = 0
            if track == 0:
                for i in range(0,51):
                    time.sleep(0.025)
                    w.move(animation.ballcanv, 5, 0)
                    Robot.update()
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
                    Robot.update()
                    w.update()
                track = 2 
                
            if track == 2:
                for i in range(0,51):
                    time.sleep(0.025)
                    w.move(animation.ballcanv, -5, 0)
                    Robot.update()
                    w.update()
                track = 3
            else: 
                for i in range(0,51):
                     time.sleep(0.025)
                     w.move(animation.ballcanv, 0, -5)
                     Robot.update()
                     w.update()
                track = 0
                w.update()
                    

#bball()
Robot()
animation()





mainloop()
