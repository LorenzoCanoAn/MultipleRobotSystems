import numpy as np
import cv2
import tkinter as tk
from robot import *
# This class is supposed to handle at the same time the environment and the visualization of it.

class base:
    def __init__(self, position=np.array([0,0])):

        self.position=position

class environment:

    def __init__(self, i_params=None):
        self.robots={}

        self.init_params(i_params)

        self.base=base(position=np.array(self.params["base_pose"]))

        self.init_robots()

        self.init_plot()
        self.neighbours_information = {}



    def init_params(self, i_params):
        if type(i_params) == type(None):
            self.init_default_params()
        elif type(i_params) != type(dict()):
            raise Exception("The input parameters must be a dictionary")
        else:
            self.init_default_params()
            for key in i_params.keys():
                if key in self.params.keys():
                    self.params[key] = i_params[key]
                else:
                    print("The param {} does not belong to the environment class.".format(key))

    def compute_neighbours(self):
        self.neighbours_information={}
        for i in self.robots:

            self.neighbours_information[i]=[]
            if self.robots[i].state!='Active':

                continue
            for j in self.robots:
                if self.robots[j].state!='Active' or i==j:

                    continue

                if np.linalg.norm(self.robots[i].position[-1]-self.robots[j].position[-1])<self.robots[i].connection_radius:

                    self.neighbours_information[i].append(j)

    def init_default_params(self):
        self.params = {
            "env_height"  : 400,
            "env_width"   : 600,
            "nBots"     : 20,
            "base_pose" : [10,10],
            "window_name": "display"
        }

    def init_plot(self):
        root = tk.Tk()
        s_height = root.winfo_screenheight() * 0.9
        root.destroy()

        aspect_ratio = self.params["env_width"] / self.params["env_height"]
        self.scale = self.params["env_height"] / s_height

        self.w_size = np.array([int(s_height),int(s_height*aspect_ratio),3]);
        self.background = np.ones(self.w_size,dtype="uint8")*int(255/2)
        self.robot_layer = np.zeros(self.w_size,dtype="uint8")
        self.covered_layer = np.zeros(self.w_size,dtype="uint8")
        self.draw_env()

    def combine_layers(self):
        self.dispImage = self.background
        self.dispImage = overlay(self.dispImage, self.covered_layer  )
        self.dispImage = overlay(self.dispImage, self.robot_layer)

    def coord_to_pixel(self, point):
        """ Our coordinate system for the environtment will 
            go from 0 to env_height in the y axis and from 0 to r_widht in the x axis
            - point is a numpy array """

        x = int(point[0] / self.scale) # scale the point to put it on pixels
        y = int(self.w_size[0] - point[1]/ self.scale) # the y axis has to change
        return [x, y]

    def draw_base(self):
        pose = self.coord_to_pixel(np.array(self.params["base_pose"]))
        self.background = cv2.circle(self.background, (pose[0],pose[1]), 2, (200,0,0), -1)

    def draw_robots(self):
        self.robot_layer = np.zeros(self.w_size,dtype="uint8")
        for i in range(len(self.robots)):

            pose=self.coord_to_pixel(self.robots[i].position[-1])
            self.robot_layer = cv2.circle(self.robot_layer, (pose[0],pose[1]), 1, (0,200,0), -1)
            self.robot_layer = cv2.circle(self.robot_layer, (pose[0], pose[1]), self.robots[i].radius, (255, 255, 255))
            if self.robots[i].state=='Active':
                col=(0,255*self.robots[i].battery.battery/100,255-255*self.robots[i].battery.battery/100)
            elif self.robots[i].state=='Returning':
                col=(0,0,255)
            elif  self.robots[i].state=='Charging':
                col=(128,0,128)
            self.robot_layer=cv2.putText(self.robot_layer, str(round(self.robots[i].battery.battery,2)), (pose[0]+4, pose[1]+4),cv2.FONT_HERSHEY_SIMPLEX,0.4,color=col)

            self.covered_layer = cv2.circle(self.covered_layer, (pose[0], pose[1]), 3, (0, 200, 200), -1)
    
    def draw_env(self):
        self.draw_base()
        self.draw_robots()
        self.combine_layers()
        cv2.imshow(self.params["window_name"], self.dispImage)
    def init_robots(self):
        for i in range(self.params['nBots']):
            self.robots[i]=(robot(i,self.base.position,init_position=np.array([600*np.random.uniform(0,1),400*np.random.uniform(0,1)])))


    def update_robots(self):
        self.compute_neighbours()
        for i in self.robots:
            self.robots[i].update()
            self.robots[i].update_neighbours(self)





def overlay(image1, image2):
    mask = np.sum(image2,axis=2) == 0 # value of 1 where image 2 is 0
    d = mask.shape
    mask = np.reshape(mask,[d[0],d[1],-1])
    overlayed = np.multiply(image1,mask) + image2
    return overlayed
