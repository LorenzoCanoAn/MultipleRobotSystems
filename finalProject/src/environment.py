import numpy as np
import cv2
import tkinter as tk

# This class is supposed to handle at the same time the environment and the visualization of it.
class environment:

    def __init__(self, i_params=None):
        self.init_params(i_params)
        self.init_plot()

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

    
    def init_default_params(self):
        self.params = {
            "r_height"  : 500,
            "r_width"   : 500,
            "nBots"     : 20,
            "base_pose" : np.reshape(np.array([10,10],dtype="double"),[2,1]),
            "window_name": "display"
        }

    def init_plot(self):
        root = tk.Tk()
        s_width = root.winfo_screenwidth()
        s_height = root.winfo_screenheight()
        root.destroy()


        self.w_size = int((s_width*(s_height>s_width) + s_height*(s_height<=s_width))*0.8)
        self.background = np.zeros([self.w_size, self.w_size, 3],dtype="uint8")
        self.robot_layer = np.zeros([self.w_size, self.w_size, 3],dtype="uint8")
        self.covered_layer = np.zeros([self.w_size, self.w_size, 3],dtype="uint8")
        self.combine_layers();
        cv2.imshow(self.params["window_name"], self.dispImage)

    def combine_layers(self):
        self.dispImage = self.background
        self.dispImage = overlay(self.dispImage, self.background)
        self.dispImage = overlay(self.dispImage, self.robot_layer)


def overlay(image1, image2):
    mask = np.sum(image2,axis=2) == 0
    d = mask.shape
    mask = np.reshape(mask,[d[0],d[1],-1])
    overlayed = np.multiply(image1,mask) + image2
    return overlayed