import numpy as np
import matplotlib.pyplot as plt

class environment:
    def __init__(self, params):
        self.init_params(params)


    def init_params(self, params):
        if type(params) != type(dict()):
            raise Exception("The input parameters must be a dictionary")
        else:
            
