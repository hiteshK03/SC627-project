import matplotlib.pyplot as plt
import time
import random
from collections import deque
import numpy as np
from controller import MPCFramework

class Trajectory():

    def __init__(self):
        self.num_obs = 3
        self.obs = []
        # self.obs_path = []
        # self.veh_path = []
        self.x = -4
        self.y = 5
        self.x_v = 0
        self.y_v = 2
        self.t = 0
        self.delta_y = self.y - self.y_v
        # print(self.x)

    def obj_path(self):
        self.x += 15
        self.y = 5
        time.sleep(0.1)

    def veh_path(self):
        eps = random.random()/10
        thr = 30
        if self.x_v < 120:
            self.x_v += 15
            self.y_v = 2 + eps
        elif self.x_v >=120 and self.x_v<120+thr:
            self.x_v += 4.5
            self.y_v += self.delta_y/(thr/2.5) + eps
        else:
            self.x_v += 15
            self.y_v = 5 + eps

    def plot(self):
        x1 = [0]*40
        y1 = [0]*40
        x2 = [0]*40
        y2 = [0]*40
        ax = plt.axes(xlim=(-5, 350), ylim=(0, 7))

        line1, = plt.plot(x1, y1, marker='s', color='r')
        line2, = plt.plot(x2, y2, marker='>', color='b')
        plt.ion()
        plt.ylim([0,8])
        plt.show()
        state = np.array([0.,1.,0.,0.,0.,0.])
        timestep = 0.1
        constants = np.array([2271, 1.421, 1.434, 4600, 132000, 136000, 10400, 10800, 24800])

        mpc = MPCFramework(state, timestep, constants)

        while self.x_v<350:
            x1.append(self.x)
            y1.append(self.y)
            x2.append(self.x_v)
            y2.append(self.y_v)
            mpc.optimization()
            self.veh_path()
            self.obj_path()
            line1.set_data(x1, y1)
            line2.set_data(x2, y2)
            plt.draw()
            print(x1[0])
            # i += 1
            self.t+=1
            time.sleep(0.1)
            plt.pause(0.0001)

if __name__=="__main__":
    A = Trajectory()
    A.plot()
