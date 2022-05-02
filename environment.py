import matplotlib.pyplot as plt
import time
import random
from collections import deque
import numpy as np
from simulation import Trajectory
from controller import MPCFramework
from model import VehicleModel

class Env():

    def __init__(self):
        self.obs_path = []
        self.veh_path = []
        
    def create(self):
        ax = plt.axes(xlim=(-5, 350), ylim=(0, 7))
        plt.plot([150, 155], [0.2, 3.8], c='k')
        plt.plot([-5, 150], [0.2, 0.2], c='k')
        plt.plot([-5, 350], [6.8, 6.8], c='k')
        plt.plot([155, 155], [3.8, 3.8], c='k', linestyle='--')
        plt.plot([155, 350], [3.8, 3.8], c='k')
        # plt.show()

def main():
	###3 random test
    E = Env()
    E.create()
    A = Trajectory()
    A.plot()
    # state = np.array([0.,1.,0.,0.,0.,0.])
    # timestep = 0.1
    # constants = np.array([2271, 1.421, 1.434, 4600, 132000, 136000, 10400, 10800, 24800])

    # mpc = MPCFramework(state, timestep, constants)
    # for i in range(2):
    #     mpc.optimization()

if __name__=="__main__":
    main()
