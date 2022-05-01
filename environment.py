import matplotlib.pyplot as plt
import time
import random
from collections import deque
import numpy as np
from simulation import Trajectory

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
        # plt.axline((-5, 0), (150, 0), c='k', mew=2)
        # plt.axline((150, 0), (155, 3.8), c='k', mew=2)
        # plt.axline((155, 3.8), (350, 3.8), c='k', mew=2)
        # plt.axline((-5, 7.8), (350, 7.8), c='k', mew=2)
        # plt.axline((-5, 3.8), (155, 3.8), c='k', linestyle='--', mew=2)
        # plt.show()

if __name__=="__main__":
    E = Env()
    E.create()
    A = Trajectory()
    A.plot()
