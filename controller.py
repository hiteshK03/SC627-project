import numpy as np
import random
import time
import matplotlib.pyplot as plt
from model import VehicleModel
from potentials import Potentials
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

class MPCFramework():

	def __init__(self, state, timestep, constants):
		self.sample_iter = 5
		self.k = 2
		self.t = 0
		self.t_list=[]
		self.timestep = timestep
		self.constants = constants
		self.state = state
		self.F_xt_max = 13000
		self.F_xt_min = -24800
		self.F_xt_list = []
		self.delta_max = 0.2
		self.delta_min = -0.2
		self.delta_list = []
		self.Q_y = 0.2
		self.Q_u = 0.01
		self.R_f = 2e-9
		self.R_d = 100
		self.X_0 = 20
		self.Y_0 = 5
		self.u_obj = 5
		self.Y_lane = 3.8
		self.F_xt = [0]*self.k
		self.delta = [0]*self.k
		self.state_list = []
		self.state_list.append(self.state)	
		self.model = VehicleModel(self.state, self.timestep, self.constants)

	def costFunction(self, F_xt, delta):
		cf = 0
		u_des, Y_des = self.model.stateCopy[1:3]
		self.model.updateModel(np.array([F_xt, delta]))
		u, Y = self.model.stateCopy[1:3]
		self.obs = Potentials(self.X_0, self.Y_0, self.u_obj, self.model.stateCopy[0], self.model.stateCopy[2], self.model.stateCopy[1])
		self.lane = Potentials(self.X_0, self.Y_lane, None, self.model.stateCopy[0], self.model.stateCopy[2], self.model.stateCopy[1])
		U = self.obs.noCrossPotential() + self.lane.laneChangePotential()
		cf = U + self.Q_y*(Y-Y_des)**2 + self.Q_u*(u-u_des)**2 + self.R_f*(F_xt)**2 + self.R_d*(delta)**2
		return cf

	def optimization(self):
		min_cost = 1e10
		cost = 0
		F_xt, delta = [0]*self.k, [0]*self.k
		for _ in range(self.sample_iter):
			F_xt[0] = (random.random()*(self.F_xt_max-self.F_xt_min))+self.F_xt_min
			for _ in range(self.sample_iter):
				F_xt[1] = (random.random()*(self.F_xt_max-self.F_xt_min))+self.F_xt_min
				for _ in range(self.sample_iter):
					delta[0] = (random.random()*(self.delta_max-self.delta_min))+self.delta_min
					for _ in range(self.sample_iter):
						delta[1] = (random.random()*(self.delta_max-self.delta_min))+self.delta_min
						for i in range(self.k):
							cost+=self.costFunction(F_xt[i], delta[i])
						if cost < min_cost:
							min_cost = cost
							self.F_xt = F_xt
							self.delta = delta
		self.update()
		time.sleep(0.1)

	def update(self):
		self.model.updateModel(np.array([self.F_xt[0], self.delta[0]]))
		self.F_xt_list.append(self.F_xt[0])
		self.delta_list.append(self.delta[0])
		self.t+=1
		self.t_list.append(self.t)
		self.check_state()	
		self.state_list.append(self.model.stateCopy)

	def check_state(self):
		cap = [300.0, 68.750,  7, 17.000,  360.0,  37.0]
		for i, (a, b) in enumerate(zip(self.model.stateCopy, cap)):
			if a>b:
				self.model.stateCopy[i] = b

	def plot(self):
		ax = plt.axes(ylim=(-1, 1))
		plt.plot(self.t_list,self.delta_list)
		plt.show()

def main():
	###3 random test
	state = np.array([0.,1.,0.,0.,0.,0.])
	timestep = 0.1
	constants = np.array([2271, 1.421, 1.434, 4600, 132000, 136000, 10400, 10800, 24800])

	mpc = MPCFramework(state, timestep, constants)
	for i in range(60):
		mpc.optimization()
	mpc.plot()


if __name__ == '__main__':
	main()
