import numpy as np
import random
import time
from model import VehicleModel
from potentials import Potentials
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

class MPCFramework():

	def __init__(self, state, timestep, constants):
		self.sample_iter = 5
		self.k = 2
		self.t = 0
		self.timestep = timestep
		self.constants = constants
		self.state = state
		self.F_xt_max = 13000
		self.F_xt_min = -24800
		self.delta_max = 0.2
		self.delta_min = -0.2
		self.Q_y = 0.2
		self.Q_u = 0.01
		self.R_f = 2e-9
		self.R_d = 100
		self.F_xt = [0]*self.k
		self.delta = [0]*self.k
		self.state_list = []
		self.state_list.append(self.state)	
		self.model = VehicleModel(self.state, self.timestep, self.constants)

	def costFunction(self, F_xt, delta):
		cf = 0
		u_des, Y_des = self.state[1:3]
		self.model.updateModel(np.array([F_xt, delta]))
		u, Y = self.state[1:3]
		self.obs = Potentials(20, 5, 5, self.state[0], self.state[2], self.state[1])
		self.lane = Potentials(20, 3.8, None, self.state[0], self.state[2], self.state[1])
		U = self.obs.noCrossPotential() + self.lane.laneChangePotential()
		cf = U + self.Q_y*(Y-Y_des)**2 + self.Q_u*(u-u_des)**2 + self.R_f*(F_xt)**2 + self.R_d*(delta)**2
		return cf

	def optimization(self):
		min_cost = 0
		cost = 0
		F_xt, delta = [0]*self.k, [0]*self.k
		for _ in range(self.sample_iter):
			F_xt[0] = (random.random()%(self.F_xt_max-self.F_xt_min+1))+self.F_xt_min
			for _ in range(self.sample_iter):
				F_xt[1] = (random.random()%(self.F_xt_max-self.F_xt_min+1))+self.F_xt_min
				for _ in range(self.sample_iter):
					delta[0] = (random.random()%(self.delta_max-self.delta_min+1))+self.delta_min
					for _ in range(self.sample_iter):
						delta[1] = (random.random()%(self.delta_max-self.delta_min+1))+self.delta_min
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
		self.state_list.append(self.state)

def main():
	###3 random test
	state = np.array([0.,1.,0.,0.,0.,0.])
	timestep = 0.1
	constants = np.array([2271, 1.421, 1.434, 4600, 132000, 136000, 10400, 10800, 24800])

	mpc = MPCFramework(state, timestep, constants)
	for i in range(2):
		mpc.optimization()


if __name__ == '__main__':
	main()