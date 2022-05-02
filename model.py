import numpy as np
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

class VehicleModel():

	def __init__(self, state, timestep, constants):

		# state (x) = [X, u, Y, v, theta, r]
		self.X = state[0]
		self.u = state[1]
		self.Y = state[2]
		self.v = state[3]
		self.theta = state[4]
		self.r = state[5]
		self.stateCopy = state.copy()

		# control (u_c) = [F_xt, delta]
		self.F_xt = None
		self.delta = None

		# slipside angles = [alpha_f, alpha_r]
		self.alpha_f = None
		self.alpha_r = None

		# constants = [m, lf, lr, Iz, Cf, Cr, F_yf_max, F_yr_max]
		self.m = constants[0]
		self.lf = constants[1]
		self.lr = constants[2]
		self.Iz = constants[3]
		self.Cf = constants[4]
		self.Cr = constants[5]
		self.F_yf_max = constants[6]
		self.F_yr_max = constants[7]
		self.F_xt_max = constants[8]

		# tireForces = [F_yf, F_yr]
		self.F_yf = None
		self.F_yr = None

		self.dt = timestep

	def updateModel(self, control):

		self.F_xt = np.sign(control[0]) * min(abs(control[0]), self.F_xt_max)
		self.delta = control[1]

		self.alpha_f = self.delta - ( (self.v + self.lf * self.r) / self.u )
		self.alpha_r = - ( ( self.v - self.lr * self.r ) / self.u )

		self.F_yf = np.sign(self.alpha_f) * min(abs(self.Cf * self.alpha_f), self.F_yf_max)
		self.F_yr = np.sign(self.alpha_r) * min(abs(self.Cr * self.alpha_r), self.F_yr_max) 
		# print(self.F_yf, self.F_yr)

		self.stateCopy[0] = self.stateCopy[0] + (self.u * np.cos(self.theta) - self.v * np.sin(self.theta)) * self.dt
		self.stateCopy[1] += ((1 / self.m) * (self.F_xt + self.v * self.r)) * self.dt
		self.stateCopy[2] += (self.v * np.cos(self.theta) + self.u * np.sin(self.theta)) * self.dt
		self.stateCopy[3] += ((1 / self.m) * (self.F_yf + self.F_yr - self.u * self.r)) * self.dt
		self.stateCopy[4] += (self.r) * self.dt
		self.stateCopy[5] += ((1 / self.Iz) * (self.lf * self.F_yf - self.lr * self.F_yr))

		self.X = self.stateCopy[0]
		self.u = self.stateCopy[1]
		self.Y = self.stateCopy[2]
		self.v = self.stateCopy[3]
		self.theta = self.stateCopy[4]
		self.r = self.stateCopy[5]
		
		# print(self.stateCopy)

def main():
	###3 random test
	state = np.array([0.,1.,0.,0.,0.,0.])
	control = np.array([1000.,0.01])
	timestep = 0.1
	constants = np.array([2271, 1.421, 1.434, 4600, 132000, 136000, 10400, 10800, 24800])

	model = VehicleModel(state, timestep, constants)
	for i in range(1000):
		model.updateModel(control)
		control[0] += 100


if __name__ == '__main__':
	main()