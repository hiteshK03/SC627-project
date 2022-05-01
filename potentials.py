import numpy as np

class Potentials():

	def __init__(self):
		self.a_i = 0.1       # intensity
		self.b_i = 0.1       # shape
		self.X_0 = 0.1
		self.Y_0 = 0.1
		self.T_0 = 0.1
		self.a_n = 2
		self.u = 2
		self.u_o = 1
		self.theta = 0.45
		self.delta_u = 3
		self.delta_v = 3
		self.Da = 2
		self.Xs = self.X_0 + self.u*self.T_0 + (self.delta_u**2/(2*self.a_n))
		self.Ys = self.Y_0 + (self.u*np.sin(self.theta)+self.u_o*np.sin(self.theta))*self.T_0 + (self.delta_v**2/(2*self.a_n))

	def sd(self, X, Y):
		return np.sqrt((X-self.X_0)**2 + (Y-self.Y_0)**2)
	
	def noCrossPotential(self, X, Y):
		return self.a_i/(self.sd(X/self.Xs, Y/self.Ys))**self.b_i

	def CrossPotential(self, X, Y):
		return self.a_i*np.exp(-1*self.b_i*self.sd(X/self.Xs, Y/self.Ys))

	def laneChangePotential(self, X, Y):
		if self.sd(X,Y) < self.Da:
			return self.a_q*(self.sd(X,Y)-self.Da)**2
		else:
			return 0