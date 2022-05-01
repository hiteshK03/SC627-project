from re import X
import numpy as np

class Potentials():

	def __init__(self, X, Y, u_o):
		self.a_i = self.U_saf = 1       # intensity done
		self.b_i = None       			# shape done
		self.U_acc = 10					# present done
		self.U_unc = 2					# present done
		self.U_lma = 2					# present done

		self.X_0 = None					# present done
		self.Y_0 = None					# present done
		self.T_0 = 0.25					# present done
		self.a_n = 1					# present done
		self.a_max = 9					# present done
		self.a_q = None					# done
		self.u = 2						# present done
		self.u_o = u_o					# from obs p_done
		self.theta = 0.45				# not sure, maybe from vehicle
		self.delta_u = None				# done
		self.delta_v = None				# done
		self.Da = 0.5					# present done
		self.X = X						# from obs
		self.Y = Y						# from obs
		cond1 = self.u_o*self.u < 0 and self.u > 0 and self.X < self.X_0
		cond2 = self.u_o*self.u < 0 and self.u < 0 and self.X > self.X_0
		if cond1 or cond2:
			self.delta_u = self.u-self.u_o
		else:
			self.delta_u = 0			
		self.Xs = self.X_0 + self.u*self.T_0 + (self.delta_u**2/(2*self.a_n))
		self.Ys = self.Y_0 + (self.u*np.sin(self.theta)+self.u_o*np.sin(self.theta))*self.T_0 + (self.delta_v**2/(2*self.a_n))
		self.s_i = self.sd(self.X/self.Xs, self.Y/self.Ys)
		X_c, Y_c = self.delta_u**2/(2*self.a_max), self.delta_v**2/(2*self.a_max)
		self.s_c = self.sd(X_c, Y_c)

	def sd(self, x, y):
		return np.sqrt((x-self.X_0)**2 + (y-self.Y_0)**2)
	
	def noCrossPotential(self):
		self.b_i = np.log(self.U_saf/self.U_acc)/np.log(self.s_c)
		return self.a_i/(self.s_i**self.b_i)

	def CrossPotential(self):
		self.b_i = np.log(self.U_saf/self.U_unc)/np.log(self.s_c)
		return self.a_i*np.exp(-1*self.b_i*self.sd(self.X/self.Xs, self.Y/self.Ys))

	def laneChangePotential(self, X, Y):
		self.a_q = self.U_lma/(self.Da**2)
		if self.sd(X,Y) < self.Da:
			return self.a_q*(self.sd(X,Y)-self.Da)**2
		else:
			return 0