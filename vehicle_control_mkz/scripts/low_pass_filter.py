#!/usr/bin/env python
import pdb
import numpy as np
import matplotlib.pyplot as plt

#### Define a Base Class
class Filter(object):
	#### BY DEFAULT LETS JUST CREATE A FIR MOVING AVERAGE
	def __init__(self,consNum=[0.25,0.75],consDen=1.0, dcGain=1.0):
		self.consNum = consNum
		self.consDen = consDen
		self.DC = dcGain

		if isinstance(consNum,list):
			self.lenNum = len(consNum)
		else:
			self.lenNum = 0
		if isinstance(consDen,list):
			self.lenDen = len(consDen)
		else:
			self.lenDen = 0

		self.rawValues = [0.0]*self.lenNum
		self.filtValues=[0.0]*self.lenDen
	
	def update_filter(self,valNow):
		len_ = self.lenDen
		if len_ >= 1:
			self.filtValues[1:len_]  = self.filtValues[0:len_-1]
			self.filtValues[0] = valNow
		else:
			self.filtValues = valNow

	def update_raw(self,valNow):
		len_ = self.lenNum
		if len_ >= 1:
			self.rawValues[1:len_]  = self.rawValues[0:len_-1]
			self.rawValues[0] = valNow
		else:	
			self.rawValues = valNow
		

#### Define Child Classes which will inherit the Base Class and its methods from above
class LinearFilter(Filter):
	def __init__(self,consNum=0.05,consDen=[1,-0.95],dcGain=1.0):
		#### We use the super keyword to avoid directly referring to the base class...
		#### In python3.0 it's a lot easier to use than in python 2.7
		super(LinearFilter, self).__init__(consNum,consDen,dcGain)

	def update_filter(self,valNow):
		# Update Raw Value list
		super(LinearFilter,self).update_raw(valNow)
		# Compute Weighted Filtered Value
		temp=0.0
		if self.lenNum == 0:
			temp = self.consNum*self.rawValues		
		elif self.lenNum == 1:
			temp = self.consNum[0]*self.rawValues[0]
		else:
			for i in range(self.lenNum):
				temp += self.consNum[i]*self.rawValues[i]
		

		# SUM THE PREVIOUS FILTERED VALUE
	
		if self.lenDen == 0:
			temp = temp/self.consDen
		elif self.lenDen == 1:
			temp = temp/self.consDen[0]
		else:
			for j in range(self.lenDen-1):
				# filtValue is nonrecursive jth spot is n-1
				temp -= self.consDen[j+1]*self.filtValues[j]
			temp = temp/self.consDen[0]

		# DC GAIN
		temp *= self.DC

		# Latest filtered value 
		super(LinearFilter,self).update_filter(temp)	

		return temp

	
#### Nonlinear filter test
#### Same as previous
class NonLinearFilter(Filter):
		
	def __init__(self,order,cons=None):
	
		#### We use the super keyword to avoid directly referring to the base class...
		#### In python3.0 it's a lot easier to use than in python 2.7
		super(NonLinearFilter, self).__init__(order,cons)


		#### Keep track of the past 100 raw values to estimate std
		if len(self.values)<100:
			#### PREPEND
			self.rawValues.insert(0,valNow)
		else:
			self.rawvalues[1:100]  = self.rawValues[0:99]
			self.rawValues[0] = valNow

	def __nonlinear_gain(self,valNow,R):
		# To prevent underestimating of the variance, use ddof=1
		std_ = np.std(self.values,ddof=1)
		if abs(valNow) <= R*std_:
			P = 1.0 - abs(valNow/(R*std_))
		else:
			P = 0.0
		return P 

	def update_filter(self,valNow,R=3.5):
		super(NonLinearFilter,self).update_filter(valNow)
		P = self.__nonlinear_gain(valNow,R) #Tune R value
		self.cons[0] = 1-P


		#### Assign remaining 'filter gains' equally
		len_ = len(self.cons)
		if P <= 0.0000000001:
			vals_ = 0.0
		else:
			vals_= P/(len_-1.0)
		self.cons[1:len_]  = [vals_]*(len_-1)
	

		# Compute Weighted Filtered Value
		temp=0.0
		for i in range(len(self.cons)):
			temp += self.cons[i]*self.filtValues[i]
		
		# Posteriori Update: Latest filtered value 
		self.filtValues[0] = temp		
		return temp

		
#### EXAMPLE USAGE ####
if __name__=="__main__":
	# y[n] = 1/a0 * (sum{bi*x[n-i] - sum{aj*y[n-j])
	# LinearFilter([b0, b1z^-1,..., bN z^-N],[a0, a1z^-1,...aM z^-M],DC)
	# DC GAIN: https://dsp.stackexchange.com/questions/11411/gain-of-fir-iir-filters
	lowpassFilter = LinearFilter(0.05,[1,-0.95]) # Simple First order filter			
	#lowpassFilter = LinearFilter([1, 3, 3, 1],[1 ,-2.0286, 1.4762, -0.3714],1.0/9.0)	#Butterworth 3rd order
	#NLlowpassFilter = NonLinearFilter(4.0)

	#### CREATE SINE WAVE WITH NORMALLY DISTRIBUTED NOISE	
	t = np.linspace(0,2*np.pi,1000)
	y = np.sin(t)
	noise = np.random.normal(0,0.15,1000)
	yn = y+noise
	yn_filt = [0.0]*1000
	#yn_filt_nonL = [0.0]*1000
	for i in range(1000):
		yn_filt[i] = lowpassFilter.update_filter(yn[i])
		#yn_filt_nonL[i] = NLlowpassFilter.filter(yn[i],3.5)
	'''
	plt.figure
	plt.plot(t,y)
	plt.plot(t,yn)	
	plt.plot(t,yn_filt)
	#plt.plot(t,yn_filt_nonL)
	plt.legend(['y','yn','yn_filt','yn_filt_nonL'])
	plt.show()
	'''

	


	
		
		
