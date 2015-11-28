from multiprocessing import Pool, Lock, Process, Value
from functools import partial

class weightParticle():
	"""	
	def thread_weight(self, scan, pf, pose):

		print("POOP")

		weight = pf.sensor_model.get_weight(scan, pose)
		
		if weight > maxim.value:
			lock.acquire()
			total.value += weight
			maxim.value = weight
			
		else:
			lock.acquire()
			total.value += weight
			
		lock.release()
				
		return weight
	"""	
	def p(self, po):
		lock.acquire()
		lock.release()
		return po+1

	def init(self, l, t, m):
		global lock
		global total
		global maxim
		total = t
		maxim = m
		lock = l
	
	def __init__(self, scan, pf, particles):
		lc = Lock()
		tot = Value('d', 0.0)
		maxi = Value('d', 0.0)
		p = Pool(processes = 8, initializer = self.init, initargs = (lc, tot, maxi,))
		
		#For testing
		k = [1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8]
		
		#self.particleWeights = p.map(partial(self.thread_weight, scan, pf), particles)

#		q = p.map(self.p, k)

		self.particleWeights = p.map(self.p, k)

#		self.particleWeights = p.map(self.p, particles)
#		self.totalWeight = tot.value
#		self.maximumWeight = maxi.value
		if self.maximumWeight < 8:
			self.exploded = True
		else:
			exploded.self = False
