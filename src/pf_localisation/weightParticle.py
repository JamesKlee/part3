from multiprocessing import Pool, Lock, Process, Value
from functools import partial

class weightParticle():
	
	def thread_weight(scan, pf, pose)
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
		
        self.particleWeights = p.map(partial(self.thread_weight, scan, pf), particles)
		self.totalWeight = tot.value
        self.maximumWeight = maxi.value