#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import * 
from ackermann_msgs.msg import AckermannDriveStamped

class Avoid():
	def __init__(self, Kp, Kd):
	    	self.error = 0 #how far the angle is from desired
	      	self.last_error = 0 
		
		self.Kp = Kp
		self.Kd = Kd
		
		self.speed = 0
		self.steering_angle = 0
		
		self.safety_threshold = 0.5

		rospy.init_node("avoid_node") 
		
		self.sub = rospy.Subscriber('scan', LaserScan, self.callback) #subscribing to node "scan" which has LaserScan type messages
	      	self.pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped)
		
		rospy.on_shutdown(self.shutdown)
        
	'''
	you give the callback to the publisher. Whenever they are ready, 
	they call this function and shit happens
	ex: sending a message on Facebook, and instead of checking for a 
	response every 5 min, Facebook will notify you
	**does this by sending a callback to your computer
	'''

	def callback(self, scan):
		center_index = self.findLargestSpace(scan.ranges[180:900], 1.5) #180 deg; in front of the bot
        
		self.error = 540 - center_index
		self.steering_angle = self.Kp * self.error + self.Kd * (self.error - self.last_error)
		self.last_error = self.error
		
		#safety
		if(min(scan.ranges[525:555]) < self.safety_threshold):
			print "Killed"
			self.speed = -0.1
		else:
			self.speed = 2.0
		
		msg = AckermannDriveStamped()
		msg.drive.speed = self.speed
		msg.drive.steering_angle = self.steering_angle
		
		self.pub.publish(msg)
        
        	
	'''
	Takes L as the list of ranges to look for the largest free space in
	Takes threshold as the distance to look for points in
	Returns an int that is the index of the middle of the largest free space
	'''
	def findLargestSpace(self, L, threshold):
	      	center = 0
	    	largestSpace = 0
	    	newL = []
	    
		#mark all legal areas as True
	    	for i in L:
			newL.append(i > threshold)
		    
		#go through and find the largest free space

		

	    	for i in range(len(newL)):
			if(newL[i]):
				count = 0
		    		while newL[i]:
					count += 1
		        		i += 1
					if(i>=len(newL)):
						break
		    		if(count > largestSpace):
		        		largestSpace = count
		        		center = (count / 2) + i
	    	return center # a point out of 1081
      
	def shutdown(self):
		self.pub.publish(AckermannDriveStamped())
		rospy.sleep(1)

if __name__ == '__main__':
	Avoid(sys.argv[1], sys.argv[2])
	rospy.spin()
    
