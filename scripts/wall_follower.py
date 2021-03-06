#!/usr/bin/env python

# general imports for all python nodes
import rospy
import sys

# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan, Joy # joystick and laser scanner msgs

class Wall():
    #this is the variable that holds the angle of the turning: set by 
    angle = 0
    right = True #flag that determines which wall to follow
    death = False
    speed=2
    #initializes the last three errors. I do this to allow the getSteeringCmd method to find the de and "previous" e
    errors = [0,0,0]
    
    '''
    Finds the integral of the error in the floating frame
    
    '''
    def getIntegral(self):
        return sum(self.errors)

    '''
    Finds the error given a goal, list, and slice
    
    '''
    def getError(self, goal, L, begin, end):
        return goal - (min(L[begin:end]))
        
    '''
    Finds the angle to steer at using the full left and full right parameters
    '''
    def getSteeringCmd(self, fullLeft, fullRight):
        Kp = float(sys.argv[1])
        Kd = float(sys.argv[2])
        de =  self.errors[-3] - self.errors[-1]
        u = Kp * self.errors[-1] + Kd * de
        return u
    
    '''
    Call back in charge of reading laser scan data and responding by setting the drive angle and death if too close
    '''
    def laserSteeringCallback(self, msg):
        #Set the floating frame time (over the last 40 scans which is about 1 second
        if(len(self.errors)>39):
            self.errors.pop(0)
          
          
        #get the laser information.
        if self.right: #right
            
            error = self.getError(0.45, msg.ranges, 200, 540) #TODO tweak range
            
            self.errors.append(error)
            
            self.angle = self.getSteeringCmd(-1, 1)+.1 #add to align the wheels to u=0 being straight
            
            self.death = min(msg.ranges[525:555]) < 0.5 #safety controller

	    #if wall in front
	    if( max(msg.ranges[(270 - WALL_FIND_ANGLE)/2 *1081/270:(270 + WALL_FIND_ANGLE)/2*1081/270]) < WALL_TURN_DISTANCE):
		self.angle = 1.0
            
        else: #left 
            
            error = self.getError(.45, msg.ranges, 540,900) #TODO tweak range
            
            self.errors.append(error)
            
            self.angle = -self.getSteeringCmd(-1, 1)+.1 #reverse cause going opposing dir
            
            self.death = min(msg.ranges[525:555]) < 0.5 #safety controller

	    	    #if wall in front
	    if( max(msg.ranges[(270 - WALL_FIND_ANGLE)/2 *1081/270:(270 + WALL_FIND_ANGLE)/2*1081/270]) < WALL_TURN_DISTANCE):
		self.angle = -1.0
            
        
        self.drive_cmd.drive.steering_angle = self.angle
            
        #safety controller
        if self.death:
            print "Dead"
            self.drive_cmd.drive.speed = -0.1
        elif self.drive_cmd.drive.speed > 0:
            print self.angle
            self.drive_cmd.drive.speed = self.speed

        self.drive.publish(self.drive_cmd) # post this message
        
    '''
    callback for controlling which wall to follow
    
    A goes left
    B goes right
    '''
    def sideSwitchCallback(self, msg):
        
        #0 corresponds to 'A' and 1 corresponds to 'B' on game controller
        if (msg.buttons[1]): #right (B)
            self.right = True
        elif (msg.buttons[0]): #left (A)
            self.right = False
        
    '''
    Runs when the node dies
    '''
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        #always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
        rospy.sleep(1)
    
    '''
    Initializes the class and runs the drive publisher
    '''
    def __init__(self):
        #setup the node
        rospy.init_node('wall_bang', anonymous = False)
        rospy.on_shutdown(self.shutdown)
        
        # output messages/sec (also impacts latency)
        rate = 10 
        
        # node specific topics (remap on command line or in launch file)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 5)
        
        #sets the subscriber
        rospy.Subscriber('scan', LaserScan, self.laserSteeringCallback)
        rospy.Subscriber('vesc/joy', Joy, self.sideSwitchCallback)
        
        
        # fill out fields in ackermann steering message (to go straight)
        self.drive_cmd = AckermannDriveStamped()
        self.drive_cmd.drive.speed = self.speed
        self.drive_cmd.drive.steering_angle = self.angle
        
        rospy.spin()
        self.drive.publish(AckermannDriveStamped())
        
if __name__ == '__main__':
    Wall()
