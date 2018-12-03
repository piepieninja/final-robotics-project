#! /usr/bin/env python
import rospy
from threading import Thread, Lock
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from pid import PID

class MazeSolver:

    def __init__(self):
	print("inside init maze")
        rospy.init_node('MazeSolverNode')
        self.rate = rospy.Rate(10)

        rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
	print("subscribed to topics maze")
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 5)

        self.vel = Twist()
        self.laser = None
        self.odom = None

        # set up wall detection
        self.minLaserValues = 40            # amount of laserspoints that must be next to each other
        self.intervalEpsilon = 1.5          # upper- and lower-bound
        self.mutex2 = Lock()

        # set up loop detection
        self.knownPoints = []
        self.epsilonAroundPoints = 0.4     # circle around the saved points
        self.timeoutForDetection = 12       # delay in seconds

        # set up wall follower
        self.leftHand = True            # True, if the robot uses the left sensor for wall follow
        self.laserIndex = 359           # left laser index
        self.turnSpeed = -0.3
        self.distanceToWall = 0.5

       # distance from the wall to the robot and min distance for obstacles detection
        self.angle = 1.5707963          # around 90 degree
        self.minLasersSide = [150,170]  # lasers used for basic obstacles detection
        self.mutex = Lock()
        # set up PID
        kp = 6
        ki = 2
        kd = 1.5
        outMin = -0.5
        outMax = 0.5
        iMin = -0.5
        iMax = 0.5
        self.pid = PID(kp, ki, kd, outMin, outMax, iMin, iMax)
        # actual drive state (initial: WallDetection)
        self.driveState = "WallDetection"

    def odom_callback(self, odom_msg):
        self.odom = odom_msg

    def laserscan_callback(self, laser_msg):
	ranges =[]

        for i in range(270,360):
		ranges.append(laser_msg.ranges[i])
		ranges.append(laser_msg.ranges[i])
	for i in range(0,90):
		ranges.append(laser_msg.ranges[i])
		ranges.append(laser_msg.ranges[i])
	for i in range(len(ranges)):
        if (ranges[i] != 0.0):      
            ranges[i] = math.sqrt((0.08)**2 + (ranges[i])**2 - 2*0.08*ranges[i]*math.fabs(math.cos(1.5707963+(i*0.008726))))
        else:
            ranges[i] = 30
        self.laser = laser_msg
	self.laser.ranges = ranges

    def startSolver(self):
        rospy.loginfo("start Maze Solver Node")
        # if the robot uses the right sensor for wall follow
        if(not self.leftHand):
            self.laserIndex = 0
            self.minLasersSide = [190, 210]
            self.turnSpeed *= (-1)

        while not rospy.is_shutdown():
            if(self.laser and self.odom): # laser and odom data arrived from callback
                # append the origin to the known points
                if(len(self.knownPoints) == 0):
                    self.knownPoints.append([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, rospy.Time.now().to_sec()])
		#print("in while loop at ", rospy.Time.now().to_sec())
		#print("laser ranges in 170 to 190 are ", self.laser.ranges[170:190])
		#print("laser ranges in ", self.minLasersSide[0], " to ", self.minLasersSide[1], " are ", self.laser.ranges[self.minLasersSide[0]:self.minLasersSide[1]])
                # take the min laser value in front of the robot (simple obstacles detection)
                actMinLaserValue = min(min(self.laser.ranges[170:190], self.laser.ranges[self.minLasersSide[0] : self.minLasersSide[1]]))
		print("in while loop and min laser value detected front is ", actMinLaserValue)
                if(self.driveState == "WallDetection"):
		    print("in Wall Detection ", rospy.Time.now().to_sec())
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0
                    self.wallDetection(actMinLaserValue)
                elif(self.driveState == "driveToWall"):
		    print("Now drive to the farthest wall at distance : ", self.distanceToWall, "  min laser range detected is: ", actMinLaserValue)
                    if(self.mutex.locked() == False):
			print("obstacle in front of the robot or wall arrived??????")
                        # obstacle in front of the robot or wall arrived
                        if(actMinLaserValue <= self.distanceToWall):
			    print("arrived : save the actual position used for loop detection", self.odom.pose.pose.position.x,self.odom.pose.pose.position.y, rospy.Time.now().to_sec())
                            # save the actual position used for loop detection
                            self.knownPoints.append([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y, rospy.Time.now().to_sec()])
                            self.vel.linear.x = 0.0
                            self.vel.angular.z = 0.0
			    print("rotate angle by 90 and make state as wall follow")
                            self.rotate_angle(self.angle, self.turnSpeed)
                            self.driveState = "WallFollow"
                        else:
			    print("Not arrived: DRIVE DRIVE DRIVE!!!!!!!!")
                            self.vel.linear.x = 0.4
                            self.vel.angular.z = 0.0
                elif(self.driveState == "WallFollow"):
                    if(self.mutex.locked() == False):
			print("Let me follow this wall")
                        # check known points (loop detection)
                        for i in range(0, len(self.knownPoints)):
                            if(self.odom.pose.pose.position.x - self.epsilonAroundPoints <= self.knownPoints[i][0] <= self.odom.pose.pose.position.x + self.epsilonAroundPoints and
                            self.odom.pose.pose.position.y - self.epsilonAroundPoints <= self.knownPoints[i][1] <= self.odom.pose.pose.position.y + self.epsilonAroundPoints and
                            self.knownPoints[i][2] + self.timeoutForDetection <  rospy.Time.now().to_sec()):
                                rospy.loginfo("Loop detected")
                                self.driveState = "WallDetection"
                        self.wallFollower(actMinLaserValue)
                self.velPub.publish(self.vel)
            self.rate.sleep()
        rospy.spin()

    def wallDetection(self, actMinLaserValue):
	print("in Wall Detection function with min laser : ", actMinLaserValue, "  and Distance to wall is : ", self.distanceToWall)
        self.mutex2.acquire()
        try:
            if(actMinLaserValue < self.distanceToWall):
		print("in Wall Detection the distance is less so ROTATE by 90: ", rospy.Time.now().to_sec())
                self.rotate_angle(self.angle, self.turnSpeed)
            else:
		print("in Wall Detection Not near to any obstacle, search for a wall to follow by checking the points: ", rospy.Time.now().to_sec())
                # search for a wall to follow
                arrValues = []
                i = 0
                # do wall detection
                while(i < len(self.laser.ranges)):
                    valCounter = 0
                    for k in range(i+1, len(self.laser.ranges)):
                        # check if actual value is in interval
                        if(self.laser.ranges[i] - self.intervalEpsilon) <= self.laser.ranges[k] <= (self.laser.ranges[i] + self.intervalEpsilon):
                            valCounter += 1
                        else:
                            if(valCounter > self.minLaserValues):
                                arrValues.append([self.laser.ranges[i], i, k])
                            valCounter = 0
                            i = k
                        if(k == len(self.laser.ranges) - 1):
                            i = len(self.laser.ranges)

                # turn the robot to the wall (use the wall with the max distance to the robot)
                getLaserIndex = 180;
		print("in Wall Detection got arrValues : ", arrValues)
                if(arrValues):
                    getLaserIndex = int(math.floor((max(arrValues)[1] + max(arrValues)[2]) / 2))
		    print("in Wall Detection Laser Index becomes : ", getLaserIndex) 
                if(getLaserIndex <= 175 or getLaserIndex >= 185):
		    print("in Wall Detection Laser Index is in front face : ", getLaserIndex)
                    tmpAngle = self.laser.angle_min + (getLaserIndex * self.laser.angle_increment)
		    print("in Wall Detection Change index to angle : ", tmpAngle)
                    if(getLaserIndex < 170): # rotation to the right
		   	print("in Wall Detection Laser Index to the right i.e. less than 170 : ", getLaserIndex)
                        self.rotate_angle(abs(tmpAngle), -0.3)
                    else:   # rotation to the left
		    	print("in Wall Detection Laser Index to the left i.e. more than 170 : ", getLaserIndex)
                        self.rotate_angle(abs(tmpAngle), 0.3)
                # wait until the robot has stopped the rotation
                rospy.sleep(1.5)
		print("in Wall Detection After Rotation: from state ", self.driveState, " change to dive to wall.")
                self.driveState = "driveToWall"

        finally:
            self.mutex2.release()

    def wallFollower(self, actMinLaserValue):
	print("In wall follower function")
        pidValue = self.pid.pidExecute(self.distanceToWall, self.laser.ranges[self.laserIndex])
        if(self.leftHand):
            pidValue = pidValue * (-1)
        # obstacle in front of the robot
        if(actMinLaserValue < self.distanceToWall):
	    print("Avoid obstacle")
            self.rotate_angle(self.angle, self.turnSpeed)
        elif(pidValue == 0):
            self.vel.linear.x = 0.4
            self.vel.angular.z = 0.0
        elif(pidValue != 0):
            self.vel.linear.x = 0.15
            self.vel.angular.z = pidValue

    # simple rotation function by the relativ angle in rad
    def rotate_angle(self, angle, speed):
	print("Lets rotate by ", angle)
        self.mutex.acquire()
        try:
            angular_speed = speed
            relative_angle = angle
            self.vel.linear.x = 0.0
            self.vel.angular.z = angular_speed

            # setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_angle = 0
            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)

            # forcing the robot to stop after rotation
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0
            self.velPub.publish(self.vel)
            # fixes issue #6 (Weird Driving at Wall Following)
            self.pid.resetValues()
        finally:
            self.mutex.release()

if __name__ == "__main__":
        maze_solver = MazeSolver()
	maze_solver.startSolver()
