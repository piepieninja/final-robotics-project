#!/usr/bin/env python

import roslib
import rospy
import struct
import math
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

squares = []
real_guys = [] # the ones we want to actually send around in ros

# from opencv examples
def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

# from opencv examples
def find_squares(img):
    img = cv.GaussianBlur(img, (5, 5), 0)
    global squares
    squares = []
    for gray in cv.split(img):
        for thrs in xrange(0, 255, 26):
            if thrs == 0:
                bin = cv.Canny(gray, 0, 50, apertureSize=5)
                bin = cv.dilate(bin, None)
            else:
                _retval, bin = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            bin, contours, _hierarchy = cv.findContours(bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv.arcLength(cnt, True)
                cnt = cv.approxPolyDP(cnt, 0.02*cnt_len, True)
                if len(cnt) == 4 and cv.contourArea(cnt) > 1000 and cv.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    if max_cos < 0.1:
                        squares.append(cnt)
    return squares

def is_red(x,y):
    global redness
    red_sum = 0
    divisor = 0    
    red_sum += cv_image[x  ,y  ][0]*1.2
    red_sum += cv_image[x-1,y  ][0]
    red_sum += cv_image[x+1,y  ][0]
    red_sum += cv_image[x-1,y-1][0]
    red_sum += cv_image[x-1,y+1][0]
    red_sum += cv_image[x+1,y-1][0]
    red_sum += cv_image[x+1,y+1][0]
    red_sum += cv_image[x  ,y-1][0]
    red_sum += cv_image[x  ,y+1][0]
    redness = float(red_sum)/float(9.0)
    #print redness
    if redness > 110:
        return True
    else:
        return False    


    
def update_depth(data):
    pub = rospy.Publisher('sense_dude_square_loc', Pose, queue_size=10)
    global real_guys
    count = 0
    for s in squares:
        x1 = s[0][0]
        x2 = s[2][0]
        y1 = s[0][1]
        y2 = s[2][0]
        x_mid = (x1 + x2)/2
        y_mid = (y1 + y2)/2
        # uint16 is depth stored in mm
        b = []        
        b.append(data.data[y_mid*data.step + x_mid])
        b.append(data.data[y_mid*data.step + (x_mid + 1)])
        barray = bytearray(b)
        depth = struct.unpack('>H', barray)
        dist = depth[0]/10
        if dist > 0 and dist < 200 and is_red(x_mid,y_mid):
            print str(count) + " | " + str((x_mid,y_mid)) + " | "+ str(dist) + "cm |" + str(cv_image[x_mid,y_mid]) + " | redness: " + str(redness) 
            fov = 69.4
            d_pix = fov/1920.0
            foc = 1.93 # mm
            half_max = (1920.0/2.0)*d_pix
            x_d = d_pix*x_mid - half_max # gets the angle off center that poits to the square
            x_u = math.sin(math.radians(x_d))
            y_u = math.cos(math.radians(x_d))
            z_u = 0.0
            #
            dist_x = dist*x_u
            dist_y = dist*y_u
            dist_z = dist*z_u
            #
            location = Pose()
            location.position.x = dist_x
            location.position.y = dist_y
            location.position.z = dist_z
            pub.publish(location)
            #print location
        count += 1

def sense_symbol(data):
    #print 'oof'
    bridge = CvBridge()
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "rgb8")
    except CvBridgeError as e:
        print(e)
    wot = find_squares(cv_image)
    #print wot
    # do shape detection here

def export_3d_loc(data):
    global real_guys
    pub = rospy.Publisher('sense_dude_square_loc', Pose, queue_size=10)
    for loc in real_guys:
        x = loc[0]
        y = loc[1]
        b = data.data[y*data.row_step + x*data.point_step]
        barray = bytearray(b)
        z = struct.unpack('>B', barray)
        print z
        #print "point_step: " + str(data.point_step) + " | row_step: " + str(data.row_step)
        #print data.width
        #print data.height
        #print loc
    real_guys = []
    
def listener():
        
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sense_dude', anonymous=True)

    depth_maybe = "camera/aligned_depth_to_color/image_raw"
    color_maybe = "camera/color/image_raw"
    point_maybe = "camera/depth_registered/points"
    
    rospy.Subscriber(depth_maybe, Image, update_depth)
    rospy.Subscriber(color_maybe, Image, sense_symbol)
    #rospy.Subscriber(point_maybe, PointCloud2, export_3d_loc)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    listener()
