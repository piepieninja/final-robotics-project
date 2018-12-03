#!/usr/bin/env python

import roslib
import rospy
import struct
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#def __init__(self):
#    self.bridge = CvBridge()

squares = []

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
    for val_x in range(x-2,x+2):
        for val_y in range(y-2,y+2):
            # give a boost to the mid pixel
            if val_x == x and val_y == y:
                red_sum += cv_image[val_x,val_y][0]/10
            red_sum += cv_image[val_x,val_y][0]
            divisor += 1
    redness = float(red_sum)/float(divisor)
    #print redness
    if redness > 95:
        return True
    else:
        return False
    
            
def update_depth(data):
    #print 'yeet'
    #print squares
    count = 0
    for s in squares:
        x1 = s[0][0]
        x2 = s[2][0]
        y1 = s[0][1]
        y2 = s[2][0]
        x_mid = (x1 + x2)/2
        y_mid = (y1 + y2)/2
        
        #print data.height
        #print data.width
        #print data.encoding
        #print data.step
        
        # uint16 is depth stored in mm
        b = []        
        b.append(data.data[y_mid*data.step + x_mid])
        b.append(data.data[y_mid*data.step + (x_mid + 1)])
        barray = bytearray(b)
        #print barray
        #depth = int.from_bytes(b, byteorder='big', signed=False)
        depth = struct.unpack('>H', barray)
        dist = depth[0]/10
        #print (depth[0]/10)
        
        if dist > 20 and dist < 1000 and is_red(x_mid,y_mid):
            print str(count) + " | " + str((x_mid,y_mid)) + " | "+ str(dist) + "cm |" + str(cv_image[x_mid,y_mid]) + " | redness: " + str(redness)  
        count += 1
        
        
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def sense_symbol(data):
    #print 'oof'
    bridge = CvBridge()
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    wot = find_squares(cv_image)
    #print wot
    # do shape detection here
        
def listener():
        
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sense_dude', anonymous=True)

    depth_maybe = "camera/aligned_depth_to_color/image_raw"
    color_maybe = "camera/color/image_raw"
    
    rospy.Subscriber(depth_maybe, Image,  update_depth)
    rospy.Subscriber(color_maybe, Image,  sense_symbol)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    listener()
