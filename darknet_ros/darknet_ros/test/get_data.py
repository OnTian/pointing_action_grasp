#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes  # type of Yolo result
import numpy as np
import time

global my_data, key
my_data = []; key = 1
def listener():
    global key
    global my_data

    time.sleep(2)  # have to set a sleep,"make sure" get a result data 
    key = 1
    if my_data != []:  # "double check" for getting the data indeed
        print(my_data)

        
def callback(data):
    global key, my_data
    if key == 1:
        my_data = data  # yolo result by array[frame, bounding_boxes]
        key = 0

if __name__ == '__main__':
    rospy.init_node('the_node', anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)  # Yolo topic
    rate = rospy.Rate(40)
    listener()
    rospy.spin()

'''#----------initial value-----------
global M, C, pic_M, pic_C
M,C ,pic_M, pic_C= 0.0, 0.0, [0.0,0.0], [0.0,0.0]
global image_x, arm_x, image_y, arm_y, key, MID_X, MID_Y
key = 1; MID_X = 0; MID_Y = 0
image_x = [85,    168,  551,  742, 1093, 1220, 1215, 1215, 1212, 1125,  737,  549,   204,   81,   84,   84,   490,   664,   838,   577,   749,   663,   576,   748,   490,   834]
arm_x   = [-30, -25.5, -5.5,  5.0, 23.5, 29.5, 29.5, 29.5,   25,   25,  4.5, -5.5, -24.5,  -31,  -30,  -30, -10.8,  -2.3,   6.2, -6.55,  1.95,  -2.3, -6.65,  1.85, -10.9,   6.0]
image_y = [70,     69,   92,   94,   69,   74,  263,  412,  602,  601,  570,  571,   597,  595,  406,  260,   108,   110,   111,   197,   199,   284,   369,   370,   454,   457]
arm_y   = [59.8, 59.8, 58.3, 58.3, 59.3, 59.3, 49.3, 40.8, 30.8, 30.8, 32.8, 32.8,  31.8, 31.8, 41.8, 49.8, 54.62, 54.61, 54.61, 50.56, 50.56, 46.31, 42.26, 42.16, 38.01, 37.91]
global action, my_data
action = 0; my_data = []

#----------content of Swich for python----------
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args:
            self.fall = True
            return True
        else:
            return False

#----------get a data from Yolo just once "key = 1"----------
def callback(data):
    global key, my_data
    if key == 1:
        my_data = data  # yolo result by array[frame, bounding_boxes]
        key = 0

#----------some thing use of data----------
def means():
    global MID_X, MID_Y, my_data    
    for item in my_data.bounding_boxes:  # bounding_boxes[Class(Str), probability(float), xmax(float), xmin(float), ymax(float), ymin(float)]
        if item.Class == 'lemon':  # able to forcus the particular object
	    xmin = float(item.xmin)
	    xmax = float(item.xmax)
	    ymin = float(item.ymin)
	    ymax = float(item.ymax)
	    MID_X = float(xmin + xmax)/2  # get center[x, y] of rectangle
	    MID_Y = float(ymin + ymax)/2

#----------liner transformation----------
def curve():
    global image_x, arm_x  # X compornet 
    I_X = np.array(image_x)    
    A_X = np.array(arm_x)
    fx = np.polyfit(I_X, A_X, 1)
    cfx = np.poly1d(fx)

    global image_y, arm_y  # Y compornet
    I_Y = np.array(image_y) 
    A_Y = np.array(arm_y)
    fy = np.polyfit(I_Y, A_Y, 1)
    cfy = np.poly1d(fy) 
                    
    global pic_M, M  # transformation result
    M[0] = cfx(pic_M[0])
    M[1] = cfy(pic_M[1])

#----------robot motion planning----------     
def listener():
    global MID_X, MID_Y, key
    global action, my_data
    for case in switch(action): 
        if case(0):
            print('case0')
            my_data != []  # initialize
            action = 1
            break
        if case(1):
            print('case1')
            time.sleep(1.5)  # have to set a sleep,"make sure" get a result data 
            key = 1
            if my_data != []:  # "double check" for getting the data indeed
                action = 2
            break
        if case(2):
            print('case2')
            curve()
            break
        if case():
            rospy.on_shutdown(myhook)

def myhook():
    print ("shutdown time!")

if __name__ == '__main__':
    rospy.init_node('the_node', anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)  # Yolo topic
    rate = rospy.Rate(40)
    listener()
    rospy.spin()'''
