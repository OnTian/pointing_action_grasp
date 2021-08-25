#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes  #yolo result by array[]
import numpy as np
import time
import signal

global M, C, pic_M, pic_C
M,C ,pic_M, pic_C= 0.0,0.0,[0.0,0.0],[0.0,0.0]
cont = True

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

def callback(data):
    global key, MID_X, MID_Y, my_data
    if key != 0:
        my_data = data
        key = 0
    
    for item in my_data.bounding_boxes:
        # print(item)
        # for item in data.bounding_boxes:
        # if item.Class == '8':
        #     print('1')
        xmin = float(item.xmin)
        xmax = float(item.xmax)
        ymin = float(item.ymin)
        ymax = float(item.ymax)
        MID_X = float(xmin + xmax)/2
        MID_Y = float(ymin + ymax)/2
        pic_M = [MID_X, MID_Y]
        pic_C = [MID_X, MID_Y]

def curve():
    global image_x  
    I_X = np.array(image_x)
    global arm_x    
    A_X = np.array(arm_x)
    fx = np.polyfit(I_X, A_X, 1)              # x1
    print(fx)
    cfx = np.poly1d(fx)                       # 
    print(cfx)

    global image_y  
    I_Y = np.array(image_y)
    global arm_y 
    A_Y = np.array(arm_y)
    fy = np.polyfit(I_Y, A_Y, 1)
    cfy = np.poly1d(fy)
    print("=========================================")                     
    global M, C, OB_1, OB_2
    global pic_M, pic_C, pic_OB_1, pic_OB_2, pic_ball

    M[0] = cfx(pic_M[0])
    M[1] = cfy(pic_M[1])
    C[0] = cfx(pic_C[0])
    C[1] = cfy(pic_C[1])
    print(M,'\n',C)
    
def listener():
    global MID_X, MID_Y, a, key
    global action, my_data
    for case in switch(action): 
        if case(0):
            print('case0')
            action = 1
            break
        if case(1):
            print('case1')
            time.sleep(1.5)
            action = 2
            break
        if case(2):
            print('case2')
            curve()
            key = 1
            break
        if case(): # default, could also just omit condition or 'if True'
            global cont
            rospy.signal_shutdown('finish')
            rospy.on_shutdown(myhook)

def handler(signal, frame):
    global cont
    cont = False

def the_node():
    global cont
    disable_signals = True
    rospy.init_node('the_node', anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
    rate = rospy.Rate(40)
    # try:
    while cont:
        listener()
        rate.sleep()

    rospy.signal_shutdown('finish')
    rospy.spin()

def myhook():
    global cont, action
    print ("shutdown time!")
    for case in switch(action): 
        if case(): # default, could also just omit condition or 'if True'
            print('back to home')

if __name__ == '__main__':
    global image_x, arm_x, image_y, arm_y, key, a, MID_X
    key = 1; a = [0, 0]; MID_X = 0; MID_Y = 0
    image_x = [85,    168,  551,  742, 1093, 1220, 1215, 1215, 1212, 1125,  737,  549,   204,   81,   84,   84,   490,   664,   838,   577,   749,   663,   576,   748,   490,   834]
    arm_x   = [-30, -25.5, -5.5,  5.0, 23.5, 29.5, 29.5, 29.5,   25,   25,  4.5, -5.5, -24.5,  -31,  -30,  -30, -10.8,  -2.3,   6.2, -6.55,  1.95,  -2.3, -6.65,  1.85, -10.9,   6.0]
    image_y = [70,     69,   92,   94,   69,   74,  263,  412,  602,  601,  570,  571,   597,  595,  406,  260,   108,   110,   111,   197,   199,   284,   369,   370,   454,   457]
    arm_y   = [59.8, 59.8, 58.3, 58.3, 59.3, 59.3, 49.3, 40.8, 30.8, 30.8, 32.8, 32.8,  31.8, 31.8, 41.8, 49.8, 54.62, 54.61, 54.61, 50.56, 50.56, 46.31, 42.26, 42.16, 38.01, 37.91]
    global action
    action = 0
    signal.signal(signal.SIGINT, handler)
    the_node()
