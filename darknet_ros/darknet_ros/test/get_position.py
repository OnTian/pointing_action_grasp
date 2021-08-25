#! /usr/bin/env python
# '''command : python get_position.py --image [image path]'''
import copy
import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
# sys.path.insert(0, "/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/")
import cv2
import argparse
#import rospy
#from transform.srv import Send_Ints, Send_IntsResponse

# def handle_add_two_ints(req):
# 	global refPt
#     print("Returning image positon")
#     return Send_IntsResponse(refPt)

# def send_position_server():
#     rospy.init_node('send_position_server')
#     s = rospy.Service('send_position', Send_Ints, handle_add_two_ints)
#     print("Ready to send position.")
#     rospy.spin()

def click_and_crop(event, x, y, flags, param):
	# grab references to the global variables
	global refPt, cropping
	# if the left mouse button was clicked, record the starting
	# (x, y) coordinates and indicate that cropping is being
	# performed
	if event == cv2.EVENT_LBUTTONDOWN:
		refPt = [(x, y)]
		# curve(refPt)
		print('1',refPt)
		cropping = True
	# check to see if the left mouse button was released
	elif event == cv2.EVENT_LBUTTONUP:
		# record the ending (x, y) coordinates and indicate that
		# the cropping operation is finished
		refPt.append((x, y))
		cropping = False
		# draw a rectangle around the region of interest
		cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
		cv2.imshow("image", image)
	#send_position_server()


if __name__ == '__main__':
	global refPt, cropping
	refPt = []
	cropping = False
	# construct the argument parser and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--image", required=True, help="Path to the image")
	args = vars(ap.parse_args())
	# load the image, clone it, and setup the mouse callback function
	image = cv2.imread(args["image"])
	# image = cv2.imread(args["image"])
	clone = copy.copy(image)
	cv2.namedWindow("image")
	cv2.setMouseCallback("image", click_and_crop)
	# keep looping until the 'q' key is pressed
	while True:
		# display the image and wait for a keypress
		cv2.imshow("image", image)
		key = cv2.waitKey(1) & 0xFF
		# if the 'r' key is pressed, reset the cropping region
		if key == ord("r"):
			image = copy.copy(clone)
		# if the 'c' key is pressed, break from the loop
		elif key == ord("c"):
			break
	# if there are two reference points, then crop the region of interest
	# from teh image and display it
	if len(refPt) == 2:
		roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
		cv2.imshow("ROI", roi)
		cv2.waitKey(0)
	# close all open windows
	cv2.destroyAllWindows()


