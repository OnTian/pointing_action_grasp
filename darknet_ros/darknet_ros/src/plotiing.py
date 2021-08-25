#!/usr/bin/env python3
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import rospy
import sys
# sys.path.insert(1, "/home/onda/tensorflow/lib/python3.5/site-packages/")
import math
import numpy as np
# import cv2
# from sensor_msgs.msg import Image

from darknet_ros_msgs.srv import plot_position
# from darknet_ros_msgs.srv import SendMovement

global point_transformed
point_transformed = []
global im_data
im_data = 0

def callback(data):
    im_data = data

def transformation(x, y, z):
    rotation_mat = np.array([[np.cos(15*np.pi/180), 0., np.sin(15*np.pi/180),-0.065],
                            [0., 1, 0, 0.08],
                            [-np.sin(15*np.pi/180), 0., np.cos(15*np.pi/180), 0.09],
                            [0.,0.,0.,1.]])#rotate 30-degree around x-axis
    # transfar_mat = np.array([[-0.14], [0.], [-0.21]])#[base-center to camera-center, 0, hieght]
    point = [[],[],[]]
    for i in range(3):
        # pre_point = np.dot(rotation_mat, np.array([[x[i]], [y[i]], [z[i]]])) + transfar_mat
        pre_point = np.dot(rotation_mat, np.array([[x[i]], [y[i]], [z[i]], [1.0]]))
        point[i] = [pre_point[0].tolist(), pre_point[1].tolist(), pre_point[2].tolist()]
    return point
    """
    rotation_mat = np.array([[np.cos(np.pi/18), 0., np.sin(np.pi/18),-0.14],[0., 1, 0, 0.06],[-np.sin(np.pi/18), 0., np.cos(np.pi/18), -0.21],[0.,0.,0.,1.]])#rotate 30-degree around x-axis
    # transfar_mat = np.array([[-0.14], [0.], [-0.21]])#[base-center to camera-center, 0, hieght]
    point = [[],[],[]]
    for i in range(3):
        # pre_point = np.dot(rotation_mat, np.array([[x[i]], [y[i]], [z[i]]])) + transfar_mat
        pre_point = np.dot(rotation_mat, np.array([[x[i]], [y[i]], [z[i]], [1.0]]))
        point[i] = [pre_point[0].tolist(), pre_point[1].tolist(), pre_point[2].tolist()]
    return point
    """

# def handle_content(req):
#     global point_transformed
#     if req == True:
#         return SendMovementResponse(point_transformed, True)
#     else:
#         return SendMovementResponse(point_transformed, False)

# def sending_server():
#     rospy.init_node('send_movement_server')
#     s = rospy.Service('send_movement', SendMovement, handle_content)
#     rospy.spin()

def add_two_ints_client():
    global point_transformed
    print("go")
    rospy.wait_for_service('show_figure')
    try:
        add_two_ints = rospy.ServiceProxy('show_figure', plot_position)
        done = True
        resp1 = add_two_ints(done)
        if resp1.is_done == True:
            x = [[resp1.head_minimum_z[0], resp1.pointing_minimum_z[0], resp1.target[0]]]
            y = [[resp1.head_minimum_z[1], resp1.pointing_minimum_z[1], resp1.target[1]]]
            z = [[resp1.head_minimum_z[2], resp1.pointing_minimum_z[2], resp1.target[2]]]
            for i in range(len(resp1.object)):
                if i%3 == 0: x.append(resp1.object[i])
                if i%3 == 1: y.append(resp1.object[i])
                if i%3 == 2: z.append(resp1.object[i])
            #plane
            # x = np.arange(-5, 5, 0.2)
            # y = np.arange(-5, 5, 0.2)
            # xx, yy = np.meshgrid(x, y)
            # zz = -(resp1.object[0]*xx + resp1.object[1]*yy + resp1.object[3]) / resp1.object[2]
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot3D(x[0], y[0], z[0])
        ax.scatter3D(x[1:],y[1:],z[1:])
        #plane
        # ax.plot_surface(xx, yy, zz, color = 'b', alpha = 0.5)
        plt.show()
        # file = open("record.txt", 'a')
        # file.write("%s\n%s\n%s\n"%(x, y, z))
        # file.close()

        point_transformed = transformation(x[0],y[0],z[0])
        print(point_transformed)
        """
        global im_data
        im_data = cv2.imread('')
        point_size = 1
        point_color = (0,0,255)
        thickness = 4
        # point_x = resp1.target[0]
        # point_y = resp1.target[0]
        cv2.circle(im_data, [point_x, point_y], point_size, point_color, thickness)
        cv2.imshow('my image', im_data)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        """

        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
       
    

if __name__ == "__main__":
    # rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, callback)
    # sending_server()
    add_two_ints_client()