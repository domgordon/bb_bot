import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import json
import pickle
import subprocess
import os
import sys
from pprint import pprint
import image_geometry
from ast import literal_eval
import numpy as np
import collections
import moveit_commander
import moveit_python
from moveit_msgs.msg import MoveItErrorCodes
import threading

bridge = CvBridge()

#THIS IS MOVED INTO BAXTER_THE_POOL_WIZ!

def image_callback_rgb(msg):
   # print("received an image!")
    try:
        #convert ros image msg to opencv2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        global count_rgb 
        count_rgb+=1
        #make all the filenames same length:
        if len(str(msg.header.stamp.nsecs)) >= 9:
            #zeros = "0"*(9-len(str(msg.header.stamp.nsecs)))
            #timestamp = str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs) + zeros
            timestamp = str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)
            #print(count)
            #save your openCV2 image as a jpg
            picName = "/home/student/Baxter-The-Pool-Wiz/rgb/" + timestamp + ".jpeg"
            print(picName)

            cv2.imwrite(picName, cv2_img)


def image_callback_depth(msg):
   # print("received an image!")
    try:
        #convert ros image msg to opencv2
        #cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
    except CvBridgeError, e:
        print(e)
    else:
        global count_depth
        count_depth+=1
        #print(count)
        #save your openCV2 image as a jpg
        #make all the filenames same length:
        if len(str(msg.header.stamp.nsecs)) >= 9:
            #zeros = "0"*(9-len(str(msg.header.stamp.nsecs)))
            #timestamp = str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs) + zeros
            timestamp = str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)
            picName = "/home/student/Baxter-The-Pool-Wiz/depth/" + timestamp + ".jpeg"
            # print(picName)
            cv_image_array = np.array(cv2_img, dtype = np.dtype(np.float32))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            # cv_image_resized = cv2.resize(cv_image_norm, (1920, 1080), interpolation = cv2.INTER_CUBIC)
            cv2.imwrite(picName, cv_image_norm*255)

def callback_3d_depth(msg):
    # save message into file and read into file at a different 
    if len(str(msg.header.stamp.nsecs)) >= 9:
        #zeros = "0"*(9-len(str(msg.header.stamp.nsecs)))
        #timestamp = str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs) + zeros
        timestamp = str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)
        picName = "/home/student/Baxter-The-Pool-Wiz/camInfo/" + timestamp + ".obj"
        #use json to save camInfo file 
        f = open(picName,"w")
        pickle.dump(msg, f)
        f.close()
    


def main():
    global count_depth
    global count_rgb
    global  l
    count_depth=0
    count_rgb=0
    l = threading.Lock()
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic_depth = "/kinect2/hd/image_depth_rect"
    image_topic_rgb = "/kinect2/hd/image_color_rect"
    image_topic_caminfo = "/kinect2/hd/camera_info"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic_depth, Image, image_callback_depth)
    rospy.Subscriber(image_topic_caminfo, CameraInfo, callback_3d_depth)
    rospy.Subscriber(image_topic_rgb, Image, image_callback_rgb)
    # Spin until ctrl + c
    rospy.spin()
    #call openpose here 
    openposedir = os.chdir("/home/student/dependencies/openpose/")
    p = subprocess.Popen(['./build/examples/openpose/openpose.bin', '--image_dir', '/home/student/Baxter-The-Pool-Wiz/rgb', '--write_json', '/home/student/Baxter-The-Pool-Wiz/json_output/', '--part_candidates'])
    p.wait()
    print("done openpose")
#    b = subprocess.Popen(['.', 'baxter.sh', 'sim'])
    b2 = subprocess.Popen(['python', '/home/student/Baxter-The-Pool-Wiz/jointcontroller2.py'])
    print('done robot')
if __name__ == '__main__':
    main()
    