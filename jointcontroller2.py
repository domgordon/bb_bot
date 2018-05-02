import os
import sys
import json
import pickle
from pprint import pprint
import image_geometry
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ast import literal_eval
import numpy as np
import collections
import rospy
import pickle
import moveit_commander
import moveit_python
from moveit_msgs.msg import MoveItErrorCodes

def anglecalc(a, b, c):
	ba = a - b
	bc = c - b
	cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
	angle = np.arccos(cosine_angle)
	deg = np.degrees(angle)
	return np.radians(deg)


bridge = CvBridge()

ci_dir = '/home/student/Baxter-The-Pool-Wiz/camInfo'
depth_dir  = '/home/student/Baxter-The-Pool-Wiz/depth'
parts = ["Nose", "Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist"]
picdir = '/home/student/Baxter-The-Pool-Wiz/json_output'

#list of dictionarys (list of the body rays)
output = []
idx = 0

seen = set()

#for all the json files in the current directory (opencv/output)
for sd, d, fs in os.walk(picdir):
	for f in fs:
		print(f)
		#if its a json
		if (".json" in f) and (f not in seen):
			seen.add(f)
			timestamp = f.split("_")[0] #get the timestamp
			camfile = timestamp + ".obj" #get the f
			depthfile = timestamp + ".jpeg" #get the filename for the corresp depth jpeg
			bodyparts = {}
			bodyray = {}
			bodydict = {}
			print(f)
			newdir = picdir + '/' + f
			with open(newdir) as data_file: #open this json file
				data = json.load(open(newdir))
				#fill bodyparts dictionary with the desired parts as keys and u,v coords as vals
				for i in range(0,8):
					pc = data['part_candidates'][0][str(i)]
					max = 0
					#if there are more than two sets of (x,y,c) coordinates 
					if(len(pc) > 6):
						print(data['part_candidates'])
					if(len(pc) > 3):
						if(pc[2] > pc[5]):
							#print(pc[0:3])
							bodyparts[parts[i]] = pc[0:2]
						else:
							#print(pc[3:6])
							bodyparts[parts[i]] = pc[3:5]
					else:
						#print(pc)
						bodyparts[parts[i]] = pc[0:2]
			#print(bodyparts)
			#for all the jpegs in the depth directory
			for subdr, firs, files in os.walk(depth_dir):
				for d_file in files:
					#if there is the matching timestamp for the current json
					if d_file == depthfile:
						full_path = depth_dir + '/' + d_file
						#read in this depth image
						cv_image = cv2.imread(full_path, cv2.IMREAD_GRAYSCALE)
						#for each desired body part, use the u,v to get the d
						for part in bodyparts:
							u = int(round(bodyparts[part][0]))
							v = int(round(bodyparts[part][1]))
							d = cv_image[v][u]
							bodyparts[part]= [u,v,d]
						#print(bodyparts)
						for subdr, firs, files in os.walk(ci_dir):
							for ci_file in files:
								if ci_file == camfile:
									full_path2 = ci_dir + '/' + ci_file
									cam_info = pickle.load(open(full_path2, 'rb'))
									for part in bodyparts:
										u = bodyparts[part][0]
										v = bodyparts[part][1]
										d = bodyparts[part][2]
										camera_model = image_geometry.PinholeCameraModel()
										ci = cam_info
										camera_model.fromCameraInfo(ci)
										rect_uv = camera_model.rectifyPoint((u,v))
										ray = camera_model.projectPixelTo3dRay(rect_uv)
										#convert it to z
										world = [0,0,0]
										for i in range(0,3):
											x = ray[i]
											world[i] = round(x,3)
										
										bodyray[part] = world
									#print bodyray
									bodydict[timestamp] = bodyray
									output.append(bodydict)
									idx += 1 

body_angles = {}

#for each entry
for i in range(0, len(output)):
	for timestamp in output[i]:
		joints = output[i][timestamp]

		time = float(int(timestamp)/100000)

		rwrist = np.array(joints['RWrist'])
		relbow = np.array(joints['RElbow'])
		rshoulder = np.array(joints['RShoulder'])
		lwrist = np.array(joints['LWrist'])
		lelbow = np.array(joints['LElbow'])
		lshoulder = np.array(joints['LShoulder'])
		neck = np.array(joints['Neck'])

		#find right shoulder angle: neck, rshoulder, relbow
		rshoulder_ang = anglecalc(neck, rshoulder, relbow) - np.pi
		#find left shoulder angle: neck, lshoulder, lelbow
		lshoulder_ang = anglecalc(neck, lshoulder, lelbow) - np.pi
		#find right elbow angle: rshoulder, relbow, rwrist
		relbow_ang = np.pi - anglecalc(rshoulder, relbow, rwrist)
		#fing left elbow angle: lshoulder, lelbow, lwrist
		lelbow_ang = np.pi -anglecalc(lshoulder, lelbow, lwrist)
		#angles = ['left_s0':, 'left_s1':, 'left_e0':, 'left_e1':, 'left_w0':, 'left_w1':, 'left_w2':, 'left_hand':, 'left_endpoint':]
		#angles = {'RShoulderAng': rshoulder_ang, 'LShoulderAng': lshoulder_ang, 'RElbowAng': relbow_ang, 'LElbowAng': lelbow_ang}
		print (lshoulder_ang)
		if (lelbow_ang > 0.9 and lelbow_ang <2.3 and relbow_ang > 0.9 and relbow_ang < 2.3):
			print ("rollie")
			#if(time <= 86384769222120):
			#rollie
			l = [0.7, lshoulder_ang, -3, lelbow_ang, 0.0, 0.0, 0.0]
			r = [-0.7, rshoulder_ang, 3, relbow_ang, 0.0, 0.0, 0.0]
		elif ((lelbow_ang > 1.8 and relbow_ang <.65) or (relbow_ang > 1.8 and lelbow_ang <.65)):
			print ("dab")
			#elif(time <= 86386421926017):
			#dab
			l = [0.7, lshoulder_ang, -2, lelbow_ang, 0.0, 0.0, 0.0]
			r = [-0.7, rshoulder_ang, 2, relbow_ang, 0.0, 0.0, 0.0]
		else:
			print ("pants")
			#pants
			l = [0.7, -lshoulder_ang, -1, lelbow_ang, 0.0, 0.0, 0.0]
			r = [-0.7, -rshoulder_ang, 1, relbow_ang, 0.0, 0.0, 0.0]
				
		body_angles[time] = (l,r)

ordered = collections.OrderedDict(sorted(body_angles.items()))


rospy.init_node("tmp")

# Initialize moveGroupInterface
both_arm_move_group = moveit_python.move_group_interface.MoveGroupInterface("both_arms", "base")

# Left arm joint names
both_joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

for timestamp in ordered:
	print(timestamp)
	left_pose, right_pose = ordered[timestamp]
	both_pose = left_pose + right_pose
	print both_pose
	#both together ??
	both_arm_move_group.moveToJointPosition(both_joint_names, both_pose)
	both_arm_move_group.get_move_action().wait_for_result()
	result_both = both_arm_move_group.get_move_action().get_result()
	if result_both.error_code.val == MoveItErrorCodes.SUCCESS:
		print("Successful!")
	else:
		print("Error both!")