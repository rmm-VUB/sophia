#!/usr/bin/env python


import rospy

from math import *
import numpy as np
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from mocap_vicon.msg import Markers
from mocap_vicon.msg import Marker

from reba_vicon.msg import JointAngles
from reba_vicon.msg import JointAngle

import sys

x = 0
y = 1
z = 2

#name list of dots
list_dot = ['lower_arm_r1','lower_arm_r2','upper_arm_r1','shoulder_r1','shoulder_l2','upper_arm_l2','lower_arm_l2','neck2','hip_r2','hip_l2','knee_r2','knee_l2','ankle_r2','ankle_l2']
#dico position
body_coord = {}

reba_A_table = np.array([
	[[1,2,3,4],[1,2,3,4],[3,3,5,6]],
	[[2,3,4,5],[3,4,5,6],[4,5,6,7]],
	[[2,4,5,6],[4,5,6,7],[5,6,7,8]],
	[[3,5,6,7],[5,6,7,8],[6,7,8,9]],
	[[4,6,7,8],[6,7,8,9],[7,8,9,9]]
	])

reba_B_table = np.array([
	[[1,2,2],[1,2,3]],
	[[1,2,3],[2,3,4]],
	[[3,4,5],[4,5,5]],
	[[4,5,5],[5,6,7]],
	[[6,7,8],[7,8,8]],
	[[7,8,8],[8,9,9]]
	])

reba_C_table = np.array([
	[1,1,2,3,4,6,7,8,9,10,11,12],
	[1,2,3,4,4,6,7,8,9,10,11,12],
	[1,2,3,4,4,6,7,8,9,10,11,12],
	[2,3,3,4,5,7,8,9,10,11,11,12],
	[3,4,4,5,6,8,9,10,10,11,12,12],
	[3,4,5,6,7,8,9,10,10,11,12,12],
	[4,5,6,7,8,9,9,10,11,11,12,12],
	[5,6,7,8,8,9,10,10,11,12,12,12],
	[6,6,7,8,9,10,10,10,11,12,12,12],
	[7,7,8,9,9,10,11,11,12,12,12,12],
	[7,7,8,9,9,10,11,11,12,12,12,12],
	[7,8,8,9,9,10,11,11,12,12,12,12]
	])

def RebaA(body_coord):

	rebaAtable = np.array([
		[[1,2,3,4],[1,2,3,4],[3,3,5,6]],
		[[2,3,4,5],[3,4,5,6],[4,5,6,7]],
		[[2,4,5,6],[4,5,6,7],[5,6,7,8]],
		[[3,5,6,7],[5,6,7,8],[6,7,8,9]],
		[[4,6,7,8],[6,7,8,9],[7,8,9,9]]
		])

	Aneck = RebaAneck()
	Atrunk = RebaAtrunk(body_coord['torso'],body_coord['neck'],body_coord['shoulder','l'],body_coord['shoulder','r'])
	Alegs = RebaAlegs(body_coord['hip','l'],body_coord['knee','l'],body_coord['foot','l'],body_coord['hip','r'],body_coord['knee','r'],body_coord['foot','r'])

	print ('Reba_neck: {},Reba_truk: {}, Reba_legs:{}'.format(Aneck,Atrunk,Alegs))

	RebaAscore = rebaAtable[Atrunk-1,Aneck -1,Alegs -1]

	return RebaAscore

def RebaAneck():

	RebaAneck = 1

	return min(3,RebaAneck)

def RebaAtrunk(trunk,neck,lshoul,rshoul):
	global x,y,z

	trunk_comp = {}

	trunk_comp['bend_face'] = [neck[x]-trunk[x],0,neck[z]-trunk[z]]
	trunk_comp['bend_side'] = [0,neck[y]-trunk[y],neck[z]-trunk[z]]
	trunk_comp['twist'] = [lshoul[x]-rshoul[x],lshoul[y]-rshoul[y],0]

	#Trunk bend face compute
	bend_comp = [0,0,1]
	twist_comp = [1,0,0]

	trunk_bend_face = ScalProdInv(trunk_comp['bend_face'],bend_comp)
	trunk_bend_side = ScalProdInv(trunk_comp['bend_side'],bend_comp)
	trunk_twist = ScalProdInv(trunk_comp['twist'],twist_comp)

	RebaAtrunk = 1

	if (trunk_bend_face > 5) or (neck[x] < trunk[x]):

		RebaAtrunk += 1

		if trunk_bend_face > 20:

			RebaAtrunk += 1
			print('trunk middle bended')
			if trunk_bend_face > 80:
				print('trunk hard bended')
				RebaAtrunk += 1

	if trunk_bend_side > 10:
		print('trunk side bended')
		RebaAtrunk += 1

	if abs(trunk_twist-90) > 10:
		print('trunk stwisted')
		RebaAtrunk += 1

	#print("trunk_bend_face: {}, trunk_bend_side: {}, trunk_twist: {}".format(trunk_bend_face,trunk_bend_side,trunk_twist))

	return min(5,RebaAtrunk)



def RebaAlegs(hipL,kneeL,footL,hipR,kneeR,footR):

	epsiAlign = 30
	#aligned legs?: 1 if yes 2 if not
	kneeAngleL = PythInv(hipL,kneeL,footL)
	kneeAngleR = PythInv(hipR,kneeR,footR)
	#print ("Knee_L: {}, Knee_R: {}".format(kneeAngleL,kneeAngleR))

	if (kneeAngleL < kneeAngleR - epsiAlign):
		rebaALegs = 2

		if kneeAngleR < 110:
			rebaALegs = 4

		elif kneeAngleR <140:
			rebaALegs = 3

	elif (kneeAngleR < kneeAngleL - epsiAlign):
		rebaALegs = 2

		if kneeAngleL < 110:
			rebaALegs = 4

		elif kneeAngleL <140:
			rebaALegs = 3

	elif (kneeAngleL < 110 or kneeAngleR < 110):
		rebaALegs = 3


	elif (kneeAngleL < 140 or kneeAngleR < 140):
		rebaALegs = 2

	else:
		rebaALegs = 1

	return min(4,rebaALegs)


def RebaB(body_coord):

	RebaBtable = np.array([
		[[1,2,2],[1,2,3]],
		[[1,2,3],[2,3,4]],
		[[3,4,5],[4,5,5]],
		[[4,5,5],[5,6,7]],
		[[6,7,8],[7,8,8]],
		[[7,8,8],[8,9,9]]
		])


	BupArm = RebaBupArm(body_coord)
	BlowArm = RebaBlowArm(body_coord)
	Bwrist = RebaBwrist()

	print('Reba Arm Up: {} , Reba Arm Low:  {}'.format(BupArm,BlowArm))

	RebaBscore = RebaBtable[BupArm-1,BlowArm-1,Bwrist-1]

	return RebaBscore

def RebaBupArm(body):
	side_limit = 20
	mid_hip = [0,0,0]
	mid_shoulder = [0,0,0]
	Vdfp = {x:[0,0,0],y:[0,0,0],z:[0,0,0],"sh_sh":[0,0,0],"hip_sh":[0,0,0]}
	Vdsp = {x:[0,0,0],y:[0,0,0],z:[0,0,0]}

	for i in range(3):

	#create mid hip/shoulder point
		mid_hip[i] = body['hip','r'][i]+((body['hip','l'][i]-body['hip','r'][i])/2)
		mid_shoulder[i] = body['shoulder','r'][i]+((body['shoulder','l'][i]-body['shoulder','r'][i])/2)

	#vect dir of mid face plan

		Vdfp["sh_sh"][i] = body['shoulder','l'][i]-body['shoulder','r'][i]
		Vdfp["hip_sh"][i] = body['shoulder','l'][i]-mid_hip[i]

	#vect dir of mid side plan

		Vdsp[y][i] = mid_shoulder[i]-mid_hip[i]

	Vdsp[x] = perpVect3D(Vdfp["sh_sh"],Vdfp["hip_sh"])
	Vdsp[z] = perpVect3D(Vdsp[x],Vdsp[y])


	#Compute Vect of up arm for each arm
	VupArm = {'r':[0,0,0],'l':[0,0,0]}
	VupArmuvp = {'r':[0,0,0],'l':[0,0,0]}
	UpArmComp = [0,-1,0]
	UpViewArmComp = [1,0,0]
	RebaBupArm = {}
	upArmAngle={'r':0,'l':0}
	upViewArmAngle={'r':0,'l':0}

	for j in ['l','r']:
		for i in range(3):
			VupArm[j][i] = body['elbow',j][i] - body['shoulder',j][i]

		VupArm[j] = changeRefVec(Vdsp,VupArm[j])

		#Proj on up view plan
		#VupArmsp[j] = [VupArm[j][x],VupArm[j][y],0]
		VupArmuvp[j] = [VupArm[j][x],0,VupArm[j][z]]

		#Compute angle between Comparaison and proj
		upArmAngle[j] = ScalProdInv(VupArm[j],UpArmComp)
		upViewArmAngle[j] = ScalProdInv(VupArmuvp[j],UpViewArmComp)

		print('angle up arm {}: {}'.format(j,upArmAngle[j]))
		print('angle up view arm {}: {}'.format(j,upViewArmAngle[j]))
		RebaBupArm[j] = 1

		if upArmAngle[j] > 20:

			RebaBupArm[j] += 1

			if upArmAngle[j] > 45:

				RebaBupArm[j] += 1

				if (abs(upViewArmAngle[j]) < 90 and abs(upViewArmAngle[j]) < 90-side_limit) or (abs(upViewArmAngle[j]) > 90 and abs(upViewArmAngle[j]) < 90+side_limit):

					RebaBupArm[j] += 1

				elif upArmAngle[j] > 90:

					RebaBupArm[j] += 1

	return min(max(RebaBupArm['l'],RebaBupArm['r']),6)



def RebaBlowArm(body):

	RebaBlowArm = {}
	lowArmAngle = {}

	for j in ['l','r']:

		lowArmAngle[j] = PythInv(body['hand',j],body['elbow', j],body['shoulder', j]) - 90

		print "lower arm", j, " : ", lowArmAngle[j]

		RebaBlowArm[j] = 1

		if abs(lowArmAngle[j]) > 15.0:
			RebaBlowArm[j] += 1



	return min(2,RebaBlowArm[j])

def RebaBwrist():

	RebaBwrist = 1

	return min(3,RebaBwrist)

def RebaC(body_coord):
	RebaCtable = np.array([
		[1,1,2,3,4,6,7,8,9,10,11,12],
		[1,2,3,4,4,6,7,8,9,10,11,12],
		[1,2,3,4,4,6,7,8,9,10,11,12],
		[2,3,3,4,5,7,8,9,10,11,11,12],
		[3,4,4,5,6,8,9,10,10,11,12,12],
		[3,4,5,6,7,8,9,10,10,11,12,12],
		[4,5,6,7,8,9,9,10,11,11,12,12],
		[5,6,7,8,8,9,10,10,11,12,12,12],
		[6,6,7,8,9,10,10,10,11,12,12,12],
		[7,7,8,9,9,10,11,11,12,12,12,12],
		[7,7,8,9,9,10,11,11,12,12,12,12],
		[7,8,8,9,9,10,11,11,12,12,12,12]
		])

	REBA_A = RebaA(body_coord)
	REBA_B = RebaB(body_coord)

	REBA_C = RebaCtable[REBA_A-1][REBA_B-1]

	return (REBA_A,REBA_B,REBA_C)


def callbackJointPositions(data):
	global list_dot, body_coord
	for name in list_dot:
		for m in data.markers:
			if name == m.marker_name:
				body_coord[name] = [m.translation.x,m.translation.y,m.translation.z]

				if name == 'hip_l1':
					body_coord['shoulder_r1'] = [m.translation.x,m.translation.y,m.translation.z]
					

	# Mid-point hip
	body_coord['mid_hip'] = np.add(body_coord['hip_r2'], body_coord['hip_l2'])//2


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def angle_between(v1, v2, pr=False):
	# normalize B (A and B should both be normalized)
	v1 = unit_vector(v1)
	v2 = unit_vector(v2)

	# cross and dot products of A and B
	v3 = np.cross(v1,v2)
	v4 = np.dot(v1,v2)

	# create and normalize a quaternion
	Q = [v3[0], v3[1], v3[2], v4+1]
	Q = unit_vector(Q)

	# v3 = np.cross(v1,v2)
	# k_cos_theta = np.dot(v1,v2)
	# k = norm(v1)*norm(v2)
	#
	# if(k_cos_theta/k == -1):
	# 	Q = [0, unit_vector(orthogonal(v1))]
	# else:
	# 	Q = [k_cos_theta + k, v3[0], v3[1], v3[2]]
	# #Q = np.array(Q)
	# #print Q
	# Q = unit_vector(Q)

	# convert quaternion to euler representation
	#Q =  tf.transformations.quaternion_about_axis(angle(v1,v2),v3)
	euler = tf.transformations.euler_from_quaternion(Q,'sxyz')
	#euler = quaternion_to_euler_angle_vectorized1(Q[3],Q[0],Q[1],Q[2])
	# roll = euler[0]
	# pitch = euler[1]
	# yaw = euler[2]
	euler = np.array(euler)

	if pr:
		print Q

	return np.degrees(euler)
	#return euler


def quaternion_to_euler_angle_vectorized1(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = np.degrees(np.arctan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = np.where(t2>+1.0,+1.0,t2)
	#t2 = +1.0 if t2 > +1.0 else t2

	t2 = np.where(t2<-1.0, -1.0, t2)
	#t2 = -1.0 if t2 < -1.0 else t2
	Y = np.degrees(np.arcsin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = np.degrees(np.arctan2(t3, t4))

	return X, Y, Z

def dot_product(x, y):
    return sum([x[i] * y[i] for i in range(len(x))])

def norm(x):
    return sqrt(dot_product(x, x))

def normalize(x):
    return [x[i] / norm(x) for i in range(len(x))]

def project_onto_plane(x, n):
    d = dot_product(x, n) / norm(n)
    p = [d * normalize(n)[i] for i in range(len(n))]
    return [x[i] - p[i] for i in range(len(x))]

def orthogonal(v):
	x = np.random.randn(3)
	return np.dot(x,v) * v

def calculate_vector():
	global body_coord

	vector = {}
	# Body vectors
	vector['neck'] = np.subtract(body_coord['neck2'], body_coord['shoulder_r1'])
	vector['trunk'] = np.subtract(body_coord['shoulder_r1'], body_coord['mid_hip'])
	vector['left_shoulder'] = np.subtract(body_coord['shoulder_l2'], body_coord['shoulder_r1'])
	vector['right_shoulder'] = np.subtract(body_coord['upper_arm_r1'], body_coord['shoulder_r1'])
	vector['left_upper_arm'] = np.subtract(body_coord['upper_arm_l2'], body_coord['shoulder_l2'])
	vector['right_upper_arm'] = np.subtract(body_coord['lower_arm_r1'], body_coord['upper_arm_r1'])
	vector['left_lower_arm'] = np.subtract(body_coord['lower_arm_l2'], body_coord['upper_arm_l2'])
	vector['right_lower_arm'] = np.subtract(body_coord['lower_arm_r2'], body_coord['lower_arm_r1'])
	vector['left_hip'] = np.subtract(body_coord['hip_l2'], body_coord['mid_hip'])
	vector['right_hip'] = np.subtract(body_coord['hip_r2'], body_coord['mid_hip'])
	vector['left_knee'] = np.subtract(body_coord['knee_l2'], body_coord['hip_l2'])
	vector['right_knee'] = np.subtract(body_coord['knee_r2'], body_coord['hip_r2'])
	vector['left_ankle'] = np.subtract(body_coord['ankle_l2'], body_coord['knee_l2'])
	vector['right_ankle'] = np.subtract(body_coord['ankle_r2'], body_coord['knee_r2'])

	return vector

def degrees(angle):
	return angle*180/pi

def calculate_joint_angles(v):
	gravity_vector = [0,0,1]

	angles = {'neck_flexion':0,'neck_bending': 0, 'trunk_flexion': 0, 'trunk_bending': 0, 'trunk_twist': 0,
				'right_clavicle_elevation': 0, 'right_clavicle_abduction': 0, 'right_shoulder_abduction': 0, 'right_shoulder_flexion': 0,
				'right_elbow_rotation': 0, 'right_elbow_flexion': 0, 'right_hip_abduction': 0, 'right_hip_flexion': 0, 'right_knee_flexion': 0,
				'left_clavicle_elevation': 0, 'left_clavicle_abduction': 0, 'left_shoulder_abduction': 0, 'left_shoulder_flexion': 0,
				'left_elbow_rotation': 0, 'left_elbow_flexion': 0, 'left_hip_abduction': 0, 'left_hip_flexion': 0, 'left_knee_flexion': 0}

	# # Neck
	neck_angles = angle_between(v['trunk'],v['neck'])
	angles['neck_flexion'] = neck_angles[0]
	angles['neck_bending'] = neck_angles[1]

	# Trunk
	trunk_angles = angle_between(v['trunk'],gravity_vector)
	#print v['trunk']
	trunk_hip_angles = angle_between(v['right_hip'],v['trunk'])
	angles['trunk_flexion'] = trunk_angles[0]
	angles['trunk_bending'] = 90-trunk_hip_angles[1]
	if trunk_hip_angles[2] == -180 or trunk_hip_angles[2] == 180:
		angles['trunk_bending'] = - angles['trunk_bending']
	shoulder_hip_angles = angle_between(v['right_hip']-v['left_hip'],v['right_shoulder']-v['left_shoulder'])
	angles['trunk_twist'] = shoulder_hip_angles[2]

	# Right clavicle
	right_clavicle_angles = angle_between(v['right_hip'],v['right_shoulder'])
	angles['right_clavicle_elevation'] = right_clavicle_angles[1]
	angles['right_clavicle_abduction'] = right_clavicle_angles[2]

	# Left clavicle
	left_clavicle_angles = angle_between(v['left_hip'], v['left_shoulder'])
	angles['left_clavicle_elevation'] = -left_clavicle_angles[1]
	angles['left_clavicle_abduction'] = -left_clavicle_angles[2]

	# Right arm (upper arm)
	right_upper_arm_angles = angle_between(v['right_shoulder'],v['right_upper_arm'])
	angles['right_shoulder_abduction'] = right_upper_arm_angles[1] + 90
	if right_upper_arm_angles[2] == -180 or right_upper_arm_angles[2] == 180:
		angles['right_shoulder_abduction'] = - angles['right_shoulder_abduction']
	p = project_onto_plane(v['right_upper_arm'],v['right_shoulder'])
	if norm(p) > 0.001:
		right_upper_arm_angles_1 = angle_between(np.cross(v['right_shoulder'],v['trunk']),p)
	else:
		right_upper_arm_angles_1 = [0, 0, 0]
	angles['right_shoulder_flexion'] = -right_upper_arm_angles_1[0] - 90


	# Left arm (upper arm)
	left_upper_arm_angles = angle_between(v['left_shoulder'],v['left_upper_arm'])
	angles['left_shoulder_abduction'] = -left_upper_arm_angles[1] + 90
	if left_upper_arm_angles[2] == -180 or left_upper_arm_angles[2] == 180:
		angles['left_shoulder_abduction'] = - angles['left_shoulder_abduction']
	p = project_onto_plane(v['left_upper_arm'],v['left_shoulder'])
	if norm(p) > 0.001:
		left_upper_arm_angles_1 = angle_between(np.cross(v['left_shoulder'],v['trunk']),p)
	else:
		left_upper_arm_angles_1 = [0, 0, 0]
	angles['left_shoulder_flexion'] = -left_upper_arm_angles_1[0] + 90

	# Right arm (lower arm)
	angles['right_elbow_flexion'] = degrees(angle(v['right_upper_arm'],v['right_lower_arm']))
	p = project_onto_plane(v['right_lower_arm'],v['right_upper_arm'])
	if norm(p) > 0.001:
		right_lower_arm_angles_1 = angle_between(np.cross(v['right_upper_arm'],v['right_shoulder']),p)
	else:
		right_lower_arm_angles_1 = [0, 0, 0]
	#print right_lower_arm_angles_1
	angles['right_elbow_rotation'] = right_lower_arm_angles_1[2]

	#Left arm (lower arm)
	angles['left_elbow_flexion'] = degrees(angle(v['left_upper_arm'],v['left_lower_arm']))
	p = project_onto_plane(v['left_lower_arm'],v['left_upper_arm'])
	if norm(p) > 0.001:
		left_lower_arm_angles_1 = angle_between(np.cross(v['left_upper_arm'],v['left_shoulder']),p)
	else:
		left_lower_arm_angles_1 = [0, 0, 0]
	#print left_lower_arm_angles_1
	angles['left_elbow_rotation'] = left_lower_arm_angles_1[2]

	# Right hip
	right_hip_angles = angle_between(v['right_hip'],v['right_knee'])
	angles['right_hip_abduction'] = right_hip_angles[1] + 90
	if right_hip_angles[2] == -180 or right_hip_angles[2] == 180:
		angles['right_hip_abduction'] = - angles['right_hip_abduction']
	p = project_onto_plane(v['right_knee'],v['right_hip'])
	if norm(p) > 0.001:
		right_hip_angles_1 = angle_between(np.cross(v['right_hip'],v['trunk']),p)
	else:
		right_hip_angles_1 = [0, 0, 0]
	angles['right_hip_flexion'] = -right_hip_angles_1[0] - 90

	# Left hip
	left_hip_angles = angle_between(v['left_hip'],v['left_knee'])
	angles['left_hip_abduction'] = -left_hip_angles[1] + 90
	if left_hip_angles[2] == -180 or left_hip_angles[2] == 180:
		angles['left_hip_abduction'] = - angles['left_hip_abduction']
	p = project_onto_plane(v['left_knee'],v['left_hip'])
	if norm(p) > 0.001:
		left_hip_angles_1 = angle_between(np.cross(v['left_hip'],v['trunk']),p)
	else:
		left_hip_angles_1 = [0, 0, 0]
	angles['left_hip_flexion'] = -left_hip_angles_1[0] + 90

	# Right knee
	angles['right_knee_flexion'] = degrees(angle(v['right_knee'],v['right_ankle']))

	# Left knee
	angles['left_knee_flexion'] = degrees(angle(v['left_knee'],v['left_ankle']))

	return angles

def REBA(v):
	global reba_C_table


def score_A(v):
	global reba_A_table

	neck_score = 1
	# Neck flexion
	neck = angle_between(v['neck'],v['trunk'])

if __name__ == '__main__':

	#publisher initialisation
	pub_REBA = rospy.Publisher('/reba', Int16, queue_size=0)
	pub_joint_angles = rospy.Publisher('/vicon/joint_angles', JointAngles, queue_size=0)

	#subscriber
	sub_positions = rospy.Subscriber("/vicon/labeled_markers", Markers, callbackJointPositions)

	#node initialisation
	rospy.init_node('reba_scoring')

	#list of name
	list_dot2 = ['head_1','neck_1','torso_1','left_shoulder_1','left_elbow_1','left_hand_1','left_hip_1','left_knee_1','left_foot_1','right_shoulder_1','right_elbow_1','right_hand_1','right_hip_1','right_knee_1','right_foot_1']

	rospy.sleep(1)

	while not rospy.is_shutdown():
		v = calculate_vector()
		j = calculate_joint_angles(v)

		msg = JointAngles()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()
		msg.joint_angles = []
		for name in enumerate(j):
			j_ang = JointAngle()
			j_ang.name = name[1]
			j_ang.value = j[name[1]]
			msg.joint_angles.append(j_ang)

		pub_joint_angles.publish(msg)
		REBA_scores = REBA(j)
		pass

		# try:
		# 	Reba=RebaC(body_coord2)
		# 	print('RebaA : {}'.format(Reba[0]))
		# 	print('RebaB : {}'.format(Reba[1]))
		# 	print('RebaC : {}'.format(Reba[2]))
		# 	#publish
		# 	pub_REBA.publish(Reba[2])
		#
		# except (ValueError):
		# 	print('fail reba')
		# 	rate.sleep()
