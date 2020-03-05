#!/usr/bin/env python


import rospy

from math import *
import numpy as np
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from mocap_vicon.msg import Markers
from mocap_vicon.msg import Marker

import sys

x = 0
y = 1
z = 2

#name list of dots
list_dot = ['lower_arm_r1','lower_arm_r2','upper_arm_r1','shoulder_r1','shoulder_l2','upper_arm_l2','lower_arm_l2','neck2','hip_r2','hip_l2']
#dico position
body_coord = {}

def compute_orient_mat(euler_angle):
	#euler_angle =  tf.transformations.euler_from_quaternion((Quater.x,Quater.y,Quater.z,Quater.w))
	c = [0,0,0]
	s = [0,0,0]
	for i in range(3):
		c[i] = cos(euler_angle[i])
		s[i] = sin(euler_angle[i])
	return np.array([[c[2]*c[1],c[2]*s[1]*s[0]-s[2]*s[0] ,c[2]*s[1]*c[0]+s[2]*s[0]],[s[2]*c[1],s[2]*s[1]*s[0]-c[2]*c[0],s[2]*c[1]*c[0]-c[2]*s[0]],[-s[1],c[1]*s[0],c[1]*c[0]]])

def fromKinectToAbs_coord(coord):
	# we need to change the abs_cord
	abs_coord = list(kinect_pose['coord']) #transform tuple into list
	for i in range(3):
		for j in range(3):
			abs_coord[i] += coord[j]*orient_mat[i][j]

	return abs_coord


def lenVect(A,B):
	global x,y,z
	AB = sqrt((B[x] -A[x])**2 + (B[y] -A[y])**2 + (B[z] -A[z])**2)
	return AB

def PythInv(A,B,C):
	global x,y,z
	#compute the angle ABC

	AB = sqrt((B[x] -A[x])**2 + (B[y] -A[y])**2 + (B[z] -A[z])**2)
	AC = sqrt((C[x] -A[x])**2 + (C[y] -A[y])**2 + (C[z] -A[z])**2)
	BC = sqrt((C[x] -B[x])**2 + (C[y] -B[y])**2 + (C[z] -B[z])**2)

	#print (AB)
	print((-AC**2 + AB**2 + BC**2) / (2*AB*BC))
	ABC = acos((-AC**2 + AB**2 + BC**2) / (2*AB*BC))

	return degrees(ABC)

def ScalProdInv(v1,v2):
	global x,y,z
	#compute angle between v1 and v2

	sp = v1[x]*v2[x] + v1[y]*v2[y] + v1[z]*v2[z]
	lenv1 = sqrt(v1[x]**2 + v1[y]**2 + v1[z]**2)
	lenv2 = sqrt(v2[x]**2 + v2[y]**2 + v2[z]**2)

	angle = acos(sp/(lenv1*lenv2))

	return degrees(angle)

def changeRefVec(new_ref,V):
	global x,y,z

	ref = {}

	for i in range(3):
		ref[i] = [new_ref[x][i],new_ref[y][i],new_ref[z][i]]
	A = np.array([ref[x],ref[y],ref[z]])
	B = np.array(V)

	new_coord = np.linalg.solve(A,B)

	new_V = [new_coord[0],new_coord[1],new_coord[2]]

	return new_V



def perpVect3D(V1,V2):

	#find the vector perpendiculat to the 2 others

	A = np.array([V1[:2],V2[:2]])
	B = np.array([-V1[2],-V2[2]])
	#print(A,B)

	X = np.linalg.solve(A,B)

	V = [X[0],X[1],1.]

	normV = sqrt(V[0]**2+V[1]**2)

	return [i / normV for i in V]


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


if __name__ == '__main__':

	global kinect_pose,orient_mat

	#publisher initialisation
	pub_REBA = rospy.Publisher('/reba', Int16, queue_size=0)

	#subscriber
	sub_positions = rospy.Subscriber("/vicon/labeled_markers", Markers, callbackJointPositions)

	#node initialisation
	rospy.init_node('reba_scoring')

	#frame origine
	kinect_pose = {'coord': (-0.0,0.0,1.85),'euler':(0.0,pi/6,0.0)}
	#compute orient matrix
	orient_mat =compute_orient_mat(kinect_pose['euler'])

	rate = rospy.Rate(40/5)

	#list of name
	list_dot2 = ['head_1','neck_1','torso_1','left_shoulder_1','left_elbow_1','left_hand_1','left_hip_1','left_knee_1','left_foot_1','right_shoulder_1','right_elbow_1','right_hand_1','right_hip_1','right_knee_1','right_foot_1']

	rospy.sleep(1)

	print body_coord

	sys.exit(0)

	while not rospy.is_shutdown():
		print '-----------------'
		#for name in list_dot :
		#	try:
				#trans = tfBuffer.lookup_transform('body', name,  rospy.Time.now(),rospy.Duration(1.0))
				#print trans
				#body_coord[name] = fromKinectToAbs_coord((trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z))
				#body_coord[name] =
			#except:
		#		print 'not receiving'
		#		continue


		#print body_coord
		#trasformation body coord
		body_coord2 = {}
		body_coord2['head']=body_coord['neck2']
		body_coord2['neck']=[body_coord['shoulder_r1'][0],body_coord['shoulder_r1'][1],body_coord['shoulder_r1'][2]+200]
		body_coord2['torso']=body_coord['shoulder_r1']
		body_coord2['shoulder','l']=body_coord['shoulder_l2']
		body_coord2['elbow','l']=body_coord['upper_arm_l2']
		body_coord2['hand','l']=body_coord['lower_arm_l2']
		body_coord2['hip','l']=body_coord['hip_l2']
		body_coord2['knee','l']=[body_coord['hip_l2'][0],body_coord['hip_l2'][1],body_coord['hip_l2'][2]-400]
		body_coord2['foot','l']=[body_coord['hip_l2'][0],body_coord['hip_l2'][1],body_coord['hip_l2'][2]-800]
		body_coord2['shoulder','r']=body_coord['upper_arm_r1']
		body_coord2['elbow','r']=body_coord['lower_arm_r1']
		body_coord2['hand','r']=body_coord['lower_arm_r2']
		body_coord2['hip','r']=body_coord['hip_r2']
		body_coord2['knee','r']=[body_coord['hip_r2'][0],body_coord['hip_r2'][1],body_coord['hip_r2'][2]-400]
		body_coord2['foot','r']=[body_coord['hip_r2'][0],body_coord['hip_r2'][1],body_coord['hip_r2'][2]-800]


		try:
			Reba=RebaC(body_coord2)
			print('RebaA : {}'.format(Reba[0]))
			print('RebaB : {}'.format(Reba[1]))
			print('RebaC : {}'.format(Reba[2]))
			#publish
			pub_REBA.publish(Reba[2])

		except (ValueError):
			print('fail reba')
			rate.sleep()
