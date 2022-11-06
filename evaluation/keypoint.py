#! /usr/bin/env python
#this file writes the data from the vo topics into txt files after they are transformed in the right coordinate system
#furthermoe it transforms the data of the vo algorithms  in the odometry frame and publishes the transformed data into the topics vop and #vo2 this topics are then used for fusion.
#additionally it takes care of the imu data to get more suitable covariances
#it expects as parameters one parameter identifying if the tfs should be calculated or if it should write down the data of the algorithms which are published
#when getransforms=2
# the second parameter is the tf file where the data for the transformations read from the live topics odom and gt should be stored if you apply changes in the order of the transforms written into the file
# apply the changes also to shelllivedata.sh which reads the file linewise
#when getransforms=1 
# the second parameter is the bag file containing odometry information the second is the groundtruthfile
# the data is written into the file tf.txt 
#when getransforms=0
#  the next four parameters are the four transformations from map into odom from map into base_footprint and from odom to base_footprint and from cameralink to cameracoloropticalframe 
# one of this can be generated from this script when setting getransform to 1 by reading from the optitrackfile which will give us opti->base_footprint
# we then apply base_footprint to odom to get opti->odom 
#the transformations must consist of timestamp(format sec.nsec), x, y, z, qx, qy, qz, qw 
#each vlaue must be separated by a comma and followed by a whitespace
# the last parameter destingues between different algorithms that use the tf topic to publish there data

import tf2_ros
import tf2_geometry_msgs #import the packages first
import rospy
import tf
import rosbag
import numpy as np
import os
import tf2_msgs.msg
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped,TransformStamped
from tf import transformations as t
import sys
import math 
import copy


full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
#https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
def matfromquatfromtf(tf):
	x=tf.transform.rotation.x
	y=tf.transform.rotation.y
	z=tf.transform.rotation.z
	w=tf.transform.rotation.w
	tx=tf.transform.translation.x
	ty=tf.transform.translation.y
	tz=tf.transform.translation.z
	return matfromquat(x,y,z,w,tx,ty,tz)

def matfrompose(pos):
	x=pos.pose.pose.orientation.x
	y=pos.pose.pose.orientation.y
	z=pos.pose.pose.orientation.z
	w=pos.pose.pose.orientation.w
	tx=pos.pose.pose.position.x
	ty=pos.pose.pose.position.y
	tz=pos.pose.pose.position.z
	return matfromquat(x,y,z,w,tx,ty,tz)

def matfromquat(x,y,z,w,tx,ty,tz):
	M=np.zeros((4, 4), dtype=np.float64)
	nom=(w*w + x*x + y*y + z*z)
	w=w/nom;
	x=x/nom;
	y=y/nom;
	z=z/nom;
	M[0,0]=1-2*y*y-2*z*z
	M[0,1]=2*x*y - 2*w*z
	M[0,2]=2*x*z+2*w*y
	M[1,0]=2*x*y+2*w*z
	M[1,1]=1-2*x*x-2*z*z
	M[1,2]=2*y*z-2*w*x
	M[2,0]=2*x*z-2*w*y
	M[2,1]=2*y*z+2*w*x
	M[2,2]=1-2*x*x-2*y*y
	M[3,3]=1
	M[0,3]=tx
	M[1,3]=ty
	M[2,3]=tz
	return M
#https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
def quatfrommat(M):	
	trace=M[0,0] + M[1,1] + M[2,2]
	q=[0,0,0,0]
	if (trace > 0):
		s= 0.5 /math.sqrt(trace+1) 
		q = [ (M[2,1]-M[1,2])*s, (M[0,2]-M[2,0])*s, (M[1,0]-M[0,1])*s,0.25/s]
	else:
		if (M[0,0] >M[1,1] and M[0,0]>M[2,2]):
				s= 2* math.sqrt(1 + M[0,0] -M[1,1] -M[2,2])
				q = [ 0.25*s, (M[0,1]+M[1,0])/s, (M[0,2]+M[2,0])/s, (M[2,1]-M[1,2])/s]
		elif(M[1,1]>M[2,2]):
				s= 2*math.sqrt(1 -M[0,0] + M[1,1] - M[2,2])
				q = [ (M[0,1]+M[1,0])/s, 0.25*s, (M[1,2]+M[2,1])/s, (M[0,2]-M[2,0])/s]
		else:
				s= 2*math.sqrt(1 -M[0,0] -M[1,1] + M[2,2])
				q =[ (M[0,2]+M[2,0])/s, (M[1,2]+M[2,1])/s,0.25*s, (M[1,0]-M[0,1])/s ]		
	return q

def tffromMat(M,stamp,frameid,childframe):
	trans=TransformStamped()
	trans.header.frame_id=frameid
	trans.header.stamp=stamp
	trans.child_frame_id=childframe
	q=quatfrommat(M)
	trans.transform.rotation.x=q[0]
	trans.transform.rotation.y=q[1]
	trans.transform.rotation.z=q[2]
	trans.transform.rotation.w=q[3]
	trans.transform.translation.x=M[0,3]
	trans.transform.translation.y=M[1,3]
	trans.transform.translation.z=M[2,3]
	return trans
def zerotranstf(tf):
	tf.transform.translation.x=0
	tf.transform.translation.y=0
	tf.transform.translation.z=0
def filltf(trans,arg,frame_id,child_id,inv):
	trans.header.stamp=arg[0]
	trans.header.frame_id=frame_id
	trans.child_frame_id=child_id
	trans.transform.translation.x=arg[1]
	trans.transform.translation.y=arg[2]
	trans.transform.translation.z=arg[3]
	multi=1
	if inv == 1:
		multi=-1
	trans.transform.rotation.x=arg[4]*multi
	trans.transform.rotation.y=arg[5]*multi
	trans.transform.rotation.z=arg[6]*multi
	trans.transform.rotation.w=arg[7]*multi
def sign(x): 
	return 1-(x<=0)*2 
def getcor(posCovMsg,prevyaw,corx,cory,siftx,sifty,yawoff):
	angles = tf.transformations.euler_from_quaternion([posCovMsg.pose.orientation.x, posCovMsg.pose.orientation.y, posCovMsg.pose.orientation.z, posCovMsg.pose.orientation.w])
	yawdiff=0
	if(sign(angles[2])==sign(prevyaw)):
		yawdiff=angles[2]-prevyaw
	elif(sign(angles[2])!=sign(prevyaw)) and abs(angles[2]-prevyaw)<3.14:
		yawdiff=angles[2]-prevyaw
	elif(sign(prevyaw)<0) and (sign(angles[2])>0):
		yawdiff=prevyaw+3.1415926535898*2-angles[2]
	if(sign(prevyaw)>0) and (sign(angles[2])<0):
		yawdiff=prevyaw-angles[2]
	#this makes sense because we first calculate the shift currently in x due to the rotation and then the shifted x caused by the newrotationangle 
	#we then subtract the two to get the corection term, to image this draw two coordinate system and the vector between these systems has the lenght of the total shift
	#the rotation steeming from x,y shift
	prevyaw=prevyaw+math.atan2(sifty,siftx)-yawoff
	corx[0]=corx[0]+(math.cos(prevyaw+yawdiff)-math.cos(prevyaw))*(math.sqrt(math.pow(siftx,2)+math.pow(sifty,2)))
	cory[0]=cory[0]+(math.sin(prevyaw+yawdiff)-math.sin(prevyaw))*(math.sqrt(math.pow(siftx,2)+math.pow(sifty,2)))
#latest version
def getinvers2(trans4):	
	trans4r=TransformStamped()
	M=matfromquatfromtf(trans4)
	Minv=np.linalg.inv(M)
	trans4r=tffromMat(Minv,trans4.header.stamp,trans4.child_frame_id,trans4.header.frame_id)
	return trans4r
#check for nan values in data
def checkfornan(value):
	if math.isnan(value):
		transvalue=0
	else:
		transvalue=value
	return transvalue
#generates a tf msg from odom message
def generatetffromodommgs(msg):
	trans4=TransformStamped()
	trans4.header.frame_id="odom"
	trans4.child_frame_id="base_footprint"
	trans4.transform.translation.x=checkfornan(msg.pose.pose.position.x)
	trans4.transform.translation.y=checkfornan(msg.pose.pose.position.y)
	trans4.transform.translation.z=checkfornan(msg.pose.pose.position.z)
	if math.isnan(msg.pose.pose.orientation.x) and math.isnan(msg.pose.pose.orientation.y) and math.isnan(msg.pose.pose.orientation.z) and math.isnan(msg.pose.pose.orientation.w):
		quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
		trans4.transform.rotation.x=quaternion[0]	
		trans4.transform.rotation.y=quaternion[1]	
		trans4.transform.rotation.z=quaternion[2]	
		trans4.transform.rotation.w=quaternion[3]	
	else:
		trans4.transform.rotation.x=checkfornan(msg.pose.pose.orientation.x)
		trans4.transform.rotation.y=checkfornan(msg.pose.pose.orientation.y)
		trans4.transform.rotation.z=checkfornan(msg.pose.pose.orientation.z)
		trans4.transform.rotation.w=checkfornan(msg.pose.pose.orientation.w)
	return trans4
#generates a tf msg from pose message
def generatetffrompose(msg,frame,child):
	trans4=TransformStamped()
	trans4.header.frame_id=frame
	trans4.child_frame_id=child
	trans4.transform.translation.x=checkfornan(msg.pose.position.x)
	trans4.transform.translation.y=checkfornan(msg.pose.position.y)
	trans4.transform.translation.z=checkfornan(msg.pose.position.z)
	if math.isnan(msg.pose.orientation.x) and math.isnan(msg.pose.orientation.y) and math.isnan(msg.pose.orientation.z) and math.isnan(msg.pose.orientation.w):
		quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
		trans4.transform.rotation.x=quaternion[0]	
		trans4.transform.rotation.y=quaternion[1]	
		trans4.transform.rotation.z=quaternion[2]	
		trans4.transform.rotation.w=quaternion[3]	
	else:
		trans4.transform.rotation.x=checkfornan(msg.pose.orientation.x)
		trans4.transform.rotation.y=checkfornan(msg.pose.orientation.y)
		trans4.transform.rotation.z=checkfornan(msg.pose.orientation.z)
		trans4.transform.rotation.w=checkfornan(msg.pose.orientation.w)
	return trans4
#read a file
def read_file(filepath_to_read):
	with open(filepath_to_read) as gt_in:
		return (np.fromstring(gt_in.read(), sep="   ").reshape(-1, 8))
#despite the fact that a pose normally stores the tf in the oppossing direction, this function works with poses as if they store the tf in the given direction 
#the reason is that we use the tf2_geometry_msgs.do_transform_pose to calculate the resulting transformation of two consecutive tfs
#this function takes a pose as the second transformation and the result is still a pose which then can be seen as the resulting tf and be stored as shown belown
def writetotffile(stamp,pose,frame,child):
	with open (path+"/tf.txt","a") as gt_pose_file:
		gt_pose_file.write("header:frame_id: "+frame+" child_frame_id: "+child+" stamp: "+str(format(stamp,'.9f'))+" transform.translation.x: "+str(pose.position.x)+" transform.translation.y: "+str(pose.position.y)+" transform.translation.z: "+str(pose.position.z)+" transform.rotation.x: "+str(pose.orientation.x)+" transform.rotation.y: "+str(pose.orientation.y)+" transform.rotation.z: "+str(pose.orientation.z)+" transform.rotation.w: "+str(pose.orientation.w)+"\n")	
		gt_pose_file.write("for shell "+frame+" to "+child+" : \'\""+str(format(stamp,'.9f'))+", "+str(format(pose.position.x,'.11f'))+", "+str(format(pose.position.y,'.11f'))+", "+str(pose.position.z)+", "+str(format(pose.orientation.x,'.9f'))+", "+str(format(pose.orientation.y,'.9f'))+", "+str(format(pose.orientation.z,'.9f'))+", "+str(format(pose.orientation.w,'.9f'))+"\"\' \n")
#write away initial state
def writetotffileinitalstate(pos):
	angles = tf.transformations.euler_from_quaternion([pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z, pos.pose.orientation.w])
	with open (path+"/tf.txt","a") as gt_pose_file:
		gt_pose_file.write("Initalstate Odom for shell:\'\"["+str(format(pos.pose.position.x,'.9f'))+", "+str(format(pos.pose.position.y,'.9f'))+", 0.0, 0.0, 0.0, "+str(format(angles[2],'.9f'))+", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\"\' \n")
#this is our new tffile containing all the data
def writetospecfiletfandinital(file,inital,stamp,tfposoptitobase,tfposodomtobase,tfposoptitoodom):
	angles = tf.transformations.euler_from_quaternion([inital.pose.orientation.x, inital.pose.orientation.y, inital.pose.orientation.z, inital.pose.orientation.w])
	with open (file,"a+") as gt_pose_file:
		gt_pose_file.write("["+str(format(inital.pose.position.x,'.9f'))+", "+str(format(inital.pose.position.y,'.9f'))+", 0.0, 0.0, 0.0, "+str(format(angles[2],'.9f'))+", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n")	
		gt_pose_file.write("\""+str(format(stamp,'.9f'))+", "+str(format(tfposoptitobase.position.x,'.11f'))+", "+str(format(tfposoptitobase.position.y,'.11f'))+", "+str(format(tfposoptitobase.position.z,'.11f'))+", "+str(format(tfposoptitobase.orientation.x,'.9f'))+", "+str(format(tfposoptitobase.orientation.y,'.9f'))+", "+str(format(tfposoptitobase.orientation.z,'.9f'))+", "+str(format(tfposoptitobase.orientation.w,'.9f'))+"\" \n")
		gt_pose_file.write("\""+str(format(stamp,'.9f'))+", "+str(format(tfposodomtobase.position.x,'.11f'))+", "+str(format(tfposodomtobase.position.y,'.11f'))+", "+str(format(tfposodomtobase.position.z,'.11f'))+", "+str(format(tfposodomtobase.orientation.x,'.9f'))+", "+str(format(tfposodomtobase.orientation.y,'.9f'))+", "+str(format(tfposodomtobase.orientation.z,'.9f'))+", "+str(format(tfposodomtobase.orientation.w,'.9f'))+"\" \n")
		gt_pose_file.write("\""+str(format(stamp,'.9f'))+", "+str(format(tfposoptitoodom.position.x,'.11f'))+", "+str(format(tfposoptitoodom.position.y,'.11f'))+", "+str(format(tfposoptitoodom.position.z,'.11f'))+", "+str(format(tfposoptitoodom.orientation.x,'.9f'))+", "+str(format(tfposoptitoodom.orientation.y,'.9f'))+", "+str(format(tfposoptitoodom.orientation.z,'.9f'))+", "+str(format(tfposoptitoodom.orientation.w,'.9f'))+"\" \n")
#this adds to the tffile the tf into imu cam and lidar
def writetfimucamlidartofile(file,stamp,tftoimu,tftocam,tftolidar):
	with open (file,"a") as gt_pose_file:
		gt_pose_file.write("\""+str(format(stamp,'.9f'))+", "+str(format(tftoimu.transform.translation.x,'.11f'))+", "+str(format(tftoimu.transform.translation.y,'.11f'))+", "+str(format(tftoimu.transform.translation.z,'.11f'))+", "+str(format(tftoimu.transform.rotation.x,'.9f'))+", "+str(format(tftoimu.transform.rotation.y,'.9f'))+", "+str(format(tftoimu.transform.rotation.z,'.9f'))+", "+str(format(tftoimu.transform.rotation.w,'.9f'))+"\" \n")
		gt_pose_file.write("\""+str(format(stamp,'.9f'))+", "+str(format(tftocam.transform.translation.x,'.11f'))+", "+str(format(tftocam.transform.translation.y,'.11f'))+", "+str(format(tftocam.transform.translation.z,'.11f'))+", "+str(format(tftocam.transform.rotation.x,'.9f'))+", "+str(format(tftocam.transform.rotation.y,'.9f'))+", "+str(format(tftocam.transform.rotation.z,'.9f'))+", "+str(format(tftocam.transform.rotation.w,'.9f'))+"\" \n")
		gt_pose_file.write("\""+str(format(stamp,'.9f'))+", "+str(format(tftolidar.transform.translation.x,'.11f'))+", "+str(format(tftolidar.transform.translation.y,'.11f'))+", "+str(format(tftolidar.transform.translation.z,'.11f'))+", "+str(format(tftolidar.transform.rotation.x,'.9f'))+", "+str(format(tftolidar.transform.rotation.y,'.9f'))+", "+str(format(tftolidar.transform.rotation.z,'.9f'))+", "+str(format(tftolidar.transform.rotation.w,'.9f'))+"\" \n")
#remove a file
def removefile(name):
       try:
		os.remove(name)
		print(name+" removed!")
       except:
		print (name+" was not there")
#it will only use the rotation yaw part of the pose
def writeposetofile(filename,posetowrite,fromcam):
	angles = tf.transformations.euler_from_quaternion([posetowrite.pose.orientation.x, posetowrite.pose.orientation.y, posetowrite.pose.orientation.z, posetowrite.pose.orientation.w])
	#necessary because the visual camera system applied a -3.14 ... rotation we reverse it here 
	additionalrot=0
	if fromcam == 1:
		additionalrot=+3.1415926535897931/2
	quat = tf.transformations.quaternion_from_euler(0,0,angles[2]+additionalrot) 
	qx =   quat[0]   
	qy =   quat[1]  
	qz =   quat[2]  
	qw =   quat[3] 
	writetofile(filename,posetowrite.header.stamp,posetowrite.pose.position.x,posetowrite.pose.position.y,posetowrite.pose.position.z,qx,qy,qz,qw)
#write to a file x,y,z etc data
def writetofile(filename,stamp,x,y,z,qx,qy,qz,qw):
		with open (path+"/"+filename,"a") as odom_pose_file:
			odom_pose_file.write(str(stamp.secs)+"."+str(stamp.nsecs).zfill(9)+ "   "+str(format(x,'.11f'))+ "   " +str(format(y,'.11f'))+"   "+str(format(z,'.11f'))+"   "+str(format(qx,'.11f'))+"   "+str(format(qy,'.11f'))+"   "+str(format(qz,'.11f'))+"   "+str(format(qw,'.11f'))+"\n")
		print("Position for "+filename+": "+str(x)+" "+str(y)+" "+str(z)+" "+str(qx)+" "+str(qy)+" "+str(qz)+" "+str(qw))
#workaround cause our writetotffile function only takes pose message to store the trans and rot data of our transformation generateposefromtf
def filltfintoposewithoutinver(tf):
	odomtfaspose = PoseStamped()
	odomtfaspose.header.seq=1
	odomtfaspose.header.frame_id=tf.header.frame_id
	odomtfaspose.header.stamp=tf.header.stamp
	odomtfaspose.pose.position.x=tf.transform.translation.x
	odomtfaspose.pose.position.y=tf.transform.translation.y
	odomtfaspose.pose.position.z=tf.transform.translation.z
	odomtfaspose.pose.orientation.x=tf.transform.rotation.x
	odomtfaspose.pose.orientation.y=tf.transform.rotation.y
	odomtfaspose.pose.orientation.z=tf.transform.rotation.z
	odomtfaspose.pose.orientation.w=tf.transform.rotation.w
	return odomtfaspose
#generates and publishes a message in odometry frame from a pose message in base footprint frame if cam is 0 the message does not come from realsense cov is the covariance matrix
def genandpubposewcovinodomfrombasefootmsg(tfodomtobase,msg,cam,cov,pub1):
	posCovMsg = PoseWithCovarianceStamped() 
	pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, tfodomtobase)
	posCovMsg.header.frame_id = "odom"
	posCovMsg.header.stamp = msg.header.stamp
	angles = tf.transformations.euler_from_quaternion([pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w])
	additionalrot=0
	if cam == 1:
		additionalrot=+3.1415926535897931/2
	quat = tf.transformations.quaternion_from_euler(0,0,angles[2]+additionalrot)	  
	posCovMsg.pose.pose.position = pose_transformed.pose.position
	posCovMsg.pose.pose.orientation.x = quat[0]
	posCovMsg.pose.pose.orientation.y = quat[1]
	posCovMsg.pose.pose.orientation.z = quat[2]
	posCovMsg.pose.pose.orientation.w = quat[3]	
	posCovMsg.pose.covariance[0:36]=cov
	pub1.publish(posCovMsg)
#generates and publishes a message in odometry frame if cam is 0 the message does not come from realsense cov is the covariance matrix
def genandpubposewcovinodomfromodompose(msg,cam,cov,pub1):
	posCovMsg = PoseWithCovarianceStamped() 
	angles = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
	additionalrot=0
	if cam == 1:
		additionalrot=+3.1415926535897931/2
	quat = tf.transformations.quaternion_from_euler(0,0,angles[2]+additionalrot) 
	posCovMsg.pose.pose.orientation.x = quat[0]
	posCovMsg.pose.pose.orientation.y = quat[1]
	posCovMsg.pose.pose.orientation.z = quat[2]
	posCovMsg.pose.pose.orientation.w = quat[3]	
	posCovMsg.pose.covariance[0:36]=cov
	posCovMsg.header.stamp=msg.header.stamp
	posCovMsg.header.frame_id="odom"
	posCovMsg.pose.pose.position=msg.pose.position
	pub1.publish(posCovMsg)
#the gettransform function for generation of a tf.txt file based on a rosbag and ground truth file
def gettransform():
	gt_data = read_file(sys.argv[3])
	optitobase=TransformStamped()
	print(gt_data[0])
	filltf(optitobase,gt_data[0],"gt","base_footprint",1)
	#get initial state of odom and odomtobase transformation
	odomtobase=TransformStamped()
	initalpose=PoseStamped()
	odomtopicfound=0
	for topic, msg, t in rosbag.Bag(sys.argv[2]).read_messages():
		if topic == "/odom" or topic == "odom" or topic == "odomc" or topic == "/odomc":
			initalpose=msg.pose
			#we later want to store this tf information in form of a tf no need to inverse it with filltfintoposewithoutinver
			odomtobase = generatetffromodommgs(msg)
			odomtopicfound=1
			break
	if odomtopicfound != 1:
		print("Bag contained no Odometry Data using 0 value")
		zeroodom = Odometry()
		initalpose=zeroodom.pose
		zeroodom.pose.pose.position.x= 0
		zeroodom.pose.pose.position.y= 0
		zeroodom.pose.pose.position.z= 0
		zeroodom.pose.pose.orientation.x = 0
		zeroodom.pose.pose.orientation.y = 0
		zeroodom.pose.pose.orientation.z = 0
		zeroodom.pose.pose.orientation.w = 1
		odomtobase=generatetffromodommgs(zeroodom)
	#now we want opti to base  
	print(odomtobase)
	optibasepose=filltfintoposewithoutinver(optitobase) 
	#now we want odom to base   
	odomtfaspose=filltfintoposewithoutinver(odomtobase)
	#we want opti to odom
	basetoodom=getinvers2(odomtobase)
	M2=matfromquatfromtf(optitobase)
	M1=matfromquatfromtf(basetoodom)
	tfoptitoodom=tffromMat(M2.dot(M1),optibasepose.header.stamp,"opti","odom")
	tfoptiodompose=filltfintoposewithoutinver(tfoptitoodom)
	'''testing
	posCovMsg = PoseWithCovarianceStamped() 
	posCovMsg.pose.pose.orientation.x=0
	posCovMsg.pose.pose.orientation.y=0
	posCovMsg.pose.pose.orientation.z=0
	posCovMsg.pose.pose.orientation.w=1
	posM=matfrompose(posCovMsg)
	Moptod=M2.dot(M1)
	Motb=matfromquatfromtf(odomtobase)
	print(Moptod.dot(Motb.dot(posM)))
	print(M2.dot(posM))''' 
	tffile = sys.argv[4]
	removefile(tffile)
	writetospecfiletfandinital(tffile,initalpose,gt_data[0,0],optibasepose.pose,odomtfaspose.pose,tfoptiodompose.pose)
	quit()
#for live data analysis
class getLive(object):
	def __init__(self,tffile):
		self._sub = rospy.Subscriber("/gt", PoseStamped, self.writeamcl)
		self._sub = rospy.Subscriber("/odomc", Odometry, self.writeamcl2)
		self._sub = rospy.Subscriber("/odom", Odometry, self.writeamcl2)
		self._sub = rospy.Subscriber("/tf_static", tf2_msgs.msg.TFMessage, self.writeamcl3)
		self.optitobase=TransformStamped()
		self.initalpose=PoseStamped()
		self.odomtobase=TransformStamped()
		self.gtrev=False
		self.finishgtodom=False
		self.odomrev=False  
		self.finishwriting=False
		self.tffile=tffile
		self.writing=False
	def writeamcl(self,posecov):
		if self.gtrev == False:
			gt_data=[float(str(posecov.header.stamp.secs)+"."+str(posecov.header.stamp.nsecs).zfill(9)),posecov.pose.position.x,posecov.pose.position.y,posecov.pose.position.z,
				posecov.pose.orientation.x,posecov.pose.orientation.y,posecov.pose.orientation.z,posecov.pose.orientation.w]
			filltf(self.optitobase,gt_data,"gt","base_footprint",1) 
			self.gtrev=True
		if self.odomrev == True:
			self.gettransformlive()
	def writeamcl2(self,posecov):
		if self.odomrev == False:
    			self.initalpose=posecov.pose
			#we later want to store this tf information in form of a tf no need to inverse it with filltfintoposewithoutinver
    			self.odomtobase = generatetffromodommgs(posecov)
			self.odomrev=True
	 	if self.gtrev == True:
			self.gettransformlive() 
	def writeamcl3(self,posecov):
		#print("In tf_static",self.finishgtodom)
		tftoimu=TransformStamped()
		tftocam=TransformStamped()
		tftolidar=TransformStamped()
		i=0
		while i<len(posecov.transforms):
			if posecov.transforms[i].header.frame_id == "base_footprint" and posecov.transforms[i].child_frame_id=="camera_link":	
				tftocam=posecov.transforms[i]
			if posecov.transforms[i].header.frame_id == "base_footprint" and posecov.transforms[i].child_frame_id=="scan_front":	
				tftolidar=posecov.transforms[i]
			if posecov.transforms[i].header.frame_id == "base_footprint" and posecov.transforms[i].child_frame_id=="imu":	
				tftoimu=posecov.transforms[i]
			i=i+1
		if self.finishgtodom == True and self.finishwriting==False:
			self.finishwriting=True
			writetfimucamlidartofile(self.tffile,self.optitobase.header.stamp,tftoimu,tftocam,tftolidar)
			print("Finished Writing")
			quit()
	def gettransformlive(self):
 		print("In gettransformlive")
		if self.finishgtodom==False and self.writing == False:
			self.writing=True
		optibasepose=filltfintoposewithoutinver(self.optitobase) 
		#now we want odom to base   
		odomtfaspose=filltfintoposewithoutinver(self.odomtobase)
		#we want opti to odom
		basetoodom=getinvers2(self.odomtobase)
		M1=matfromquatfromtf(basetoodom)
		M2=matfromquatfromtf(self.optitobase)
		tfoptitoodom=tffromMat(M2.dot(M1),optibasepose.header.stamp,"opti","odom")
		tfoptiodompose=filltfintoposewithoutinver(tfoptitoodom)  
		removefile(self.tffile)
		writetospecfiletfandinital(self.tffile,self.initalpose,self.optitobase.header.stamp,optibasepose.pose,odomtfaspose.pose,tfoptiodompose.pose)
		self.finishgtodom=True
#old versioncallback for amcl
class Amcl(object):
	def __init__(self):
		self._sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.writeamcl)
	def writeamcl(self,posecov):
		stamp= posecov.header.stamp 
		x_pos = posecov.pose.pose.position.x
		y_pos = posecov.pose.pose.position.y
		z_pos = posecov.pose.pose.position.z
		angles = tf.transformations.euler_from_quaternion([posecov.pose.pose.orientation.x, posecov.pose.pose.orientation.y, posecov.pose.pose.orientation.z, posecov.pose.pose.orientation.w])
		quat = tf.transformations.quaternion_from_euler(0,0,angles[2]) 
		q_x =   quat[0]   
		q_y =   quat[1]  
		q_z =   quat[2]  
		q_w =   quat[3]
		writetofile("amcl.txt",stamp,x_pos,y_pos,z_pos,q_x,q_y,q_z,q_w)
#callback for hector
class Hector(object):
	def __init__(self,tfmaptopose,tfodomtobase,tfbasetolidar,mal):
		removefile(path+"/hector.txt")
		self._sub = rospy.Subscriber("/slam_out_pose", PoseStamped, self.writehector)
		self.tfmaptopose=tfmaptopose
		self.tfodomtobase=tfodomtobase
		self.tfbasetolidar=tfbasetolidar
		angles = tf.transformations.euler_from_quaternion([tfbasetolidar.transform.rotation.x, tfbasetolidar.transform.rotation.y, tfbasetolidar.transform.rotation.z, 		tfbasetolidar.transform.rotation.w])
		self.yawoff=angles[2]	
		self.pub1 = rospy.Publisher('/lpose',PoseWithCovarianceStamped,queue_size=10)
		self.cor=1
		if mal == 1:
			self.cor=100 
	def writehector(self,posecov):
		#hector already publishes the data in the base_footprint system so transformation with tfbasetolidar is not necessary
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,posecov,1,[1.5*self.cor,0,0,0,0,0,
		0,1.5*self.cor,0,0,0,0,
		0,0,1.5*self.cor,0,0,0,
		0,0,0,1.5*self.cor,0,0,
		0,0,0,0,1.5*self.cor,0,
		0,0,0,0,0,1.5*self.cor],self.pub1)	
		postr = tf2_geometry_msgs.do_transform_pose(posecov, self.tfmaptopose)
		postr.header.stamp=posecov.header.stamp
		angles = tf.transformations.euler_from_quaternion([postr.pose.orientation.x, postr.pose.orientation.y, postr.pose.orientation.z, postr.pose.orientation.w])
		q = tf.transformations.quaternion_from_euler(angles[0], angles[1],angles[2]-self.yawoff)		
		postr.pose.orientation.x=q[0]
		postr.pose.orientation.y=q[1]
		postr.pose.orientation.z=q[2]
		postr.pose.orientation.w=q[3]
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		writeposetofile("hector.txt",postr,0)
#for gmap amcl karto and mrpt
class GmapandAmclandKartoandMRPT(object):
	#k is provided from the shell script and corespondence there with a algorihtm
	def __init__(self,k,tfmaptoodom,tfbasetolidar,mal):
		self._sub = rospy.Subscriber("/tf",tf2_msgs.msg.TFMessage,self.writegmap)
		self.pub1 = rospy.Publisher('/lpose',PoseWithCovarianceStamped,queue_size=10) 
		self._first=0   
		self.tfmaptoodom=tfmaptoodom
		self.ownmaptoodomtf=TransformStamped() 
		self.firstinvownmaptoodomtf=TransformStamped() 
		self.filetowrite = ""
		if int(k) == 2:
			removefile(path+"/amcl.txt")
			self.filetowrite = "amcl.txt"
		elif int(k) == 8:
			removefile(path+"/karto.txt")
			self.filetowrite = "karto.txt"
		elif int(k) == 10:
			removefile(path+"/mrpt.txt")
			self.filetowrite = "mrpt.txt"
		elif int(k) == 13:
			removefile(path+"/mrpticpslam.txt")
			self.filetowrite = "mrpticpslam.txt"
		else:
			removefile(path+"/gmap.txt")	 
			self.filetowrite = "gmap.txt"
		angles = tf.transformations.euler_from_quaternion([tfbasetolidar.transform.rotation.x, tfbasetolidar.transform.rotation.y, tfbasetolidar.transform.rotation.z, 		tfbasetolidar.transform.rotation.w])
		self.yawoff=angles[2]		
		self.cor=1
		if mal== 3 or mal== 2 or mal == 0:
			self.cor=1000
	#callback for writegmap
	def writegmap(self,gt_msg):
		i=0
		while i<len(gt_msg.transforms):
			if gt_msg.transforms[i].header.frame_id == "map" and gt_msg.transforms[i].child_frame_id=="odom":
				self.ownmaptoodomtf=gt_msg.transforms[i]
				#we need the tf into the map of the algo and the tf back from it but only at the first point in time
				angles = tf.transformations.euler_from_quaternion([self.ownmaptoodomtf.transform.rotation.x, self.ownmaptoodomtf.transform.rotation.y, self.ownmaptoodomtf.transform.rotation.z, self.ownmaptoodomtf.transform.rotation.w])
				q = tf.transformations.quaternion_from_euler(angles[0], angles[1],angles[2])
				self.ownmaptoodomtf.transform.rotation.x=q[0]
				self.ownmaptoodomtf.transform.rotation.y=q[1]
				self.ownmaptoodomtf.transform.rotation.z=q[2]
				self.ownmaptoodomtf.transform.rotation.w=q[3]
				if self._first == 0:
					self.firstinvownmaptoodomtf=getinvers2(gt_msg.transforms[i])
					print self.firstinvownmaptoodomtf
				self._first=1
			if gt_msg.transforms[i].header.frame_id == "odom" and gt_msg.transforms[i].child_frame_id=="base_footprint" and self._first != 0:
				#we want our data in pose so we need to apply another transform code still needed here
				posCovMsg=filltfintoposewithoutinver(gt_msg.transforms[i])
				posCovMsg = tf2_geometry_msgs.do_transform_pose(posCovMsg, self.ownmaptoodomtf)
				posCovMsg = tf2_geometry_msgs.do_transform_pose(posCovMsg, self.firstinvownmaptoodomtf)
				#data in base_footprint
				genandpubposewcovinodomfromodompose(posCovMsg,1,[1.5*self.cor,0,0,0,0,0,
				0,1.5*self.cor,0,0,0,0,
				0,0,1.5*self.cor,0,0,0,
				0,0,0,1.5*self.cor,0,0,
				0,0,0,0,1.5*self.cor,0,
				0,0,0,0,0,1.5*self.cor],self.pub1)
				#we now transform into the globalmap system
				posCovMsg = tf2_geometry_msgs.do_transform_pose(posCovMsg, self.tfmaptoodom) 
				angles = tf.transformations.euler_from_quaternion([posCovMsg.pose.orientation.x, posCovMsg.pose.orientation.y, posCovMsg.pose.orientation.z, posCovMsg.pose.orientation.w])
				q = tf.transformations.quaternion_from_euler(angles[0], angles[1],angles[2]-self.yawoff)
				posCovMsg.pose.orientation.x=q[0]
				posCovMsg.pose.orientation.y=q[1]
				posCovMsg.pose.orientation.z=q[2]
				posCovMsg.pose.orientation.w=q[3]
				posCovMsg.header.stamp = gt_msg.transforms[i].header.stamp
				writeposetofile(self.filetowrite,posCovMsg,0)
			i=i+1
#for ekf
class Ekf(object):
	def __init__(self,tfmaptoodom):
		removefile(path+"/ekf_pose.txt")
		self._sub = rospy.Subscriber("/odometry/filtered",Odometry,self.writeEKFOdomToFile)
		self.tfmaptoodom=tfmaptoodom
	#callback for filtered ekf
	def writeEKFOdomToFile(self,ekf_odom_msg):
		pose_transformed = tf2_geometry_msgs.do_transform_pose(ekf_odom_msg.pose, self.tfmaptoodom)
		pose_transformed.header.stamp = ekf_odom_msg.header.stamp
		writeposetofile("ekf_pose.txt",pose_transformed,0)
#orbslam
class Orbslam(object):
	def __init__(self,tfmaptopose,tfodomtobase,mal,trans4):
		removefile(path+"/visual_odom_orb.txt")
		self._sub = rospy.Subscriber("/orb_slam2_rgbd/pose", PoseStamped,self.writeOrbslam)
		self.tfmaptopose=tfmaptopose
		self.tfodomtobase=tfodomtobase
		self.trans4 = copy.deepcopy(trans4)  
		self.trans4.transform.translation.x=0
		self.trans4.transform.translation.y=0
		self.trans4.transform.translation.z=0
		self.pub1 = rospy.Publisher('/vop',PoseWithCovarianceStamped,queue_size=10)
		self.cor=100
		if mal== 6:
			self.cor=100
	#callback for orbslam
	def writeOrbslam(self,poseStamped_msg):	
		'''Die map data gehen davon aus das 0 dort liegt wo der baselink war wurden wir nun die Transformation unserer daten aus dem camera system in 
		dieses System zunaechst vornehmen haetten wir die Position der Camera im Map system. Fuehren wir trans2 aus so erhalten wir die Aussage wo sich unsere 
		Kamera zum Zeitpunkt 0 relativ zum Roboter befindet. Wir wollen aber die Position des Roboters vergleichen und diese soll am Anfang bei tfmaptopose liegen. 
		Wollten wir also die Daten miteinander vergleichen muessen wir entweder auf alle Daten trans2 anwenden, damit sie sich alle auf das selbe System beziehen oder 
		wir lassen fuer alle trans2 weg. 
		Ebenso ist es wichtig das in der Transformation vom camera_link ins cameracoloroptical frame keine Verschiebungen vorhanden sind da diese keine Verschiebungen enthaelt 
		sonst muesste man diese ebenfalls auf alle Daten anwenden'''  
		print(poseStamped_msg)
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(poseStamped_msg, self.trans4)
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(pose_transformed2, self.tfmaptopose)
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[1.5*self.cor,0,0,0,0,0,
		0,1.5*self.cor,0,0,0,0,
		0,0,1.5*self.cor,0,0,0,
		0,0,0,1.5*self.cor,0,0,
		0,0,0,0,1.5*self.cor,0,
		0,0,0,0,0,1.5*self.cor],self.pub1)
		pose_transformed2.header.stamp = poseStamped_msg.header.stamp
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		writeposetofile("visual_odom_orb.txt",pose_transformed2,0)
#ohm slam
class Ohm(object):
	def __init__(self,tfmaptopose,tfodomtobase,tfbasetolidar):
		removefile(path+"/ohmtsd.txt")
		self._sub = rospy.Subscriber("/pose", PoseStamped,self.writeohm)	
		self.tfmaptopose=tfmaptopose
		self.tfodomtobase=tfodomtobase	
		self.tfbasetolidar=tfbasetolidar 
		self.tfbaselidaronlyrot=copy.deepcopy(tfbasetolidar)
		zerotranstf(self.tfbaselidaronlyrot)
		self.pub1 = rospy.Publisher('/lpose',PoseWithCovarianceStamped,queue_size=10)  
		self.prevyaw=0
		self.first=1
		self.corx=[0.0]
		self.cory=[0.0]
		angles = tf.transformations.euler_from_quaternion([tfbasetolidar.transform.rotation.x, tfbasetolidar.transform.rotation.y, tfbasetolidar.transform.rotation.z, tfbasetolidar.transform.rotation.w])
		self.yawoff=angles[2]
	#callback for ohm
	def writeohm(self,posecov):
		postrans = tf2_geometry_msgs.do_transform_pose(posecov, self.tfbaselidaronlyrot)
		angles = tf.transformations.euler_from_quaternion([postrans.pose.orientation.x, postrans.pose.orientation.y, postrans.pose.orientation.z, postrans.pose.orientation.w])
		if self.first == 0:
			getcor(postrans,self.prevyaw,self.corx,self.cory,self.tfbasetolidar.transform.translation.x,self.tfbasetolidar.transform.translation.y,self.yawoff)
			print(self.corx[0],self.cory[0])
		self.first=0	    
		postrans.pose.position.x=postrans.pose.position.x-self.corx[0]
		postrans.pose.position.y=postrans.pose.position.y-self.cory[0]
		self.prevyaw=angles[2]
		quat = tf.transformations.quaternion_from_euler(0,0,angles[2]-self.yawoff) 
		postrans.pose.orientation.x=quat[0]
		postrans.pose.orientation.y=quat[1]
		postrans.pose.orientation.z=quat[2]
		postrans.pose.orientation.w=quat[3]
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(postrans, self.tfmaptopose)    
		pose_transformed2.header.stamp=posecov.header.stamp
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[3,0,0,0,0,0,
		0,3,0,0,0,0,
		0,0,3,0,0,0,
		0,0,0,3,0,0,
		0,0,0,0,3,0,
		0,0,0,0,0,10],self.pub1)
		writeposetofile("ohmtsd.txt",pose_transformed2,0)
#for rf2o adjust it to match rostopic where data is published
class Rf2o(object):
	def __init__(self,tfmaptopose,tfodomtobase,tfbasetolidar):
		self.filename="rf2o.txt"
		removefile(path+"/"+self.filename)
		self._sub = rospy.Subscriber("/odom_rf2o", Odometry,self.writerf2o)
		self.tfmaptopose=tfmaptopose
		self.tfodomtobase=tfodomtobase	
		self.tfbasetolidar=tfbasetolidar 
		self.tfbaselidaronlyrot=copy.deepcopy(tfbasetolidar)
		zerotranstf(self.tfbaselidaronlyrot)
		self.pub1 = rospy.Publisher('/lpose',PoseWithCovarianceStamped,queue_size=10)  
		self.prevyaw=0
		self.first=1
		self.corx=[0.0]
		self.cory=[0.0]
		angles = tf.transformations.euler_from_quaternion([tfbasetolidar.transform.rotation.x, tfbasetolidar.transform.rotation.y, tfbasetolidar.transform.rotation.z, tfbasetolidar.transform.rotation.w])
		self.yawoff=angles[2]
	def writerf2o(self,posecov):
		postrans = tf2_geometry_msgs.do_transform_pose(posecov.pose, self.tfbaselidaronlyrot)
		angles = tf.transformations.euler_from_quaternion([postrans.pose.orientation.x, postrans.pose.orientation.y, postrans.pose.orientation.z, postrans.pose.orientation.w])
		if self.first == 0:
			getcor(postrans,self.prevyaw,self.corx,self.cory,self.tfbasetolidar.transform.translation.x,self.tfbasetolidar.transform.translation.y,self.yawoff)
			print(self.corx[0],self.cory[0])
		self.first=0
		postrans.pose.position.x=postrans.pose.position.x-self.corx[0]
		postrans.pose.position.y=postrans.pose.position.y-self.cory[0]
		self.prevyaw=angles[2]
		quat = tf.transformations.quaternion_from_euler(0,0,angles[2]-self.yawoff) 
		postrans.pose.orientation.x=quat[0]
		postrans.pose.orientation.y=quat[1]
		postrans.pose.orientation.z=quat[2]
		postrans.pose.orientation.w=quat[3]
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(postrans, self.tfmaptopose)    
		pose_transformed2.header.stamp=posecov.header.stamp
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[3,0,0,0,0,0,
		0,3,0,0,0,0,
		0,0,3,0,0,0,
		0,0,0,3,0,0,
		0,0,0,0,3,0,
		0,0,0,0,0,10],self.pub1)
		writeposetofile(self.filename,pose_transformed2,0)
#for laserscanmatcher adjust it to match rostopic where data is published
class Laserscanmatcher(object):
	def __init__(self,tfmaptopose,tfodomtobase,tfbasetolidar):
		self.filename="laserscan.txt"
		removefile(path+"/"+self.filename)
		self._sub = rospy.Subscriber("/pose_stamped", PoseStamped,self.writelasma)
		self.tfmaptopose=tfmaptopose
		self.tfodomtobase=tfodomtobase
		self.tfbasetolidar=tfbasetolidar
		self.tfbaselidaronlyrot=copy.deepcopy(tfbasetolidar)
		zerotranstf(self.tfbaselidaronlyrot)
		self.pub1 = rospy.Publisher('/lpose',PoseWithCovarianceStamped,queue_size=10)  
		self.prevyaw=0
		self.first=1
		self.corx=[0.0]
		self.cory=[0.0]
		angles = tf.transformations.euler_from_quaternion([tfbasetolidar.transform.rotation.x, tfbasetolidar.transform.rotation.y, tfbasetolidar.transform.rotation.z, tfbasetolidar.transform.rotation.w])
		self.yawoff=angles[2]
	def writelasma(self,posecov):
		postrans = tf2_geometry_msgs.do_transform_pose(posecov, self.tfbaselidaronlyrot)
		angles = tf.transformations.euler_from_quaternion([postrans.pose.orientation.x, postrans.pose.orientation.y, postrans.pose.orientation.z, postrans.pose.orientation.w])
		if self.first == 0:
			getcor(postrans,self.prevyaw,self.corx,self.cory,self.tfbasetolidar.transform.translation.x,self.tfbasetolidar.transform.translation.y,self.yawoff)
			print(self.corx[0],self.cory[0])
		self.first=0	    
		postrans.pose.position.x=postrans.pose.position.x-self.corx[0]
		postrans.pose.position.y=postrans.pose.position.y-self.cory[0]
		self.prevyaw=angles[2]
		quat = tf.transformations.quaternion_from_euler(0,0,angles[2]-self.yawoff) 
		postrans.pose.orientation.x=quat[0]
		postrans.pose.orientation.y=quat[1]
		postrans.pose.orientation.z=quat[2]
		postrans.pose.orientation.w=quat[3]
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(postrans, self.tfmaptopose)    
		pose_transformed2.header.stamp=posecov.header.stamp
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[3,0,0,0,0,0,
		0,3,0,0,0,0,
		0,0,3,0,0,0,
		0,0,0,3,0,0,
		0,0,0,0,3,0,
		0,0,0,0,0,10],self.pub1)
		writeposetofile(self.filename,pose_transformed2,0)
#pslam
class PASlam(object):
	def __init__(self,tfmaptopose,tfodomtobase):
		removefile(path+"/pa_slam.txt")
		self._sub = rospy.Subscriber("/paslam/path", Path,self.writepaslam)
		self.tfmaptopose=tfmaptopose
		self.tfodomtobase=tfodomtobase
		self.pub1 = rospy.Publisher('/lpose',PoseWithCovarianceStamped,queue_size=10)   
	def writepaslam(self,pathmsg):	
		'''Die map data gehen davon aus das 0 dort liegt wo der baselink war wurden wir nun die Transformation unserer daten aus dem camera system in 
		dieses System zunaechst vornehmen haetten wir die Position der Camera im Map system. Fuehren wir trans2 aus so erhalten wir die Aussage wo sich unsere 
		Kamera zum Zeitpunkt 0 relativ zum Roboter befindet. Wir wollen aber die Position des Roboters vergleichen und diese soll am Anfang bei tfmaptopose liegen. 
		Wollten wir also die Daten miteinander vergleichen muessen wir entweder auf alle Daten trans2 anwenden, damit sie sich alle auf das selbe System beziehen oder 
		wir lassen fuer alle trans2 weg. 
		Ebenso ist es wichtig das in der Transformation vom camera_link ins cameracoloroptical frame keine Verschiebungen vorhanden sind da diese keine Verschiebungen enthaelt 
		sonst muesste man diese ebenfalls auf alle Daten anwenden'''   
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(pathmsg.poses[0], self.tfmaptopose)    
		pose_transformed2.header.stamp = pathmsg.header.stamp       
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[3,0,0,0,0,0,
		0,3,0,0,0,0,
		0,0,3,0,0,0,
		0,0,0,3,0,0,
		0,0,0,0,3,0,
		0,0,0,0,0,10],self.pub1)
		writeposetofile("pa_slam.txt",pose_transformed2,1)
#old version rather use the tf
class MRPT(object):
	#k is provided from the shell script and corespondence there with a algorihtm
	def __init__(self,k,tfmaptopose,tfodomtobase):
		removefile(path+"/mrpt.txt")
		self._sub = rospy.Subscriber("/mrpt_pose", PoseWithCovarianceStamped,self.writemrpt)
		self.tfmaptopose=tfmaptopose
		self.tfodomtobase=tfodomtobase
		self.pub1 = rospy.Publisher('/lpose',PoseWithCovarianceStamped,queue_size=10)
		self.firstinvownmaptoposetf=TransformStamped() 
		self._first=0      
	def writemrpt(self,posecovmsg):
		if self._first == 0:
			tfmrtp=generatetffrompose(posecovmsg.pose,"map","mrptpose")
			self.firstinvownmaptoposetf=getinvers2(tfmrtp) 
			self._first = 1
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(posecovmsg.pose, self.firstinvownmaptoposetf)
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(pose_transformed2, self.tfmaptopose)
		pose_transformed2.header.stamp = posecovmsg.header.stamp
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,posecovmsg.pose.covariance,self.pub1)
		writeposetofile("mrpt.txt",pose_transformed2,1)	
#lsdslam
class LSDSLAM(object):
	#k is provided from the shell script and corespondence there with a algorihtm
	def __init__(self,tfmaptopose,tfodomtobase,tfcamtoopt):
		self.filename="/lsdslam.txt"
		removefile(path+self.filename)
		self._sub = rospy.Subscriber("/lsd_slam/pose",PoseStamped,self.writemrpt)
		self.tfmaptopose=tfmaptopose
		self.tfcamtoopt=tfcamtoopt
		self.tfodomtobase=tfodomtobase
		self.pub1 = rospy.Publisher('/lpose',PoseWithCovarianceStamped,queue_size=10)
	def writemrpt(self,posecovmsg):
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(posecovmsg, self.tfcamtoopt)    
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(pose_transformed2, self.tfmaptopose)    
		pose_transformed2.header.stamp = posecovmsg.header.stamp       
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[3,0,0,0,0,0,
		0,3,0,0,0,0,
		0,0,3,0,0,0,
		0,0,0,3,0,0,
		0,0,0,0,3,0,
		0,0,0,0,0,10],self.pub1)
		writeposetofile(self.filename,pose_transformed2,1)
#monovo
class Monovo(object):
	#k is provided from the shell script and corespondence there with a algorihtm
	def __init__(self,tfmaptopose,tfodomtobase,tfcamtoopt):
		self.filename="/monovo.txt"
		removefile(path+self.filename)
		self._sub = rospy.Subscriber("/floor_cam",PoseStamped,self.writemrpt)
		self.tfmaptopose=tfmaptopose
		self.tfcamtoopt=tfcamtoopt
		self.tfodomtobase=tfodomtobase
		self.pub1 = rospy.Publisher('/monovo',PoseWithCovarianceStamped,queue_size=10)
	#callback for monovo
	def writemrpt(self,posecovmsg):
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(posecovmsg, self.tfcamtoopt)    
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(pose_transformed2, self.tfmaptopose)    
		pose_transformed2.header.stamp = posecovmsg.header.stamp       
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[3,0,0,0,0,0,
		0,3,0,0,0,0,
		0,0,3,0,0,0,
		0,0,0,3,0,0,
		0,0,0,0,3,0,
		0,0,0,0,0,10],self.pub1)
		writeposetofile(self.filename,pose_transformed2,1)
#viso2
class viso2(object):
	#k is provided from the shell script and corespondence there with a algorihtm
	def __init__(self,tfmaptopose,tfodomtobase,tfcamtoopt):
		self.filename="/viso2.txt"
		removefile(path+self.filename)
		self._sub = rospy.Subscriber("/mono_odometer/pose",PoseStamped,self.writemrpt)
		self.tfmaptopose=tfmaptopose
		self.tfcamtoopt=tfcamtoopt
		self.tfodomtobase=tfodomtobase
		self.pub1 = rospy.Publisher('/monovo',PoseWithCovarianceStamped,queue_size=10)
	def writemrpt(self,posecovmsg):
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(posecovmsg, self.tfmaptopose)    
		pose_transformed2.header.stamp = posecovmsg.header.stamp       
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[3,0,0,0,0,0,
		0,3,0,0,0,0,
		0,0,3,0,0,0,
		0,0,0,3,0,0,
		0,0,0,0,3,0,
		0,0,0,0,0,10],self.pub1)
		writeposetofile(self.filename,pose_transformed2,1)
#rgbdo
class rgbdo(object):
	#k is provided from the shell script and corespondence there with a algorihtm
	def __init__(self,tfmaptopose,tfodomtobase,tfcamtoopt):
		self.filename="/rgbdo.txt"
		removefile(path+self.filename)
		self._sub = rospy.Subscriber("/Rgbd",PoseWithCovarianceStamped,self.writemrpt)
		self.tfmaptopose=tfmaptopose
		self.tfcamtoopt=tfcamtoopt
		self.tfodomtobase=tfodomtobase
		self.pub1 = rospy.Publisher('/monovo',PoseWithCovarianceStamped,queue_size=10)
	def writemrpt(self,posecovmsg):
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(posecovmsg.pose, self.tfmaptopose)    
		pose_transformed2.header.stamp = posecovmsg.header.stamp       
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[3,0,0,0,0,0,
		0,3,0,0,0,0,
		0,0,3,0,0,0,
		0,0,0,3,0,0,
		0,0,0,0,3,0,
		0,0,0,0,0,10],self.pub1)
		writeposetofile(self.filename,pose_transformed2,1)
#rtabmap
class Rtabmap(object):
	#k is provided from the shell script and corespondence there with a algorihtm
	def __init__(self,tfmaptopose,tfodomtobase,tfcamtoopt,mal):
		self.filename="/rtabmap.txt"
		removefile(path+self.filename)
		self._sub = rospy.Subscriber("/rtabmap/odom",Odometry,self.writemrpt)
		self.tfmaptopose=tfmaptopose
		self.tfcamtoopt=tfcamtoopt
		self.tfodomtobase=tfodomtobase
		self.pub1 = rospy.Publisher('/monovo',PoseWithCovarianceStamped,queue_size=10)
		self.cor=1
		if mal == 4:
			self.cor=100
	def writemrpt(self,posecovmsg):
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(posecovmsg.pose, self.tfmaptopose)
		pose_transformed2.header.stamp = posecovmsg.header.stamp
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[1.5*self.cor,0,0,0,0,0,
		0,1.5*self.cor,0,0,0,0,
		0,0,1.5*self.cor,0,0,0,
		0,0,0,1.5*self.cor,0,0,
		0,0,0,0,1.5*self.cor,0,
		0,0,0,0,0,1.5*self.cor],self.pub1)
		writeposetofile(self.filename,pose_transformed2,1)
#for rangeflo currently not used
class Rangeflo(object):
	def __init__(self,tfmaptopose,tfodomtobase,tfbasetolidar,mal):
		self.filename="/rangflo.txt"
		removefile(path+self.filename)
		self._sub = rospy.Subscriber("/odom_rfo",Odometry,self.writemrpt)
		self.tfmaptopose=tfmaptopose
		self.tfbasetolidar=tfbasetolidar
		self.tfodomtobase=tfodomtobase
		self.pub1 = rospy.Publisher('/monovo',PoseWithCovarianceStamped,queue_size=10)
		self.cor=1
		if mal == 4:
			self.cor=100
	#callback for monovo
	def writemrpt(self,posecovmsg):
		pose_transformed2 = tf2_geometry_msgs.do_transform_pose(posecovmsg.pose, self.tfmaptopose)
		pose_transformed2.header.stamp = posecovmsg.header.stamp
		#we need the msg in the format that its in basefootprint but relative to optitrack which we got from above
		genandpubposewcovinodomfrombasefootmsg(self.tfodomtobase,pose_transformed2,1,[1.5*self.cor,0,0,0,0,0,
		0,1.5*self.cor,0,0,0,0,
		0,0,1.5*self.cor,0,0,0,
		0,0,0,1.5*self.cor,0,0,
		0,0,0,0,1.5*self.cor,0,
		0,0,0,0,0,1.5*self.cor],self.pub1)
		writeposetofile(self.filename,pose_transformed2,1)

if __name__ == "__main__":
	getransforms=np.fromstring(sys.argv[1], dtype=float, sep=',')
	print("Starting Transforms\n")
	if getransforms==1:
		print("Getting Transforms")
		gettransform()
		quit()
	elif getransforms==2:
		getLi=getLive(sys.argv[2])
		i=0
		while (not getLi.finishwriting):
			i=1
			#print("Waiting for Messages on topics gt")
		quit()
	print("init ros node")
	rospy.init_node('keypoint',anonymous=True)
	#for arg1 to arg5 and arg8 look for the meaning below mal is variable for setting a data to be less credible basically saying that the output of this algorithm should not be trusted
	#mal has to be set to the same number as k in the shell script and then makes this algorithms output be with high covarinace
	arg1=np.fromstring(sys.argv[2], dtype=float, sep=',')
	arg2=np.fromstring(sys.argv[3], dtype=float, sep=',')
	arg3=np.fromstring(sys.argv[4], dtype=float, sep=',')
	arg4=np.fromstring(sys.argv[5], dtype=float, sep=',')
	arg5=np.fromstring(sys.argv[6], dtype=float, sep=',')
	#will contain k
	arg6=np.fromstring(sys.argv[7], dtype=float, sep=',')
	mal=int(np.fromstring(sys.argv[8], dtype=float, sep=','))
	arg8=np.fromstring(sys.argv[9], dtype=float, sep=',')
	tfmaptoodom=TransformStamped()
	filltf(tfmaptoodom,arg1,"map","odom",0)
	tfmaptopose=TransformStamped()
	filltf(tfmaptopose,arg2,"map","pose",0)	
	tfodomtobase=TransformStamped()
	filltf(tfodomtobase,arg3,"odom","base_footprint",0)
	tfcamtoopt=TransformStamped()
	filltf(tfcamtoopt,arg4,"camera_link","camera_color_optical_frame",0)
	tfbasetolidar=TransformStamped()
	filltf(tfbasetolidar,arg5,"base_footprint","laser_front",0)
	tfbasetocam=TransformStamped()
	filltf(tfbasetocam,arg8,"base_footprint","camera_link",0)
	#for ekfcallback
	ekf=Ekf(tfmaptoodom)
	#for orbslam
	orbslam=Orbslam(tfmaptopose,tfodomtobase,mal,tfbasetocam)
	#for hector
	hector=Hector(tfmaptopose,tfodomtobase,tfbasetolidar,mal)
	#for gmapping and amcl and karto and mrpt
	GmapandAmclandKartoandMRPT = GmapandAmclandKartoandMRPT(arg6,tfmaptoodom,tfbasetolidar,mal)
	#for rtabmap
	Rtabmap = Rtabmap(tfmaptopose,tfodomtobase,tfcamtoopt,mal)
	#for mrpt not fast enough
	#MRPT = MRPT(sys.argv[6],tfmaptoodom,tfodomtobase) 
	#for paslam
	paslam=PASlam(tfmaptopose,tfodomtobase)
	#for ohm_tsd_slam
	ohm = Ohm(tfmaptopose,tfodomtobase,tfbasetolidar)
	#for rf2o
	rf2o = Rf2o(tfmaptopose,tfodomtobase,tfbasetolidar)
	#for laserscanmatcher
	laserscanmatcher = Laserscanmatcher(tfmaptopose,tfodomtobase,tfbasetolidar)
	#for lsdslam 
	LSDSLAM = LSDSLAM(tfmaptopose,tfodomtobase,tfcamtoopt)
	#for monovo
	Monovo = Monovo(tfmaptopose,tfodomtobase,tfcamtoopt)
	#for viso2
	Viso2 = viso2(tfmaptopose,tfodomtobase,tfcamtoopt)
	#for rgbdo
	RGBDO = rgbdo(tfmaptopose,tfodomtobase,tfcamtoopt)
	rospy.spin()
