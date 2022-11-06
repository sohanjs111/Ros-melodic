#! /usr/bin/env python
#this file writes the data from the wheel odometry and imu topics into txt files after they are transformed in the right coordinate system 
#furthermoe it transforms the data of the 
#additionally it takes care of the imu data to get more suitable covariances
# wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${camtoopt} ${basetocam} publishtf
# the first four are the transforms the last parameter defines if a tf from odom to base footprint should be published.
import tf2_ros
import tf2_geometry_msgs #import the packages first
import tf2_msgs.msg
import rospy
import tf
from tf import transformations as t
import numpy as np
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped,TransformStamped
import geometry_msgs.msg
import sys
import math
import re 
import copy

full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)

def checkfornan(value):
       if math.isnan(value):            
              transvalue=0
       else:
              transvalue=value
       return transvalue
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
def matfromquatfromtf(tf):
	x=tf.transform.rotation.x
	y=tf.transform.rotation.y
	z=tf.transform.rotation.z
	w=tf.transform.rotation.w
	tx=tf.transform.translation.x
	ty=tf.transform.translation.y
	tz=tf.transform.translation.z
	return matfromquat(x,y,z,w,tx,ty,tz)
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
#latest version
def getinvers2(trans4):	
	trans4r=TransformStamped()
	M=matfromquatfromtf(trans4)
	Minv=np.linalg.inv(M)
	trans4r=tffromMat(Minv,trans4.header.stamp,trans4.child_frame_id,trans4.header.frame_id)
	return trans4r

def buildpose(x,y,yaw):
		posCovMsg = PoseWithCovarianceStamped()	
		posCovMsg.pose.pose.position.x =x
		posCovMsg.pose.pose.position.y =y
		posCovMsg.pose.pose.position.z =0
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		posCovMsg.pose.pose.orientation.x=quaternion[0]
		posCovMsg.pose.pose.orientation.y=quaternion[1]
		posCovMsg.pose.pose.orientation.z=quaternion[2]
		posCovMsg.pose.pose.orientation.w=quaternion[3]	
		return posCovMsg
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
def broadcastodomastf(broad,odomsg):
		static_transformStamped5 = generatetffromodommgs(odomsg)
		#static_transformStamped5 = getinvers2(static_transformStamped5)
		static_transformStamped5.header.stamp = odomsg.header.stamp
		static_transformStamped5.header.frame_id = "odom"
		static_transformStamped5.child_frame_id = "base_footprint"
		broad.sendTransform([static_transformStamped5])  
  
def writeposetofile(filename,posetowrite,fromcam,prit):
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
    writetofile(filename,posetowrite.header.stamp,posetowrite.pose.position.x,posetowrite.pose.position.y,posetowrite.pose.position.z,qx,qy,qz,qw,prit)

def writetofile(filename,stamp,x,y,z,qx,qy,qz,qw,prit):
	    with open (path+"/"+filename,"a") as odom_pose_file:
		odom_pose_file.write(str(stamp.secs)+"."+str(stamp.nsecs).zfill(9)+ "   "+str(format(x,'.11f'))+ "   " +str(format(y,'.11f'))+"   "+str(format(z,'.11f'))+"   "+str(format(qx,'.6f'))+"   "+str(format(qy,'.6f'))+"   "+str(format(qz,'.6f'))+"   "+str(format(qw,'.6f'))+"\n")
		#odom_pose_file.write(str(stamp.secs)+"."+str(stamp.nsecs).zfill(9)+ "   "+str(format(x,'.11f'))+ "   " +str(format(y,'.11f'))+"   "+str(format(z,'.11f'))+"   "+str(format(qx,'.11f'))+"   "+str(format(qy,'.11f'))+"   "+str(format(qz,'.11f'))+"   "+str(format(qw,'.11f'))+"\n")  
	    if prit == True:
	    	print("Position for "+filename+": "+str(x)+" "+str(y)+" "+str(z)+" "+str(qx)+" "+str(qy)+" "+str(qz)+" "+str(qw))

class OdomTopic(object):
	def __init__(self,trans,odomtobase,broadtf,mal,topic_name = '/odomc'):
		self.trans = copy.deepcopy(trans)
		self.odomtobase = copy.deepcopy(odomtobase)  
		self._topic_name=topic_name
		self._sub = rospy.Subscriber(self._topic_name,Odometry,self.topic_callback)
		self._pub = rospy.Publisher('/odom',Odometry,queue_size=10)
		# http://lars.mec.ua.pt  eye  scripts  tf_localization			
		# startpostion in odom frame is given as x,y,z,roll,pitch,yaw we transform it now into base_footprint 	
		self.x = 0
		self.y = 0
		self.angz = 0
		self.vx=0
		self.vy=0
		self.vangz=0
                self.stamp=0
		self.first = 0
		self.broadtf = broadtf
		self.broadcaster = tf2_ros.TransformBroadcaster()	
		self.cor=1
		self.duration=0
		if mal== 0:
			self.cor=100       
	def topic_callback(self, msg):	
		#odometry twist message is given in base_footprint and we calculate x and y from them so we have to apply two transform to get them into the map frame		
		#vyl=self.vy			
		vang=msg.twist.twist.angular.z	
	
		if self.first == 0:
			self.first = 1	
			self.x = msg.pose.pose.position.x	
			self.y = msg.pose.pose.position.y
			self.stamp = msg.header.stamp
		else:
			self.duration=round((msg.header.stamp.to_sec()-self.stamp.to_sec()),3)
			vxl=(msg.twist.twist.linear.x)
			vyl=(msg.twist.twist.linear.y)
			#average over current + new angle which is calculated by current angle + velocity*duration		
			#avang=self.angz+duration*(self.vangz+vasinu)/4
			#http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom do not use an average of the angle but rather just the angle of the previous
			self.x=round(self.x + self.duration * (math.cos(self.angz) * vxl - vyl * math.sin(self.angz)),5)
			self.y=round(self.y + self.duration * (math.sin(self.angz) * vxl + vyl * math.cos(self.angz)),5)
			if vxl != 0.0:
				self.first=self.first+1


		angles = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])	 
		print(str(self.angz+self.vangz*self.duration)+" "+str(angles[2])+" "+str(self.angz)+" "+str(self.vangz)+" "+str(self.angz+self.vangz*self.duration-angles[2]))
		self.angz = angles[2]
		self.stamp = msg.header.stamp		
		self.vx=msg.twist.twist.linear.x
		self.vy=msg.twist.twist.linear.y
		self.vangz=vang	
		#data is in pose system
		posCovMsg = buildpose(self.x,self.y,self.angz)
		odomsg=Odometry()	
		odomsg.header.frame_id="odom"
		odomsg.header.stamp=msg.header.stamp
		odomsg.child_frame_id="base_footprint"
		odomsg.pose.pose=posCovMsg.pose.pose
		odomsg.pose.covariance=[0.5*self.cor,0,0,0,0,0,
		0,0.5*self.cor,0,0,0,0,
		0,0,0.5*self.cor,0,0,0,
		0,0,0,0.5*self.cor,0,0,
		0,0,0,0,0.5*self.cor,0,
		0,0,0,0,0,0.5*self.cor]
		odomsg.twist=msg.twist
		self._pub.publish(odomsg)
		posworot = tf2_geometry_msgs.do_transform_pose(posCovMsg.pose,self.odomtobase)
		odomsg.pose=posworot
		if self.broadtf == 1:
			broadcastodomastf(self.broadcaster,odomsg)
		#transform rotation 
		#transform it map to odom
		posworot = tf2_geometry_msgs.do_transform_pose(posworot,self.trans)
		posworot.header.stamp=msg.header.stamp
		#print(str(msg.twist.twist.linear.x)+" "+str(msg.twist.twist.linear.y))
		writeposetofile("/wheelodom.txt",posworot,0,False)
		#only for testing not required 
		posworot.pose.position.x=msg.twist.twist.linear.x
		posworot.pose.position.y=msg.twist.twist.linear.y
		posworot.pose.position.z=angles[2]
		posworot.pose.orientation.x=self.x
		posworot.pose.orientation.y=self.y
		writeposetofile("/odomvel.txt",posworot,0,False)
		#print("estfromodom: ",posCovMsg.pose.position.x,posCovMsg.pose.position.y)
	
def settransoftftozero(tf):
       		tf.transform.translation.x=0     
       		tf.transform.translation.y=0   
       		tf.transform.translation.z=0    
		
class ImuTopic(object):		
	def __init__(self,tfmtb,tfotb,tfbti,topic_name = '/imu/data'):
		#Data is given in camera
		self.tfmtb = copy.deepcopy(tfmtb)
		self.tfotb = copy.deepcopy(tfotb)  
		self.tfotbwouttf = TransformStamped()
		self.tfotbwouttf.transform = copy.deepcopy(tfotb.transform)	 
		settransoftftozero(self.tfotbwouttf)       		               
       		self.tfbti =  copy.deepcopy(tfbti)   
		self.tfbtiwouttf = TransformStamped() 
		self.tfbtiwouttf = copy.deepcopy(tfbti) 
		settransoftftozero(self.tfbtiwouttf)  
		self.tfbtiwoutrot = copy.deepcopy(tfbti)      
		self.tfbtiwoutrot.transform.rotation.x=0	
		self.tfbtiwoutrot.transform.rotation.y=0		               
		self.tfbtiwoutrot.transform.rotation.z=0		               
		self.tfbtiwoutrot.transform.rotation.w=1		               	               
		#we want to transform acceleartion we cannot use transformation part of this tf
       		settransoftftozero(self.tfbti)   
		self._topic_name=topic_name
		self._sub = rospy.Subscriber(self._topic_name,Imu,self.imu_callback)
		self.x = 0
		self.y = 0
		self.angz = 0
                self.stamp=0
                self.vx=0
                self.vy=0
                self.ax=0
                self.ay=0
                self.yaw=0
		self.first = 0
                self.avgax=0
                self.avgay=0
                self.meanax=0.0
                self.meanay=0.0
                self.messurements=0
		self.pub = rospy.Publisher('/poseimu',PoseWithCovarianceStamped,queue_size=10)  
		self.pub3 = rospy.Publisher('/imuinodom',Imu,queue_size=10)    
	def imu_callback(self, msg):
		#imu data is given in camera_imu_optical_frame we have to apply two transform to get them into the base_footprint frame	
	        angles = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])	        
		posCovforangularvelo=buildpose(msg.angular_velocity.x,msg.angular_velocity.y,angles[2])
		posCovMsg = PoseWithCovarianceStamped()	
		posCovMsg.pose.pose.position.x =msg.linear_acceleration.x
		posCovMsg.pose.pose.position.y =msg.linear_acceleration.y
		posCovMsg.pose.pose.position.z =0		
		posCovMsg.pose.pose.orientation=posCovforangularvelo.pose.pose.orientation			
		#angular velocity is not required cause we get a new angular with every measurement	
		#transform first into base 
		posCovMsg = tf2_geometry_msgs.do_transform_pose(posCovMsg.pose,self.tfbtiwouttf)
		posCovforangularvelo = tf2_geometry_msgs.do_transform_pose(posCovforangularvelo.pose,self.tfbtiwouttf)		
		angles = tf.transformations.euler_from_quaternion([posCovMsg.pose.orientation.x, posCovMsg.pose.orientation.y, posCovMsg.pose.orientation.z, posCovMsg.pose.orientation.w])
		if self.first == 0:
			self.first = 1	
		else:	
			duration=(msg.header.stamp.to_sec()-self.stamp.to_sec())
			avax=(self.ax+posCovMsg.pose.position.x)/2-self.meanax
			avay=(self.ay+posCovMsg.pose.position.y)/2-self.meanay
			avvx=(self.vx+avax*duration)/2
			avvy=(self.vy+avay*duration)/2	
			avyaw=(self.yaw+angles[2])/2
			self.x=self.x+duration*(math.cos(avyaw)*avvx - avvy * math.sin(avyaw))+duration*duration*(math.cos(avyaw)*avax - avay * math.sin(avyaw))/2
			self.y=self.y+duration*(math.sin(avyaw)*avvx + avvy * math.cos(avyaw))+duration*duration*(math.sin(avyaw)*avax + avay * math.cos(avyaw))/2
			self.vx=self.vx+(avax)*duration
			self.vy=self.vy+(avay)*duration
		self.stamp = msg.header.stamp				
		self.yaw=angles[2]		
		self.ax=posCovMsg.pose.position.x
		self.ay=posCovMsg.pose.position.y
		#self.avgax=self.avgax+posCovMsg.pose.position.x
		#self.avgay=self.avgay+posCovMsg.pose.position.y
		#self.messurements=self.messurements+1
		posCovMsg = buildpose(self.x,self.y,self.yaw)
		print(posCovMsg.pose.pose.position.x)
		posforimu = tf2_geometry_msgs.do_transform_pose(posCovMsg.pose,self.tfbtiwoutrot)
		posforimu = tf2_geometry_msgs.do_transform_pose(posforimu,self.tfotbwouttf)
		#transform into odom
		posToPubwcov = PoseWithCovarianceStamped()	
		posToPubwcov.pose.pose=posforimu.pose
		posToPubwcov.header.frame_id="odom"
		posToPubwcov.pose.covariance=[100000,0,0,0,0,0,
		0,100000,0,0,0,0,
		0,0,100000,0,0,0,
		0,0,0,100000,0,0,
		0,0,0,0,100000,0,
		0,0,0,0,0,100000]
		self.pub.publish(posToPubwcov)
		imumsg=Imu()
		imumsg.header.frame_id="odom"	
		imumsg.header.stamp=msg.header.stamp	
		imumsg.orientation=posforimu.pose.orientation
		posCovforangularvelo = tf2_geometry_msgs.do_transform_pose(posCovforangularvelo,self.tfotbwouttf)
		imumsg.angular_velocity.x=posCovforangularvelo.pose.position.x
		imumsg.angular_velocity.y=posCovforangularvelo.pose.position.y
		imumsg.angular_velocity.z=posCovforangularvelo.pose.position.z
		imumsg.linear_acceleration.x=posforimu.pose.position.x
		imumsg.linear_acceleration.y=posforimu.pose.position.y
		imumsg.linear_acceleration.z=posforimu.pose.position.z
		imumsg.orientation_covariance=[100000, 0, 0, 0, 100000,0, 0,0,100000]
		imumsg.angular_velocity_covariance=[100000, 0, 0, 0, 100000,0, 0,0,100000]
		imumsg.linear_acceleration_covariance=[100000, 0, 0, 0, 100000,0, 0,0,100000]
		self.pub3.publish(imumsg)
		posCovMsg = tf2_geometry_msgs.do_transform_pose(posCovMsg.pose,self.tfbtiwoutrot)
		posCovMsg = tf2_geometry_msgs.do_transform_pose(posCovMsg,self.tfmtb)
		#print("estfromimu: ",posCovMsg.pose.position.x,posCovMsg.pose.position.y,self.vx,self.vy,self.ax,self.ay)
		#for display of average
		#print("estfromimu: ",posCovMsg.pose.position.x,posCovMsg.pose.position.y,self.vx,self.vy,self.ax,self.ay,self.avgax/self.messurements,self.avgay/self.messurements)
		posCovMsg.header.stamp = msg.header.stamp
		writeposetofile("/imupose.txt",posCovMsg,0)
		
	
def filltf(trans,arg,frameid,childid):
       trans.header.stamp=arg[0]
       trans.header.frame_id=frameid
       trans.child_frame_id=childid
       trans.transform.translation.x=arg[1]
       trans.transform.translation.y=arg[2]
       trans.transform.translation.z=arg[3]
       trans.transform.rotation.x=arg[4]
       trans.transform.rotation.y=arg[5]
       trans.transform.rotation.z=arg[6]
       trans.transform.rotation.w=arg[7]

def removefile(name):
       try:
		os.remove(name)
		print(name," file Removed!")
       except:
		print (name," file not there")


if __name__ == "__main__":	
	rospy.init_node('wheelimu',anonymous=True)
	arg1=np.fromstring(sys.argv[1], dtype=float, sep=',')
	arg2=np.fromstring(sys.argv[2], dtype=float, sep=',')
	arg3=np.fromstring(sys.argv[3], dtype=float, sep=',')

	mal=int(np.fromstring(sys.argv[5], dtype=float, sep=','))
	maptobase=TransformStamped()
	filltf(maptobase,arg1,"map","base_footpirnt")
	odomtobase=TransformStamped()
	filltf(odomtobase,arg2,"odom","base_footprint")	
	odomob = OdomTopic(maptobase,odomtobase,int(sys.argv[4]),mal)  
	basetoimu=TransformStamped()
	filltf(basetoimu,arg3,"base_footprint","imu")	
	imuob = ImuTopic(maptobase,odomtobase,basetoimu)       
	removefile(path+"/wheelodom.txt")
	removefile(path+"/odomvel.txt")
	removefile(path+"/imupose.txt")
	r = rospy.Rate(10) # 10hz
	rospy.spin()	

	

	   
