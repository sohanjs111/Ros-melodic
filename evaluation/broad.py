#for our dataset publish necessary tf
# if you pass 1 as first parameter and give a bag file to read data from and a bag file to write data to will write the tfs into the new bag file in tf_static
# argument 2 and 3 will then be the input and output bag arg
# if you pass 0 as first parameter
# argument 2 will be the tf from base_footprint to camera_link
# argument 3 will be the tf from base_footprint to scan_front
# argument 4 will be the tf from base_footprint to imu
# argument 5 will be the tf from camera_link to camera_imu_optical_frame
# argument 6 will be the tf from camera_link to camera_depth_optical_frame
# argument 7 will be the tf from camera_link to camera_aligned_depth_to_color_frame
# argument 8 will be the tf from camera_link to camera_color_frame we put it into camlink to openni_camera as we add to additional transforms for some algorithms to work
# argument 9 will be the tf from camera_aligned_depth_to_color_frame to camera_color_optical_frame we put it into camlink to openni_camera as we add to additional transforms for some algorithms to work
#!/usr/bin/env python
import rospy
# to get commandline arguments
import sys
# because of transformations
import tf
import tf2_ros
import rosbag
import geometry_msgs.msg
import time
import yaml
import subprocess
import numpy as np

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

def status(length, percent):
	sys.stdout.write('\x1B[2K') # Erase entire current line
	sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
	progress = "Progress: ["
	for i in range(0, length):
		if i < length * percent:
			progress += '='
		else:
			progress += ' '
	progress += "] " + str(round(percent * 100.0, 2)) + "%"
	sys.stdout.write(progress)
	sys.stdout.flush()

if __name__ == '__main__':
	writetobag = sys.argv[1]
	print(writetobag,sys.argv[9])
	statictfbasetocam=geometry_msgs.msg.TransformStamped()
	statictfbasetoscanfront=geometry_msgs.msg.TransformStamped()
	statictfbasetoimu=geometry_msgs.msg.TransformStamped()
	statictfcamtoimuopt=geometry_msgs.msg.TransformStamped()
	statictfcamtodepthopt=geometry_msgs.msg.TransformStamped()
	statictfcamtoaligndepthopt=geometry_msgs.msg.TransformStamped()
	statictfcamtocolorframe=geometry_msgs.msg.TransformStamped()
	statictfaligndepthopttocolorframe=geometry_msgs.msg.TransformStamped()

	if int(writetobag) == 1:
		in_bag2 = sys.argv[2]
		out_bag2 = sys.argv[3]
	if int(writetobag) == 0:
		basetocamlink =np.fromstring(sys.argv[2], dtype=float, sep=',')
		filltf(statictfbasetocam,basetocamlink,"base_footprint","camera_link",0)
		basetoscanlink =np.fromstring(sys.argv[3], dtype=float, sep=',')
		filltf(statictfbasetoscanfront,basetoscanlink,"base_footprint","scan_front",0)
		basetoimulink =np.fromstring(sys.argv[4], dtype=float, sep=',')
		filltf(statictfbasetoimu,basetoimulink,"base_footprint","imu",0)
		camtoimulink =np.fromstring(sys.argv[5], dtype=float, sep=',')
		filltf(statictfcamtoimuopt,camtoimulink,"camera_link","camera_imu_optical_frame",0)
 		camtodepthlink =np.fromstring(sys.argv[6], dtype=float, sep=',')
		filltf(statictfcamtodepthopt,camtodepthlink,"camera_link","camera_depth_optical_frame",0)
 		camtoaldepthlink =np.fromstring(sys.argv[7], dtype=float, sep=',')
		filltf(statictfcamtoaligndepthopt,camtoaldepthlink,"camera_link","camera_aligned_depth_to_color_frame",0)
 		camtocolorlink =np.fromstring(sys.argv[8], dtype=float, sep=',')
		filltf(statictfcamtocolorframe,camtocolorlink,"camera_link","openni_camera",0)
 		alignedepthtocolorlink =np.fromstring(sys.argv[9], dtype=float, sep=',')
		filltf(statictfaligndepthopttocolorframe,alignedepthtocolorlink,"camera_aligned_depth_to_color_frame","camera_color_optical_frame",0)
	print(statictfbasetocam,statictfbasetoscanfront,statictfbasetoimu,statictfcamtoimuopt,statictfcamtoaligndepthopt)
	#the use of camera_link to openni_camera is a workaround so that all systems find the right transformations
	static_transformStamped1 = geometry_msgs.msg.TransformStamped()
	static_transformStamped1.header.frame_id = "openni_camera"
	static_transformStamped1.child_frame_id = "openni_rgb_optical_frame" 
	static_transformStamped1.transform.translation.x = 0.0
	static_transformStamped1.transform.translation.y = 0.0
	static_transformStamped1.transform.translation.z = 0.0
	#quat = tf.transformations.quaternion_from_euler(0, 0, 3.1415926535897931/2)
	quat = tf.transformations.quaternion_from_euler(0, 0, 0)
	static_transformStamped1.transform.rotation.x = quat[0]
	static_transformStamped1.transform.rotation.y = quat[1]
	static_transformStamped1.transform.rotation.z = quat[2]
	static_transformStamped1.transform.rotation.w = quat[3]
	static_transformStamped3 = geometry_msgs.msg.TransformStamped()
	static_transformStamped3.header.frame_id = "openni_rgb_optical_frame"
	static_transformStamped3.child_frame_id = "camera_color_frame"
	static_transformStamped3.transform.translation.x = 0.0
	static_transformStamped3.transform.translation.y = 0.0
	static_transformStamped3.transform.translation.z = 0.0
	quat = tf.transformations.quaternion_from_euler(0, 0,0)
	static_transformStamped3.transform.rotation.x = quat[0]
	static_transformStamped3.transform.rotation.y = quat[1]
	static_transformStamped3.transform.rotation.z = quat[2]
	static_transformStamped3.transform.rotation.w = quat[3]

	if int(writetobag) == 1:
		info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', in_bag2], stdout=subprocess.PIPE).communicate()[0])
		duration = info_dict['duration']
		first=0
		start_time=0
		last_time = time.clock()
		with rosbag.Bag(out_bag2, 'w') as outbag:
			for topic, msg, t in rosbag.Bag(in_bag2).read_messages():
				if first == 0:
					first =1
					start_time=t.to_sec()
				if time.clock() - last_time > .1:
					percent = (t.to_sec() - start_time) / duration
					status(40, percent)
					last_time = time.clock()
				if topic == "tf_static" and msg.transforms:
					transforms_to_keep = []
					for i in range(len(msg.transforms)):
						statictfbasetocam.header.stamp=t
						statictfbasetoscanfront.header.stamp=t
						statictfbasetoimu.header.stamp=t
						statictfcamtoimuopt.header.stamp=t
						statictfcamtodepthopt.header.stamp=t
						statictfcamtocolorframe.header.stamp=t
						statictfcamtoaligndepthopt.header.stamp=t
						statictfaligndepthopttocolorframe.header.stamp=t
						static_transformStamped1.header.stamp=t
						static_transformStamped3.header.stamp=t
						outbag.write(topic, statictfbasetoscanfront, t)
						outbag.write(topic, statictfbasetocam, t)
						outbag.write(topic, statictfbasetoimu, t)
						outbag.write(topic, statictfcamtoimuopt, t)
						outbag.write(topic, statictfcamtoaligndepthopt, t)
						outbag.write(topic, statictfcamtodepthopt, t)
						outbag.write(topic, statictfcamtocolorframe, t)
						outbag.write(topic, statictfaligndepthopttocolorframe, t)
						outbag.write(topic, static_transformStamped1, t)
						outbag.write(topic, static_transformStamped3, t)
				elif topic != 'tf_static':
					outbag.write(topic, msg, t)
	else:
		rospy.init_node('my_static_tf2_broadcaster')
		broadcaster = tf2_ros.StaticTransformBroadcaster()
		statictfbasetoimu.header.stamp = rospy.Time.now()
		statictfbasetocam.header.stamp = rospy.Time.now()
		statictfbasetoscanfront.header.stamp = rospy.Time.now()
		statictfcamtoimuopt.header.stamp = rospy.Time.now()
		statictfcamtocolorframe.header.stamp = rospy.Time.now()
		statictfcamtodepthopt.header.stamp = rospy.Time.now()
		statictfcamtoaligndepthopt.header.stamp = rospy.Time.now()
		statictfaligndepthopttocolorframe.header.stamp= rospy.Time.now()
		static_transformStamped1.header.stamp = rospy.Time.now()
		static_transformStamped3.header.stamp = rospy.Time.now()
		broadcaster.sendTransform([static_transformStamped1,static_transformStamped3,
		statictfbasetocam,statictfbasetoscanfront,statictfbasetoimu,
		statictfcamtoimuopt,statictfcamtocolorframe,statictfcamtodepthopt,statictfcamtoaligndepthopt
		,statictfaligndepthopttocolorframe])
		#we dont broadcast laser_front tf as its already in the tf topic
		#due to the fact that the coordinate system of our algorithms looks different from the realsensesystem
		rospy.spin()
		#rosrun tf static_transform_publisher 0.0929 0.0325 0.0705 1.5707963 0 -1.5707963 base_footprint Opencv 100
