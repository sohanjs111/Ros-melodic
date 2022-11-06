#for our dataset publish necessary tf
# or if you pass 1 as parameter and give a bag file to read data from and a bag file to write data to will write the tfs into the new bag file in tf_static
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
	static_transformStamped4 = geometry_msgs.msg.TransformStamped()
	static_transformStamped4.header.frame_id = "base_footprint"
	static_transformStamped4.child_frame_id = "camera_link" 
	#for evo including the transform from mount to camera_link which is 0.0 0.0175 0.0125
	static_transformStamped4.transform.translation.x = 0.45
	static_transformStamped4.transform.translation.y = 0
	static_transformStamped4.transform.translation.z = -0.11
	quat = tf.transformations.quaternion_from_euler(0,0,0)
	static_transformStamped4.transform.rotation.x = quat[0]
	static_transformStamped4.transform.rotation.y = quat[1]
	static_transformStamped4.transform.rotation.z = quat[2]
	static_transformStamped4.transform.rotation.w = quat[3]
	#the use of camera_link to openni_camera is a workaround so that all systems find the right transformations
	static_transformStamped2 = geometry_msgs.msg.TransformStamped()
	static_transformStamped2.header.frame_id = "camera_link" 
	static_transformStamped2.child_frame_id = "openni_camera"
	static_transformStamped2.transform.translation.x = 0.0
	static_transformStamped2.transform.translation.y = 0.0
	static_transformStamped2.transform.translation.z = 0.0
	quat = tf.transformations.quaternion_from_euler(0, 0, 0)
	static_transformStamped2.transform.rotation.x = quat[0]
	static_transformStamped2.transform.rotation.y = quat[1]
	static_transformStamped2.transform.rotation.z = quat[2]
	static_transformStamped2.transform.rotation.w = quat[3] 
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
	static_transformStamped3.child_frame_id = "camera_color_optical_frame" 
	static_transformStamped3.transform.translation.x = 0.0
	static_transformStamped3.transform.translation.y = 0.0
	static_transformStamped3.transform.translation.z = 0.0
	quat = tf.transformations.quaternion_from_euler(0, 0,0)
	static_transformStamped3.transform.rotation.x = quat[0]
	static_transformStamped3.transform.rotation.y = quat[1]
	static_transformStamped3.transform.rotation.z = quat[2]
	static_transformStamped3.transform.rotation.w = quat[3]
	static_transformStamped5 = geometry_msgs.msg.TransformStamped() 
	static_transformStamped5.header.frame_id = "camera_link" 
	static_transformStamped5.child_frame_id = "camera_imu_optical_frame"
	static_transformStamped5.transform.translation.x = 0.0
	static_transformStamped5.transform.translation.y = 0.0
	static_transformStamped5.transform.translation.z = 0.0        
	static_transformStamped5.transform.rotation.x = 0.0
	static_transformStamped5.transform.rotation.y = 0.0
	static_transformStamped5.transform.rotation.z = 0.0
	static_transformStamped5.transform.rotation.w = 1
	static_transformStamped7 = geometry_msgs.msg.TransformStamped()
	static_transformStamped7.header.frame_id = "camera_link"
	static_transformStamped7.child_frame_id = "camera_aligned_depth_to_color_frame" 
	#for evo including the transform from mount to camera_link which is 0.0 0.0175 0.0125
	static_transformStamped7.transform.translation.x = 0.0
	static_transformStamped7.transform.translation.y = 0.0
	static_transformStamped7.transform.translation.z = 0.0
	quat = tf.transformations.quaternion_from_euler(0, 0, 3.1415926535897931/2)
	static_transformStamped7.transform.rotation.x = quat[0]
	static_transformStamped7.transform.rotation.y = quat[1]
	static_transformStamped7.transform.rotation.z = quat[2]
	static_transformStamped7.transform.rotation.w = quat[3]
	static_transformStamped8 = geometry_msgs.msg.TransformStamped()
	static_transformStamped8.header.frame_id = "camera_link"
	static_transformStamped8.child_frame_id = "camera_depth_optical_frame" 
	#for evo including the transform from mount to camera_link which is 0.0 0.0175 0.0125
	static_transformStamped8.transform.translation.x = 0
	static_transformStamped8.transform.translation.y = 0
	static_transformStamped8.transform.translation.z = 0
	static_transformStamped8.transform.rotation.x = 0
	static_transformStamped8.transform.rotation.y = 0
	static_transformStamped8.transform.rotation.z = 0
	static_transformStamped8.transform.rotation.w = 1     
	static_transformStamped9 = geometry_msgs.msg.TransformStamped()
	static_transformStamped9.header.frame_id = "base_footprint"
	static_transformStamped9.child_frame_id = "scan_front" 
	static_transformStamped9.transform.translation.x = 0
	static_transformStamped9.transform.translation.y = 0
	static_transformStamped9.transform.translation.z = 0.64
	quat = tf.transformations.quaternion_from_euler(0, 0, 0)
	static_transformStamped9.transform.rotation.x = quat[0]
	static_transformStamped9.transform.rotation.y = quat[1]
	static_transformStamped9.transform.rotation.z = quat[2]
	static_transformStamped9.transform.rotation.w = quat[3]
	writetobag = sys.argv[1]
	print(writetobag)
	if int(writetobag) == 1:
		print("Hello")
		in_bag2 = sys.argv[2]
		out_bag2 = sys.argv[3]
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
						static_transformStamped1.header.stamp=t
						static_transformStamped2.header.stamp=t
						static_transformStamped3.header.stamp=t
						static_transformStamped4.header.stamp=t
						static_transformStamped5.header.stamp=t
						static_transformStamped7.header.stamp=t
						static_transformStamped8.header.stamp=t
						static_transformStamped9.header.stamp=t
						outbag.write(topic, static_transformStamped1, t)
						outbag.write(topic, static_transformStamped2, t)
						outbag.write(topic, static_transformStamped4, t)
						outbag.write(topic, static_transformStamped5, t)
						outbag.write(topic, static_transformStamped3, t)
						outbag.write(topic, static_transformStamped6, t)
						outbag.write(topic, static_transformStamped7, t)
						outbag.write(topic, static_transformStamped8, t)
						outbag.write(topic, static_transformStamped9, t)
				elif topic != 'tf_static':
					outbag.write(topic, msg, t)
	else:
		rospy.init_node('my_static_tf2_broadcaster')
		broadcaster = tf2_ros.StaticTransformBroadcaster()  
		static_transformStamped1.header.stamp = rospy.Time.now()
		static_transformStamped2.header.stamp = rospy.Time.now()
		static_transformStamped3.header.stamp = rospy.Time.now()  
		static_transformStamped4.header.stamp = rospy.Time.now()
		static_transformStamped5.header.stamp = rospy.Time.now()    
		static_transformStamped7.header.stamp = rospy.Time.now()  
		static_transformStamped8.header.stamp = rospy.Time.now() 
		static_transformStamped9.header.stamp = rospy.Time.now()       
		broadcaster.sendTransform([static_transformStamped1,static_transformStamped2,static_transformStamped3,static_transformStamped4,static_transformStamped5,
		static_transformStamped9])
		#we dont broadcast laser_front tf as its already in the tf topic        	       	        	
		#due to the fact that the coordinate system of our algorithms looks different from the realsensesystem     
		rospy.spin()
		#rosrun tf static_transform_publisher 0.0929 0.0325 0.0705 1.5707963 0 -1.5707963 base_footprint Opencv 100
