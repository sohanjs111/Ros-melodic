#resets rtabmap now new data gets published from the algorithm
import os
from time import sleep
import subprocess
import sys
import tf
full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
def removefile(name):
       try:
		os.remove(name)
		print(name+" removed!")
       except:
		print (name+" was not there")
removefile(path+"/rtabmap.txt")


while not os.path.exists(path+"/rtabmap.txt"):
    sleep(1)

while(1):
	time = os.path.getmtime(path+"/rtabmap.txt")
	sleep(2)
	time2=os.path.getmtime(path+"/rtabmap.txt")
	if time == time2:
		print("Unchanged")
		with open('rgbdcamerachallengegt.txt') as f:
			for line in f:
				pass
			last_line = line
		print(last_line)
		data = last_line.split("   ")
		print(data)
		x=data[1]
		y=data[2]
		z=data[3]
		qx=data[4]
		qy=data[5]
		qz=data[6]
		qw=data[7]
		euler= tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
		print(x)
		arg="rosservice call  /rtabmap/reset_odom_to_pose "+"'"+str(x)+"' '"+str(y)+"' '"+str(z)+"' '"+str(euler[0])+"' '"+str(euler[1])+"' '"+str(euler[2])+"'"
		subprocess.Popen(arg,shell=True,  executable='/bin/bash')
	else:
		print("Changed")

