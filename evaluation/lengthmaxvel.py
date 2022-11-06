# first argument is bag file from which you want to get the max odometry value
# second argument the txt file of the ground truth
import rospy
import numpy as np
import os
import math
import csv
import rosbag
import sys
   
def maxodom(in_bag):
    	maxvelx=0
	maxvely=0
	maxtot=0
	for topic, odom, t in rosbag.Bag(in_bag).read_messages():
	   if topic == "odom" or topic == "/odom":
		if abs(odom.twist.twist.linear.x) > maxvelx :
			maxvelx = abs(odom.twist.twist.linear.x)
		if abs(odom.twist.twist.linear.y) > maxvely :
			maxvely = abs(odom.twist.twist.linear.y)
		curvel= math.sqrt(math.pow(odom.twist.twist.linear.y,2)+math.pow(odom.twist.twist.linear.x,2))
		if curvel > maxtot :
			maxtot = curvel
	print("maxxvel: ",maxvelx," maxyvel: ",maxvely)
	print("maxtotalvel: \\SI{"+str(format(maxtot,'.2f'))+"}{\\meter\\per\\second}")

def getpathlength(filename):
	path_length =0
	with open(filename) as infile:          
		gt_data = np.fromstring(infile.read(), sep="   ").reshape(-1, 8) 		
		prev_x=0
		prev_y=0
		for i in range(0, np.size(gt_data, 0) - 1):
		    cur_x=gt_data[i, 1]
		    cur_y=gt_data[i,2] 
		    if i != 0:                                           
		        path_length = path_length +math.sqrt(math.pow(cur_x-prev_x,2)+math.pow(cur_y-prev_y,2))
		    prev_x=cur_x
		    prev_y=cur_y
	print ("The path length was: \\SI{"+str(format(path_length,'.2f'))+"}{\\meter}")


if __name__ == '__main__': 
	maxodom(sys.argv[1])
	getpathlength(sys.argv[2]) 


