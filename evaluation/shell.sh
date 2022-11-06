#before using the script ensure that all programs used in the script are working
#automating play of bags and running of algorithms
#for usage on our dataset
#for fast evaluation of just one dataset with one algorithm
#roscore started for every run new
#where you want the data to be stored
destinationfolder=/home/catkin_ws/src/evaluation
#where keypoint.py is located and all the other python files should be located here, broad.py, and the compiled version of remap.cpp the compile guide is in the wiki
sourcefolder=/home/catkin_ws/src/evaluation
#where launch files we additionally use lay down
additionlaunchfolder=/home/catkin_ws/src/aros/launch
#path to bags
bagfolder=/home/catkin_ws/src/Rosbags
#your rosbags
AR=("$bagfolder/lidarchallenge.bag" "$bagfolder/bestcase.bag")
#length of the bagfiles change 
ARL=(40 79)
#set to 1 if you want to skip a bag in the list
skipbag=(1 0)
numbag=${#AR[@]}
#folder of your maps
mapfolder=/home/meow/share/unity
#name of your maps
map=("$mapfolder/0-hectormap-0.yaml" "$mapfolder/mymap.yaml")
#need to be specified to by the user contains the gt for this rosbag, if you have just a csv file you can generate the file by running gttofile.py 
gtfile=("$bagfolder/lidarchallengegt.txt" "$bagfolder/bestcasegt.txt")
#this file will be generated when you set getf to 1 in this script and contains transformations
tffile=("$sourcefolder/tflidarchallenge.txt" "$sourcefolder/tfbestcase.txt")
#to get the tf (transformation) data and the initialstate of odometry into a file run the script with getf=1 with your dataset once and look inside tf file given above in tffile list
#To get initalstate we take the first odometry value of the bag and take x,y,z and qx qy qz qw convert to roll pitch yaw 
#and take good care that the values below are not separated by a enter for linebreaks.
#The idenification of the initalstate of odom is necessary because for the ekf we use differential false so that the pose data is set to this value at the very start
#The reason for this ist that imu data comes to fast and would init our filter with values 0 0 0 at start and not with the correct values of the odometry. This would lead to the ekf perform bad.
#this also enables us to use just vx,vy from odometry for fusion with the x,y,qz,qw values fom vo we will not
#The functionality to set the ekf to the inital value is enable via ekf_template.yaml file which is used by ekf_template2.launch
#if you want to get the transforms maptoodom maptopose set getf to 1 then it will generate the tffile for you which will be used if you set getf to zero
#this requires to have a rosbag and the corresponding gtfile generate with the command given in the line above gtfile 
getf=1
#we can keep the camera data to zeros and avoid by this having to apply this tf on the gt data to get gt in base_footprint
#for evo including the transform from mount to camera_link which is 0.0 0.0175 0.0125
#adjust the below transformation to match your setup
camtoopt=('"1579261351.192527056, 0.0, 0.0, 0.0, 0, 0, 0, 1"')
basetoimu=('"1579261351.192527056, 0.0, 0.0, 0.0, 0, 0, 0, 1"')
basetocam=('"1579261351.192527056, 0.45, 0.0, -0.11, 0, 0, 0, 1"')
basetolidar=('"1579261351.192527056, 0.0, 0.0, 0.64, 0, 0, 0, 1"')
#amount of repetitions
maxrep=1
#delay of bag to enable subscribers to register to topics 
#delay should be at least 3 seconds otherwise for example robot_localization will not have enough time to get the starting position right
delay=3
#rate at which the bag shall be played backe
rate=(1 1 1 1 0.1 0.1 0.1 1 1 1 1 1 1 1 1 0.1)
#for rtabmap run the removeduplicate.py script to ensure no duplicates get published.
algnam=('wheelodom' 'lidarhector' 'amcl' 'lidargmap' 'rtabmap' 'ekfall' 'orb' 'imu' 'ohm' 'rf2o' 'laserscanmatcher')
# 0 1 2 3 4 5 6 7 8 9 10
#if the order above is changed changes need to be made to keypoint.py and keypoint.py as well for odom i set mal to 0 but also passed 3 to keypoint cause gmapping was running it.
mal=-1
#which algorithm to skip set entry in this list to 1
skip=(1 1 1 1 1 1 1 1 1 1 0)
if (( $getf == 1))
then
for ((idx=0; idx<$numbag; idx++))
do
if (( ${skipbag[idx]} == 0))
then
	echo "Get tf"
	xterm -hold -e "python $sourcefolder/keypoint.py 1 ${AR[idx]} ${gtfile[idx]} ${tffile[idx]}"
	sleep 1
	echo "Got tf"
fi
done
exit
fi
#internal parameters / variables no need for changing
a=""
pause=0
#reduce it with 1 because it starts k with 0 
numofalgo=$(echo  "(${#algnam[@]}-1)"|bc )
#https://unix.stackexchange.com/questions/278502/accessing-array-index-variable-from-bash-shell-script-loop
for ((idx=0; idx<$numbag; idx++))
do
	if (( ${skipbag[idx]} == 0))
	then
		#get the tfs
		k=0
		while read -r line; do
		#https://stackoverflow.com/questions/9084257/bash-array-with-spaces-in-elements
		test=($f)
		testr=''
		for m in ${line[*]}
		do
			testr=($echo"$testr $m")
		done
		if (($k == 0))
		then 
			initalstate=("${testr[*]}")
		fi
		if (($k == 1))
		then 
			maptopose=("${testr[*]}")
		fi
		if (($k == 2))
		then 
			odomtobasefoot=("${testr[*]}")
		fi
		if (($k == 3))
		then 
			maptoodom=("${testr[*]}")
		fi
		k=$(echo "($k + 1)"|bc ) 
	done < ${tffile[idx]}
	for ((k=0; k<=$numofalgo; k++))
	do
	if (( ${skip[k]} == 0))
	then
		for ((rep=0; rep<maxrep; rep++))
		do
		#delete previous settings
		xterm -e "rosclean purge -y"
		xterm -title "roscore" -e "roscore" &
		ID=$!
		sleep 3
		if (($k == 2))
		then
			#check if the maplaunch worked otherwise amcl will not get a map
			xterm -hold -e "roslaunch $additionlaunchfolder/mapserver.launch map_file:=${map[idx]}"&
			amcladdsid=$!
		fi
		sleep 4
		#everytime orb is involved
		if (($k == 6)) || (($k == 5)) || (($k == 15))
		then
			xterm -hold -e "rostopic echo /camera/depth_registered/image_rect_raw/header" &
			myBackgroundXtermPID6=$!
		fi
		xterm -hold -e "rosparam set use_sim_time false;python $sourcefolder/broad.py 0" &
		myBackgroundXtermPID1=$!     
		amcladdsid=$myBackgroundXtermPID1
		ekf1id=$myBackgroundXtermPID1
		ekf2id=$myBackgroundXtermPID1
		ekf3id=$myBackgroundXtermPID1
		ekf4id=$myBackgroundXtermPID1
		ekf5id=$myBackgroundXtermPID1
		#sleep above is necessary so that broad.py comes up
		sleep 1
		#for wheelodom
		if (($k == 0))
		then
			#automatically closes no need for kill
			xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${basetoimu} 0 ${mal}" &
			myBackgroundXtermPID6=$!  
		fi
		#for hectorslam
		if (($k == 1))
		then
			xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${basetoimu} 0 ${mal}" &
			myBackgroundXtermPID6=$!   
			a="roslaunch $additionlaunchfolder/hectormapping_default.launch"
		fi
		#for amcl
		if (($k == 2))
		then
			a="roslaunch $additionlaunchfolder/amcl.launch"
			xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${basetoimu} 1 ${mal}" &
			myBackgroundXtermPID6=$!
		fi 
		#for gmapping
		if (($k == 3))
		then
 			xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${basetoimu} 1 ${mal}" &
			myBackgroundXtermPID6=$!  
			a="roslaunch $additionlaunchfolder/gmap.launch"  
		fi
		#for rtabmap
		if (($k == 4))
		then
			xterm -hold -e "python $sourcefolder/rtabmapchange.py" &
			ekf5id=$!
			a="roslaunch rtabmap_ros rtabmap.launch"
		fi 
		#ekf
		if (($k == 5))
		then
			xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${basetoimu} 0 ${mal}" &
			ekf1id=$!
			xterm -hold -e "roslaunch realsense2_camera opensource_tracking2.launch" &    
			ekf3id=$!   
			xterm -hold -e "roslaunch rtabmap_ros rtabmap.launch" &    
			ekf4id=$!              
			xterm -e "roslaunch $additionlaunchfolder/gmap.launch"  &
			amcladdsid=$!                      
			scaleFactor=1.25
			nLevels=8
			iniThFast=20
			minThFast=6
			#this script assume that orbslam src folder is /home/irobot/catkin_ws/src/orb_slam_2_ros/ adopt it if necessary
			xterm -e  "python $sourcefolder/eyamlorb.py $scaleFactor $nLevels $iniThFast $minThFast"
			sleep 2
			a="roslaunch orb_slam2_ros orb_slam2_d435_rgbd.launch"
			#When applying changes to the yaml file keep in mind that some vo algorithms publish to the topic vo2 while others to the topic vop because they have different message types
			#so when editing stuff for the config also of the one topic apply the same changes to the conf to the topic vop	
			#initalstate is already in odom frame. all others are currently transformed into odom and then passed to ekf 	   
			xterm -hold -e "roslaunch robot_localization ekf_template2.launch inital:='$initalstate'" &
			ekfPID=$!   
		fi 
		#for orbslam
		if (($k == 6))
		then
			#seems to be essential otherwise orbslam seems to stop publishing 		    
			scaleFactor=1.25
			nLevels=8
			iniThFast=6
			minThFast=3
			#this script assume that orbslam src folder is /home/irobot/catkin_ws/src/orb_slam_2_ros/ adopt it if necessary
			xterm -e  "python $sourcefolder/eyamlorb.py $scaleFactor $nLevels $iniThFast $minThFast"
			sleep 2
			a="roslaunch orb_slam2_ros orb_slam2_d435_rgbd.launch"
			xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${basetoimu} 1 ${mal}" &
			myBackgroundXtermPID6=$!  
		fi
		#for imu
		if (($k == 7))
		then
			xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${basetoimu} 0 ${mal}" &
			myBackgroundXtermPID6=$!  
			#automatically closes no need for kill
			# by providing the necessary information for transformation of the imu data into the odometry frame the ekf will take 
			# care to the imu data being in the right coordinate system. so there is no need to transform the data on your own
			# only the transformation from camera_link into camera_imu_ and from basefootprint to camera_link are necessary
			# we provide them within broad.py
			# the reason for this is that imu_filter_magic publish its data relative to the parent frame which is by default odom
			a="roslaunch realsense2_camera opensource_tracking2.launch" 
		fi
 		#for ohm_tsd_slam #needs tf odom to base 
		if (($k == 8))
		then
			a="roslaunch ohm_tsd_slam slam2.launch"
		fi
		#for rf2o
		if (($k == 9))
		then
			a="roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch"
		fi
		#for laserscanmatcher
		if (($k == 10))
		then
			a="roslaunch laser_scan_matcher laser_scan_ma.launch"
		fi
		echo $k
		#if you have issues with keypoint.py check first that the tf file is there otherwise the parameters below will not be provided correctly
		xterm -hold -e "python $sourcefolder/keypoint.py 0 ${maptoodom[0]} ${maptopose[0]} ${odomtobasefoot[0]} ${camtoopt} ${basetolidar} $k $mal ${basetocam}" &
		myBackgroundXtermPID0=$!
		# execute the launch command for the specific algorithm stored inside variable a
		xterm -hold -e $a &
		myBackgroundXtermPID4=$!
		#republish ros image topics to new topics used by algorithms
		xterm -hold -e "$sourcefolder/./remap" &
		remap=$!
		#play the rosbag
		xterm -hold -e "rosparam set use_sim_time true;rosbag play -k --clock -d ${delay} -r ${rate[k]} ${AR[idx]} /odom:=/odomc" &
		rosbagPID5=$! 	
		#https://unix.stackexchange.com/questions/89712/how-to-convert-floating-point-number-to-integer  
		#the bonus of one seconds ensures that rounding problems from calculation of the time of the bagplay will not have an effect
		sleptim=$(echo "(${ARL[idx]}+1)/${rate[k]}"|bc ) 
		sleptim=$(echo "${sleptim%.*}+${delay}"|bc)  
		for ((time=0; time<=$((sleptim)); time=time+1))
		do
			echo "Evaluating ${algnam[k]} current time elapsed ${time} to go ${sleptim}  on bag number ${idx} Totaltime of bag is extend with 1 seconds devided by playrate to take care of rounding effects"
			echo "If you want to pause press p"
			#https://stackoverflow.com/questions/5542016/bash-user-input-if #https://askubuntu.com/questions/446156/pause-execution-and-wait-for-user-input
			#https://stackoverflow.com/questions/9483633/how-to-use-bash-read-with-a-timeout
			mainmenuinput='k'
			read -n 1 -t 1 mainmenuinput
			if [[ $mainmenuinput == 'p' ]];
			then
				pause=1
				echo $pause
			fi
			if (($k==8)) && (($(echo "(1+$time)"| bc -l) == $((sleptim))))
			then
				xterm -e "rosrun map_server map_saver -f ohmtsd"
			fi
		done
		if ((k==0))
		then
			#store gt and odom
			mv $sourcefolder/wheelodom.txt "$destinationfolder/$idx-odom-$rep.txt"
		fi
		if ((k==1)) #hector
		then
			xterm -e "rosrun map_server map_saver -f hectormap"
			mv $sourcefolder/hectormap.pgm "$destinationfolder/$idx-hectormap-$rep.pgm"
			mv $sourcefolder/hectormap.yaml "$destinationfolder/$idx-hectormap-$rep.yaml"
			sed -i "s/hectormap.pgm/$idx-hectormap-$rep.pgm/g" "$idx-hectormap-$rep.yaml"
			mv $sourcefolder/hector.txt "$destinationfolder/$idx-hector-$rep.txt"
		fi
		if ((k==2)) #amcl
		then
			kill $amcladdsid
			kill $ekf1id
			mv $sourcefolder/amcl.txt "$destinationfolder/$idx-amcl-$rep.txt"
		fi
		if ((k==3)) #gmap
		then
			xterm -e "rosrun map_server map_saver -f gmap"
			mv $sourcefolder/gmap.txt "$destinationfolder/$idx-gmap-$rep.txt"
			mv $sourcefolder/gmap.pgm "$destinationfolder/$idx-gmap-$rep.pgm"
			mv $sourcefolder/gmap.yaml "$destinationfolder/$idx-gmap-$rep.yaml"
			sed -i "s/gmap.pgm/$idx-gmap-$rep.pgm/g" "$idx-gmap-$rep.yaml"
		fi
		if ((k==4)) #rtabmap
		then
			mv $sourcefolder/rtabmap.txt "$destinationfolder/$idx-rtabmap-$rep.txt"
			kill $ekf5id
		fi
		if ((k==5)) #ekf all
		then
			mv $sourcefolder/ekf_pose.txt "$destinationfolder/$idx-ekf_pose-$rep.txt"
			kill $ekf1id
			kill $ekf2id
			kill $ekf3id
			kill $ekf4id
			kill $ekf5id
			kill $amcladdsid
		fi
		if ((k==6)) #orb
		then
			mv $sourcefolder/visual_odom_orb.txt "$destinationfolder/$idx-orbslam2-$rep.txt"
		fi
		if ((k==7))
		then
			mv $sourcefolder/imupose.txt "$destinationfolder/$idx-imupose-$rep.txt"
		fi
 		if ((k==8)) #ohm
		then
			#mapserver will be started above because the bag needs to be running / data needs to come in for ohm to publish the map
			mv $sourcefolder/ohmtsd.pgm "$destinationfolder/$idx-ohmtsd-$rep.pgm"
			mv $sourcefolder/ohmtsd.yaml "$destinationfolder/$idx-ohmtsd-$rep.yaml"
			mv $sourcefolder/ohmtsd.txt "$destinationfolder/$idx-ohmtsd-$rep.txt"
		fi
		if ((k==9)) #rf2o
		then
			mv $sourcefolder/rf2o.txt "$destinationfolder/$idx-rf2o-$rep.txt"
		fi
		if ((k==10)) #laserscan
		then
			mv $sourcefolder/laserscan.txt "$destinationfolder/$idx-laserscan-$rep.txt"
		fi

		kill $ID
		kill $myBackgroundXtermPID0
		kill $myBackgroundXtermPID1
		kill $remap
		kill $ekfPID
		kill $myBackgroundXtermPID4
		kill $rosbagPID5 
		kill $myBackgroundXtermPID6 
		if (($pause == 1))
		then
			kill $ID
			mainmenuinput='k'                    
			while [[ $mainmenuinput != 'u' ]]
			do
				echo "want to continue? press u"
				read -n 1 -t 20 mainmenuinput  
			done
			pause=0
		fi
	done 
	fi
	done
fi
done
kill $ID
