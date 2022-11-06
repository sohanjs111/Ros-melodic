//in the case of this bag scalefactor seems not to be 1000 but just 1.0
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ctime>
using namespace sensor_msgs;
using namespace message_filters;
class SubscribeAndPublish
{
public://
    void callback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::ImageConstPtr& msg2,const sensor_msgs::CameraInfoConstPtr& info)
    {
	std::cout<<"Hello Sync"<<std::endl;
       //If encoding is the empty string (the default), the returned CvImage has the same encoding as source. so
      cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg2,"");
      cv::Mat depth = ptr->image;
      int scalefactor=1.0000;
      depth.convertTo(depth,CV_32FC1);
      depth.convertTo(depth, CV_32FC1,(1.0/scalefactor));
      depth.setTo(std::numeric_limits<float>::quiet_NaN(), depth == 0);
      ptr->image=depth;
      //https://github.com/auviitkgp/datasheets/wiki/Image-Transport-with-ROS-Messages-and-OpenCV
      //https://answers.ros.org/question/28144/how-to-change-imageconstptr-data/
      sensor_msgs::ImagePtr depthmsg = ptr->toImageMsg();
      sensor_msgs::Image rgb_msg=*msg;
      rgb_msg.header.frame_id="camera_color_optical_frame";

      colpub.publish(rgb_msg);
      (*depthmsg).header.stamp=(*msg).header.stamp;
      (*depthmsg).encoding="32FC1";
      (*depthmsg).header.frame_id="camera_color_optical_frame";
      depthpub.publish(depthmsg);
      sensor_msgs::CameraInfo current_rgb_msg=*info;
      current_rgb_msg.header.stamp=msg->header.stamp;
      colinpub.publish(current_rgb_msg);
      depthinpub.publish(current_rgb_msg);
      if(dbmod == true){ 
        debugfile.open (debugfilename, std::ios::out | std::ios::app);
        debugfile<<msg->header.stamp.sec<<"."<< std::setfill('0') << std::setw(9)<<msg->header.stamp.nsec<<std::endl;    
        debugfile.close();
      }
    }   
        //this inits the sync variable
        SubscribeAndPublish(int i):sync(MySyncPolicy(1000),image_sub,image_sub2,cam_info){
            std::cout<<"Creating"<<std::endl;          
            std::cout<<"Debugmode: "<<dbmod<<std::endl;
            image_sub.subscribe(nh, "/camera1uncompressed", 10);
            image_sub2.subscribe(nh, "/depth2uncompressed32", 10);
            cam_info.subscribe(nh, "/caminfo", 10);
            sync.registerCallback(boost::bind(&SubscribeAndPublish::callback,this, _1, _2,_3));
            colpub = nh.advertise<Image>("/camera/rgb/image_rect", 10);
            depthpub = nh.advertise<Image>("/camera/depth_registered/image_rect_raw", 10);
            colinpub = nh.advertise<CameraInfo>("/camera/rgb/camera_info", 10);
            depthinpub = nh.advertise<CameraInfo>("/camera/depth_registered/camera_info", 10);
            // https://www.tutorialspoint.com/cplusplus/cpp_date_time.htm
            std::time_t now = time(0);
            char* dt = ctime(&now);
            debugfilename="/home/meow/Downloads/pic"+std::string(dt)+".txt";
            //https://stackoverflow.com/questions/7352099/stdstring-to-char
            if(std::remove(debugfilename.c_str()) != 0 )
              std::cout<< "Error deleting file"<<std::endl;
            else
              std::cout<< "File successfully deleted" <<std::endl;
            std::cout<<"Done"<<std::endl;
        }

private:
        ros::NodeHandle nh;
        ros::Publisher colpub;
        ros::Publisher depthpub;
        ros::Publisher colinpub;
        ros::Publisher depthinpub;
        message_filters::Subscriber<Image> image_sub;
        message_filters::Subscriber<Image> image_sub2;
        message_filters::Subscriber<CameraInfo> cam_info;
        typedef sync_policies::ApproximateTime<Image,Image,CameraInfo> MySyncPolicy;
        Synchronizer<MySyncPolicy> sync;
        std::string debugfilename;
        std::ofstream debugfile;
        bool dbmod=true;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv,  "remapper");
    std::cout<<"Finished init"<<std::endl;
    SubscribeAndPublish test(0);
    std::cout<<"Finished creation"<<std::endl;
    ros::spin();
}
