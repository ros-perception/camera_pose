

// yliu 06/26/2011

#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <string>
#include <time.h>
#include <sstream>
#include <iomanip>

tf::Transform transform;
std::string parent_frame_id;
std::string child_frame_id; 
std::string pdata_file;
std::string qdata_file;

void MyCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{	
	printf("From: %s\n", msg->header.frame_id.c_str());
	printf("To  : %s\n", msg->child_frame_id.c_str());
	printf("Q = [%f, %f , %f, %f]\n",msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
	printf("p = [%f, %f, %f]\n\n", msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);

  	transform.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
  	transform.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );
  	parent_frame_id = msg->header.frame_id;
	child_frame_id = msg->child_frame_id;
	

	time_t rawtime;
  	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	 

	std::ofstream qwriter(qdata_file.c_str(), std::ios::app); 
	std::ofstream pwriter(pdata_file.c_str(), std::ios::app); 
	
	if (qwriter.is_open())
  	{	
		qwriter<<std::setw(2)<<timeinfo->tm_hour<<":"<<std::setw(2)<<timeinfo->tm_min<<":"<<std::setw(2)<<timeinfo->tm_sec;
		
		qwriter<<std::setw(15)<< msg->transform.rotation.x
		       <<std::setw(15)<< msg->transform.rotation.y 
		       <<std::setw(15)<< msg->transform.rotation.z
		       <<std::setw(15)<< msg->transform.rotation.w<<std::endl;
  	}
	else
  	{
    		printf("Error opening file.\n\n");
 	}
	
	if (pwriter.is_open())
  	{	
		pwriter<<std::setw(2)<<timeinfo->tm_hour<<":"<<std::setw(2)<<timeinfo->tm_min<<":"<<std::setw(2)<<timeinfo->tm_sec;
		
		pwriter<<std::setw(15)<<msg->transform.translation.x
		       <<std::setw(15)<<msg->transform.translation.y
		       <<std::setw(15)<<msg->transform.translation.z<<std::endl;
  	}
	else
  	{
    		printf("Error opening file. \n\n");
 	}

	qwriter.close();
	pwriter.close();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_pose_static_transform_tf_broadcaster");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("camera_pose_static_transform_update", 10,  MyCallback);
	
	
	std::stringstream ss1;
	std::stringstream ss2;
	
	time_t raw_time;
  	struct tm * time_info;
	time(&raw_time);
	time_info = localtime(&raw_time);

	ss1<<"pdata_"<< time_info->tm_year+1900
	    << time_info->tm_mon+1
	    << time_info->tm_mday <<"_"
	    << time_info->tm_hour
	    << time_info->tm_min <<time_info->tm_sec;
	pdata_file = ss1.str();

	ss2<<"qdata_"<< time_info->tm_year+1900
	    << time_info->tm_mon+1
	    << time_info->tm_mday <<"_"
	    << time_info->tm_hour
	    << time_info->tm_min <<time_info->tm_sec;
	qdata_file = ss2.str();                      
	
	std::ofstream q_writer(qdata_file.c_str(), std::ios::trunc); 
	std::ofstream p_writer(pdata_file.c_str(), std::ios::trunc); 
	if (q_writer.is_open())  
	{ q_writer<<"";}
	else
	{ printf("Error opening file.\n\n");}

	if (p_writer.is_open())  
	{ p_writer<<"";}
	else
	{ printf("Error opening file.\n\n");}

	q_writer.close();
	p_writer.close();

 	tf::TransformBroadcaster br;

	ros::Duration sleeper(100/1000.0); // 10 hz

	parent_frame_id = "a";
	child_frame_id = "a";

	while(n.ok() )  //to see whether it's yet time to exit.
	{
		ros::spinOnce();

    		if (parent_frame_id.compare(child_frame_id) != 0)
      		{	
			//printf("%d\n",parent_frame_id.compare(child_frame_id));
			//ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", parent_frame_id.c_str(), child_frame_id.c_str()  );
			//printf("%s\n", parent_frame_id.c_str());
			//printf("%s\n", child_frame_id.c_str());			
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, child_frame_id));	
			//printf("..\n");
  		}
		//printf("%s - %s \n", parent_frame_id.c_str(), child_frame_id.c_str());
	 	sleeper.sleep();
	}

  	return 0;

}
