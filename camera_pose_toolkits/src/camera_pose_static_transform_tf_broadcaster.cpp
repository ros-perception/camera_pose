

// yliu 06/26/2011

#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <string>
#include <time.h>
#include <sstream>
#include <iomanip>
#include <kdl/frames.hpp>

#include <rosbag/bag.h>





static std::list<geometry_msgs::TransformStamped> T_list;


void MyCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{	
	//printf("From: %s\n", msg->header.frame_id.c_str());
	//printf("To  : %s\n", msg->child_frame_id.c_str());
	//printf("Q = [%f, %f , %f, %f]\n",msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
	//printf("p = [%f, %f, %f]\n\n", msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);

        //printf("call back\n");


	if (T_list.size() > 0)	
	{
		for (std::list<geometry_msgs::TransformStamped>::iterator it = T_list.begin(); it != T_list.end(); ++it)
		{
			if ( msg->header.frame_id == it->header.frame_id &&  msg->child_frame_id == it->child_frame_id )
			{
				// replace
		                //printf("replacing\n");
				it->transform = msg->transform;
			}
			else
			{
		                //printf("pushing back\n");
				T_list.push_back( *msg );
			}
		}
	}
	else
	{
                //printf("pushing back\n");
		T_list.push_back( *msg );
	}

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_pose_static_transform_tf_broadcaster");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("camera_pose_static_transform_update", 10,  MyCallback);
	

 	tf::TransformBroadcaster br;

	ros::Duration sleeper(100/1000.0); // 10 hz


	// Is this a race condition ??  -- NO
	while(n.ok() )  //to see whether it's yet time to exit.
	{
                //printf("looping\n");
		ros::spinOnce();
		for (std::list<geometry_msgs::TransformStamped>::iterator it = T_list.begin(); it != T_list.end(); ++it)
		{
			tf::Transform transform;
			std::string parent_frame_id;
			std::string child_frame_id;

  			transform.setOrigin( tf::Vector3(it->transform.translation.x, it->transform.translation.y, it->transform.translation.z) );
  			transform.setRotation( tf::Quaternion(it->transform.rotation.x, it->transform.rotation.y, it->transform.rotation.z, it->transform.rotation.w) );
  			parent_frame_id = it->header.frame_id;
			child_frame_id = it->child_frame_id;

    			if (parent_frame_id.compare(child_frame_id) != 0)  //target frame and source frame are not the same
      			{	
				//printf("%s\n", parent_frame_id.c_str());
				//printf("%s\n", child_frame_id.c_str());			
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, child_frame_id));	
				//printf("..\n");
  			}
			//printf("%s - %s \n", parent_frame_id.c_str(), child_frame_id.c_str());
	 		
		}
		sleeper.sleep();
	}

	// write to rosbag
	rosbag::Bag bag;
        bag.open("transform_list.bag", rosbag::bagmode::Write);
	
	for (std::list<geometry_msgs::TransformStamped>::iterator it = T_list.begin(); it != T_list.end(); ++it)
	{
		bag.write("camera_pose_static_transform_update", ros::Time::now(), *it);
	}
	bag.close();


  	return 0;

}
