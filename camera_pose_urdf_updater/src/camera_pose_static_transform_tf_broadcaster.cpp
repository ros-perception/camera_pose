

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


void MyCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{	
	printf("Current best available calibration result:\n");
	printf("From: %s\n", msg->header.frame_id.c_str());
	printf("To  : %s\n", msg->child_frame_id.c_str());
	printf("Q = [%f, %f , %f, %f]\n",msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
	printf("p = [%f, %f, %f]\n\n", msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);

  	transform.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
  	transform.setRotation( tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w) );
  	parent_frame_id = msg->header.frame_id;
	child_frame_id = msg->child_frame_id;
	


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_pose_static_transform_tf_broadcaster");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("camera_pose_static_transform_update", 10,  MyCallback);
	

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
