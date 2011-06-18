
// yliu 06/17/2011

#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include <string>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <camera_pose_calibration/CameraCalibration.h>

#include <kdl/frames.hpp>
#include <boost/regex.hpp>

std::string new_cam_id;
std::string urdf_tree;
std::string filename;
//ros::Publisher pub; 


void MyCallback(const camera_pose_calibration::CameraCalibration::ConstPtr& msg)
{
	
	int cnt = msg->camera_id.size();
	printf("Number of cameras received: %d\n", cnt);
	KDL::Rotation CamR[cnt];
	int new_cam_index = -1;
	KDL::Rotation R; // the rotation of the new cam w.r.t. the calibrated cam
	KDL::Vector p;  // the origin displacement of the new cam w.r.t to the calibrated cam
	double kRoll, kPitch, kYaw;

	for (int i=0; i<cnt; i++)
	{
		printf("\n");
		printf("camera_id  :%s", msg->camera_id[i].c_str());

		if (msg->camera_id[i].compare(new_cam_id) == 0)
		{
			printf("  --->  new camera identified!");
			new_cam_index = i;
		}
		printf("\n");

		CamR[i]=KDL::Rotation::Quaternion(msg->camera_pose[i].orientation.x, msg->camera_pose[i].orientation.y, msg->camera_pose[i].orientation.z, msg->camera_pose[i].orientation.w); // KDL::Rotation objects

	}
	
	printf("\n");
	
	int prev_calibrated_cam_index= -1;

	if( new_cam_index == -1)
	{
		ROS_ERROR("Couldn't identify the camera to be added :( Check the camera_id you provided...");
	}
	else
	{	
		
		if ( new_cam_index !=1)  //just randomly pick one that is not new.
		{
			prev_calibrated_cam_index = 1;
		}
		else
		{	
			prev_calibrated_cam_index =	2;
		}
		
		R = CamR[new_cam_index]*CamR[prev_calibrated_cam_index].Inverse();
		p.x(msg->camera_pose[new_cam_index].position.x - msg->camera_pose[prev_calibrated_cam_index].position.x);
		p.y(msg->camera_pose[new_cam_index].position.y - msg->camera_pose[prev_calibrated_cam_index].position.y);
		p.z(msg->camera_pose[new_cam_index].position.z - msg->camera_pose[prev_calibrated_cam_index].position.z);
	}
	
	R.GetRPY(kRoll, kPitch, kYaw);
	
	printf("Adding new camera to existed urdf tree ...\n\n");

	boost::regex re("</robot>");

	std::string::const_iterator start, end; 
   	start = urdf_tree.begin(); 
   	end = urdf_tree.end(); 
	boost::match_results<std::string::const_iterator> match;  //smatch
	boost::match_flag_type flags = boost::match_default;
	
	int location = 0;
	if(regex_search(start, end, match, re, flags))   //since there is only one occurrence of </robot> in the .urdf file
   	{
		//printf("%s  -  found\n", match.begin()->str().c_str());
		location =  match.position();
		//printf("location at: %d\n", location);

		//start=match[0].second;
		//printf("%d\n", match.size());   //number of sub-matches
		//urdf_tree.insert(match[0].(second-urdf_tree.begin()),"ddyy");
		//urdf_tree.insert(match.position()+8, "dl");
	}


	//creating urdf snippet
	
	std::ostringstream buffer1, buffer2;
	buffer1 << kRoll <<" " << kPitch <<" "<< kYaw;
	buffer2 << p.x() << " " << p.y() << " " << p.z();
	std::string urdf_snippet =
	 "  <!-- added after running camera_pose_calibration -->\n"
       	 "  <joint name=\""+ msg->camera_id[new_cam_index] +  "_joint\" type=\"fixed\">" + "\n" +
	 "     <origin rpy=\"" + buffer1.str() +  "\" xyz=\"" + buffer2.str() + "\"/>" + "\n" +
	 "     <parent link=\"" + msg->camera_id[prev_calibrated_cam_index] + "\"/>" + "\n" +
	 "     <child link=\"" + msg->camera_id[new_cam_index] + "\"/>" + "\n" +
	 "  </joint>" + "\n" +
	 "  <link name=\""+ msg->camera_id[new_cam_index] + "\" type=\"camera\"/>" + "\n";


	urdf_tree.insert(location, urdf_snippet);
	//printf("\n The updated urdf tree is \n\n %s \n\n", urdf_tree.c_str());
	
	std::ofstream writer(filename.c_str(), std::ios::trunc); // in ~/.ros
	
	if (writer.is_open())
  	{
		writer<<urdf_tree;
		printf("New urdf file generated.\n");
		printf("File path:  %s\n\n", filename.c_str());
  	}
	else
  	{
    		printf("Error opening file.\n\n");
 	}

	writer.close();
	
	//checking if the tree is still OK. 
	std::string ss="\"" + msg->camera_id[prev_calibrated_cam_index] +"\""; 
	std::string::const_iterator s1,e1;
	s1 = urdf_tree.begin();
        e1 = urdf_tree.end();
	boost::match_results<std::string::const_iterator> m;
	boost::match_flag_type f = boost::match_default;

	ss = "<link name=" + ss ;
	//printf("%s\n", ss.c_str());
	boost::regex pa(ss);  

	if(regex_search(s1, e1, m, pa, f)) 
	{
		printf("The parent link of [%s_joint] is verifed in the updated .urdf file (at %d)\n", msg->camera_id[new_cam_index].c_str(), int(m.position()));
	}
	else
	{
    		ROS_WARN("The parent link of [%s_joint] seems missing in the updated .urdf file.\n You will receive an error if you do\n\n $ rosrun urdf check_urdf your_urdf.urdf\n\n", msg->camera_id[new_cam_index].c_str()  );
	}	

	printf("Done.\n");
}

	
int main(int argc, char **argv)
{
	ros::init(argc, argv, "camer_pose_urdf_updater");
	ros::NodeHandle n;

	n.getParam("new_cam_id", new_cam_id);
	n.getParam("robot_description", urdf_tree);
	n.getParam("/camera_pose_urdf_updater/filename", filename);

	printf("\ncamera_pose_urdf_updater ready...\n\n");
	
	//printf("%d\n", int(urdf_tree.length()));
	 	

	//printf("\n The existed urdf tree is \n\n %s \n\n", urdf_tree.c_str());
	
	//pub = n.advertise<camera_pose_calibration::CameraPose>("camera_pose_transform", 1000);
	//ros::Duration(0.01).sleep();
	
	  

  	ros::Subscriber sub = n.subscribe("camera_calibration", 1000, MyCallback);

	ros::spin();
	return 0;

}



		//double roll,pitch,yaw;
		//geometry_msgs::Pose pose_msg = msg->camera_pose[i];
		//tf::Pose tf_pose;

		//tf::poseMsgToTF(pose_msg, tf_pose);
		//tf_pose.getBasis().getRPY(roll,pitch,yaw);
		
		//printf("RPY angles  : %f, %f, %f \n", roll,pitch,yaw);
