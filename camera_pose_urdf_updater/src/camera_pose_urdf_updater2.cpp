// yliu 06/23/2011 for two cameras only.

#include "ros/ros.h"
#include "ros/topic.h"
#include <iostream>
#include "std_msgs/String.h"
#include <string>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <map>
#include <camera_pose_calibration/CameraCalibration.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl/frames.hpp>
#include <boost/regex.hpp>
#include <tf/transform_listener.h>
#include <time.h>
#include <boost/bind.hpp>
#include <math.h>
#include <camera_pose_calibration/Reset.h>








class Updater
{
public:
ros::NodeHandle n;
tf::TransformListener listener; // need some time to build up internal buffer
ros::Publisher pub;
//ros::Publisher reset_pub; 
ros::ServiceClient reset_client;

std::string new_cam_ns; //camera name space
std::string urdf_cam_ns;
std::string new_cam_id; //camera frame id
std::string urdf_cam_id;
std::string urdf_tree;
std::string output_filename;
KDL::Tree kdl_tree;
std::string mounting_frame;
tf::StampedTransform prev_s_transform;
int measurement_count;


void MyCallback(const camera_pose_calibration::CameraCalibration::ConstPtr& msg)
{
	measurement_count ++; 
	ros::Time TheMoment = msg->last_measurement_timestamp;
	
	//--------------------------------------------------------

    	tf::StampedTransform s_transform;
	int TransformFound = 0;	
	
	clock_t c_start, c_end;
	c_start = clock();
	while(TransformFound==0)
	{	
		TransformFound=1;
		try{
      			//listener.lookupTransform(mounting_frame, urdf_cam_id, ros::Time(0), s_transform); // get the latest available transform | from, to
			listener.lookupTransform(mounting_frame, urdf_cam_id, TheMoment, s_transform);
    		}
		catch (tf::TransformException ex){ 
			//ROS_ERROR("%s",ex.what());
			printf(".");
			TransformFound=0;
    		}
	}
	
	//printf("%f\n%f\n", TheMoment.toSec(), ros::Time::now().toSec());
		
	c_end = clock();
	//printf("Delay is %f\n", double(c_end - c_start )/CLOCKS_PER_SEC); // clock ticks per sec

	// If during calibration, the urdf camera pose relative to the mounting frame has changed, we need to clear all the previous measurement history saved by the /cal_optimizer node. 
	
	bool flag = false;

	//--- debugging info ---
	printf("\n");
	printf("\E[34m\033[1m-- Debug Info --\033[0m\n");
	printf("urdf camera pose change relative to the mounting frame\n");
	printf("measurement_count = %d \n", measurement_count);
	printf("    dx = %10f | < 0.0002 m\n", s_transform.getOrigin().x()-prev_s_transform.getOrigin().x());
	printf("    dy = %10f | < 0.0002 m\n", s_transform.getOrigin().y()-prev_s_transform.getOrigin().y());
	printf("    dz = %10f | < 0.0002 m\n", s_transform.getOrigin().z()-prev_s_transform.getOrigin().z());
	printf("dAngle = %10f | < 0.00174 rad\n", s_transform.getRotation().getAngle()-prev_s_transform.getRotation().getAngle());
	printf(" dAxis = %10f | > 0.99999\n\n\n", s_transform.getRotation().getAxis().dot( prev_s_transform.getRotation().getAxis() )/1.0 );
	
	if ( (measurement_count > 1) && ( ( fabs(s_transform.getOrigin().x()-prev_s_transform.getOrigin().x() ) > 0.0002) // 0.2 mm
				     		|| ( fabs(s_transform.getOrigin().y()-prev_s_transform.getOrigin().y() ) > 0.0002)
				     		|| ( fabs(s_transform.getOrigin().z()-prev_s_transform.getOrigin().z() ) > 0.0002)
				     		|| ( fabs(s_transform.getRotation().getAngle()-prev_s_transform.getRotation().getAngle() ) > 0.00174)  // 0.1 degree
				    		|| ( fabs( s_transform.getRotation().getAxis().dot( prev_s_transform.getRotation().getAxis() ) /1.0) <  0.99999) )  )  // aprox 0.2 degree 
	{
		flag = true;
		measurement_count = 0;
	}
	
	if (flag)
	{
		printf("\n\E[44m\033[1mDetect pose change of the urdf camera relative to the mounting frame!\033[0m\n\n");
		printf("Calling reset service...\n");

		camera_pose_calibration::Reset r_srv;
		if(reset_client.call(r_srv) )  //service calls are blocking
		{
			printf("\033[33m%ld\033[0m measurement histories are deleted.\n\n", r_srv.response.count);
			printf("\nPlease show cameras your checkerboard one more time :)\n\n\n\n");
			
			prev_s_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
			prev_s_transform.setRotation( tf::Quaternion(1.0, 0.0, 0.0, 0.0));
		}
		else
		{
			ROS_ERROR("Failed to reset the measurement history!.\n");
		}
		 
	}
	else
	{

		prev_s_transform.setOrigin(  s_transform.getOrigin() );
		prev_s_transform.setRotation(  s_transform.getRotation() );


		printf("--\n");
		//printf("%f, %f, %f\n", s_transform.getOrigin().x(), s_transform.getOrigin().y(), s_transform.getOrigin().z());
		//printf("%f, %f, %f, %f\n\n", s_transform.getRotation().x(), s_transform.getRotation().y(), s_transform.getRotation().z(),s_transform.getRotation().w());
	
		KDL::Rotation R1; // the rotation of the urdf cam frame w.r.t. the link frame on which the new sensor is mounted.
		KDL::Vector p1;  // the origin displacement of the urdf cam frame w.r.t to the link frame on which the new sensor is mounted.
		p1.x(s_transform.getOrigin().x());
		p1.y(s_transform.getOrigin().y());
		p1.z(s_transform.getOrigin().z());

		//printf("p1 is [%f, %f, %f]\n\n", p1.x(), p1.y(), p1.z());

		// Static public member function of KDL::Rotation
		R1 = KDL::Rotation::Quaternion(s_transform.getRotation().x(),
				               s_transform.getRotation().y(),
					       s_transform.getRotation().z(),
					       s_transform.getRotation().w());  //radian


		//--------------------------------------------------------


		int cnt = msg->camera_id.size();
		printf("Number of cameras received: %d\n", cnt);
		KDL::Rotation CamR[cnt];
		int new_cam_index = -1;
		int urdf_cam_index = -1;
		KDL::Rotation R2; // the rotation of the new cam w.r.t. the urdf cam
		KDL::Vector p2;  // the origin displacement of the new cam w.r.t to the calibrated (urdf) cam


		for (int i=0; i<cnt; i++)
		{
			printf("\n");
			printf("camera_id  :%s", msg->camera_id[i].c_str());

			if (msg->camera_id[i].compare(new_cam_id) == 0)
			{
				printf("  --->  new camera ");
				new_cam_index = i;
			}
			//
			if (msg->camera_id[i].compare(urdf_cam_id) == 0)
			{
				printf("  --->  urdf camera ");
				urdf_cam_index = i;
			}
			//
			//printf("\n");

			CamR[i]=KDL::Rotation::Quaternion(msg->camera_pose[i].orientation.x, msg->camera_pose[i].orientation.y, msg->camera_pose[i].orientation.z, msg->camera_pose[i].orientation.w); // KDL::Rotation objects

		}
	
		printf("\n");
	
		
		R2 = CamR[urdf_cam_index].Inverse() * CamR[new_cam_index];
		p2.x(msg->camera_pose[new_cam_index].position.x - msg->camera_pose[urdf_cam_index].position.x);
		p2.y(msg->camera_pose[new_cam_index].position.y - msg->camera_pose[urdf_cam_index].position.y);
		p2.z(msg->camera_pose[new_cam_index].position.z - msg->camera_pose[urdf_cam_index].position.z);
		//printf("p2' is [%f, %f, %f]\n\n", p2.x(), p2.y(), p2.z());
		p2 = CamR[urdf_cam_index].Inverse() * p2;
		//printf("p2 is [%f, %f, %f]\n\n", p2.x(), p2.y(), p2.z());

		//--------------------------------------------------------
		KDL::Rotation R; 
		KDL::Vector p;  
		double kRoll, kPitch, kYaw;
		R =    R1 * R2 ;
		p = p1 + R1 * p2;
		R.GetRPY(kRoll, kPitch, kYaw);


		//--------------------------------------------------------


		std::string::const_iterator start, end; 

		std::string snippet_to_remove_a = "((^)|(^ +))<joint name=\"" + msg->camera_id[new_cam_index] +  "_joint\"" + ".*" + "</joint>\n";
		boost::regex re1a(snippet_to_remove_a.c_str());
		std::string snippet_to_remove_b = "((^)|(^ +))<link name=\"" + msg->camera_id[new_cam_index]  + ".*" + "/>\n";
		boost::regex re1b(snippet_to_remove_b.c_str());
		std::string snippet_to_remove_c = "((^)|(^ +))<!-- added after running camera_pose_calibration -->\n";
		boost::regex re1c(snippet_to_remove_c.c_str());

		boost::match_results<std::string::const_iterator> match1;  
		boost::match_flag_type flag1 = boost::match_default;

		printf("\nRemoving xml snippets previously added by camera_pose_urdf_updater...\n");
	
	   	start = urdf_tree.begin(); 
	   	end = urdf_tree.end(); 
		if (regex_search(start, end, match1, re1a, flag1)) 
		{	
			urdf_tree.erase(match1.position(), match1[0].second-match1[0].first);
			// match1[0].first : The start of the sequence of characters that matched the regular expression
			// match1[0].second : The end of the sequence of characters that matched the regular expression		
			//printf("%d\n", match1[0].second-match1[0].first);
			printf("...\n");
		}

	   	start = urdf_tree.begin(); 
	   	end = urdf_tree.end(); 
		if (regex_search(start, end, match1, re1b, flag1)) 
		{	
			urdf_tree.erase(match1.position(), match1[0].second-match1[0].first);
			//printf("%d\n", match1[0].second-match1[0].first);
			printf("...\n");
		}

	   	start = urdf_tree.begin(); 
	   	end = urdf_tree.end();
		if (regex_search(start, end, match1, re1c, flag1)) 
		{	
			urdf_tree.erase(match1.position(), match1[0].second-match1[0].first);
			//printf("%d\n", match1[0].second-match1[0].first);
			printf("...\n");
		}

		//
		printf("Adding the xml snippet for the new camera frame to the urdf file ...\n\n");


	   	start = urdf_tree.begin(); 
	   	end = urdf_tree.end(); 

		boost::match_results<std::string::const_iterator> match2;  //smatch
		boost::match_flag_type flags = boost::match_default;
	
		int location = 0;
		boost::regex re2("</robot>");
		if(regex_search(start, end, match2, re2, flags))   //since there is only one occurrence of </robot> in the .urdf file
	   	{
			location =  match2.position();
			//printf("location at: %d\n", location);
			//urdf_tree.insert(match2[0].second-urdf_tree.begin()),"ddyy");
		}


		//--------------------------------------------------------
		//creating xml snippet
	
		std::ostringstream buffer1, buffer2;
		buffer1 << kRoll <<" " << kPitch <<" "<< kYaw;
		buffer2 << p.x() << " " << p.y() << " " << p.z();
		std::string urdf_snippet =
		 "  <!-- added after running camera_pose_calibration -->\n"
	       	 "  <joint name=\""+ msg->camera_id[new_cam_index] +  "_joint\" type=\"fixed\">" + "\n" +
		 "     <origin rpy=\"" + buffer1.str() +  "\" xyz=\"" + buffer2.str() + "\"/>" + "\n" +
		 "     <parent link=\"" + mounting_frame + "\"/>" + "\n" +
		 "     <child link=\"" + msg->camera_id[new_cam_index] + "\"/>" + "\n" +
		 "  </joint>" + "\n" +
		 "  <link name=\""+ msg->camera_id[new_cam_index] + "\" type=\"camera\"/>" + "\n";

		urdf_tree.insert(location, urdf_snippet);
	
		std::ofstream writer(output_filename.c_str(), std::ios::trunc); // in ~/.ros
	
		if (writer.is_open())
	  	{
			writer<<urdf_tree;
			printf("New urdf file generated.\n");
			printf("File path:  %s\n\n", output_filename.c_str());
	  	}
		else
	  	{
	    		printf("Error opening file.\n\n");
	 	}

		writer.close();



		//--------------------------------------------------------

		//checking if the tree is still OK. 
		std::string ss="\"" + msg->camera_id[urdf_cam_index] +"\""; 
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
	    		ROS_WARN("The parent link of [%s_joint] seems missing in the updated .urdf file.\n You will receive an error if you do\n\n $ rosrun urdf check_urdf your_urdf.urdf\n\n", 
					msg->camera_id[new_cam_index].c_str()  );
		}	


		//--------------------------------------------------------
		// publish updated static transform information 
			
		geometry_msgs::TransformStamped T;
		T.transform.translation.x = p.x();
		T.transform.translation.y = p.y();
		T.transform.translation.z = p.z();
		double x1,y1,z1,w1;
		R.GetQuaternion( x1,y1,z1,w1);
		T.transform.rotation.x = x1;
		T.transform.rotation.y = y1;
		T.transform.rotation.z = z1; 
		T.transform.rotation.w = w1;

		T.header.frame_id = mounting_frame;
		T.child_frame_id = new_cam_id;
		pub.publish( T );

		//--------------------------------------------------------
		printf("Done.\n");
	}
}

};







int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_pose_urdf_updater2");
	
	Updater my_updater;

	my_updater.measurement_count = 0;
	my_updater.prev_s_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	my_updater.prev_s_transform.setRotation( tf::Quaternion(1.0, 0.0, 0.0, 0.0));
		
	my_updater.n.getParam("new_cam_ns", my_updater.new_cam_ns);
	my_updater.n.getParam("urdf_cam_ns", my_updater.urdf_cam_ns);

	std::string tp1;
	tp1 = my_updater.new_cam_ns + "/camera_info";
	sensor_msgs::CameraInfo::ConstPtr msg1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(tp1, my_updater.n);
	my_updater.new_cam_id = msg1->header.frame_id;

	std::string tp2;
	tp2 = my_updater.urdf_cam_ns + "/camera_info";
	sensor_msgs::CameraInfo::ConstPtr msg2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(tp2, my_updater.n);
	my_updater.urdf_cam_id = msg2->header.frame_id;

	printf("%s\n", my_updater.new_cam_id.c_str());
	printf("%s\n", my_updater.urdf_cam_id.c_str());
	
	my_updater.n.getParam("mounting_frame", my_updater.mounting_frame);


	my_updater.n.getParam("robot_description", my_updater.urdf_tree);
	//my_updater.n.param("robot_description", urdf_tree, std::string() );  // string ( ) -- Construct string object, content is initialized to an empty string

	my_updater.n.getParam("/camera_pose_urdf_updater/output_filename", my_updater.output_filename);

	printf("\n\033[32;1m camera_pose_urdf_updater ready...\033[0m\n\n");


//---------------------------------------------------
   	
	if (!kdl_parser::treeFromString(my_updater.urdf_tree, my_updater.kdl_tree)) //create a KDL Tree from parameter server
	{
	      ROS_ERROR("Failed to construct kdl tree");
	      return false;
	}
//---------------------------------------------------
	//reset_pub = n.advertise<std_msgs::Empty>("reset", 10);

	my_updater.reset_client = my_updater.n.serviceClient<camera_pose_calibration::Reset>("reset_measurement_history");

 	my_updater.pub = my_updater.n.advertise<geometry_msgs::TransformStamped>("camera_pose_static_transform_update", 10); // this is to provide an immediate way to examine the newly added frame. 
	ros::Subscriber sub = my_updater.n.subscribe("camera_calibration", 1000, &Updater::MyCallback, &my_updater);

	//ros::Subscriber sub = my_updater.n.subscribe<camera_pose_calibration::CameraCalibration>("camera_calibration", 1000, boost::bind(&Updater::MyCallback, &my_updater, _1));
	//// Note that you need to explicitly specify the template parameter here because it cannot be deduced from the value returned by bind()
	//// For regular function pointers, it is optional to use the address-of operator (&) when taking the address of a function, but it is required for taking the address of member functions.
	ros::spin();





	return 0;
   }
