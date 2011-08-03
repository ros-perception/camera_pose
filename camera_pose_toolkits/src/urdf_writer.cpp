

// yliu 07/22/2011

#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <string>
#include <time.h>
#include <sstream>
#include <iomanip>
#include <boost/regex.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>


class URDF_Writer
{
public:
	ros::NodeHandle nh;
	ros::Subscriber sub; 
	std::string parent_frame_id;
	std::string child_frame_id; 

	KDL::Tree kdl_tree;
	
	std::string urdf_tree;
	std::string output_filename;

	URDF_Writer()
	{
		
		nh.getParam("robot_description", urdf_tree);
		//nh.param("robot_description", urdf_tree, std::string() );  // string ( ) -- Construct string object, content is initialized to an empty string
		nh.getParam("urdf_output_filename", output_filename);
		
		sub = nh.subscribe("camera_pose_static_transform_update", 10,  &URDF_Writer::MyCallbackFunc, this); //
		// ISO C++ forbids taking the address of an unqualified or parenthesized non-static member function to form a pointer to member function.  Say ‘&URDF_Writer::MyCallbackFunc’

			   	
		if (!kdl_parser::treeFromString(urdf_tree, kdl_tree)) //create a KDL Tree from parameter server
		{
		      ROS_ERROR("Failed to construct kdl tree");
		      //return false;
		}
	}



	void MyCallbackFunc(const geometry_msgs::TransformStamped::ConstPtr& msg)
	{	
		
		//printf("From: %s\n", msg->header.frame_id.c_str());
		//printf("To  : %s\n", msg->child_frame_id.c_str());
		//printf("Q = [%f, %f , %f, %f]\n",msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
		//printf("p = [%f, %f, %f]\n\n", msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);



		// STEPS:
		// remove xml snippet from previous calibration
		// locate </robot> tag
		// create a new xml snippet, insert, write to disk
		// check if the tree is still OK. 


		// -----
		// KDL::
		KDL::Rotation R = KDL::Rotation::Quaternion(msg->transform.rotation.x,
			                  		    msg->transform.rotation.y,
				       			    msg->transform.rotation.z,
				       			    msg->transform.rotation.w);
	  	parent_frame_id = msg->header.frame_id;
		child_frame_id = msg->child_frame_id;
		
		double Roll, Pitch, Yaw;
		double px, py, pz;

		px = msg->transform.translation.x;
		py = msg->transform.translation.y;
		pz = msg->transform.translation.z;

		R.GetRPY(Roll, Pitch, Yaw);



		// ------------------------------
		// removing out-dated description  
		std::string::const_iterator start, end; 

		std::string snippet_to_remove_a = "((^)|(^ +))<joint name=\"" + child_frame_id +  "_joint\"" + ".*" + "</joint>\n";
		boost::regex re1a(snippet_to_remove_a.c_str());
		std::string snippet_to_remove_b = "((^)|(^ +))<link name=\"" + child_frame_id  + ".*" + "/>\n";
		boost::regex re1b(snippet_to_remove_b.c_str());
		std::string snippet_to_remove_c = "((^)|(^ +))<!-- added after running camera_pose_calibration -->\n";
		boost::regex re1c(snippet_to_remove_c.c_str());

		boost::match_results<std::string::const_iterator> match1;  
		boost::match_flag_type flag1 = boost::match_default;

		printf("\nRemoving out-dated description xml snippet that was previously added by urdf_writer (if found)...\n");

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


		// ----------------------

		printf("Adding the xml snippet for the new camera frame to the urdf file ...\n\n");





		// ----------------------
		// locating </robot> tag

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
			//urdf_tree.insert(match2[0].second-urdf_tree.begin()),"ddyy"); // y. d. :)
		}




		// ----------------------
		// creating description xml snippet, insert to urdf_tree and write to disk

		std::ostringstream buffer1, buffer2;
		buffer1 << Roll <<" " << Pitch <<" "<< Yaw;
		buffer2 << px << " " << py << " " << pz;  //
		std::string urdf_snippet =
		 "  <!-- added after running camera_pose_calibration -->\n"
	       	 "  <joint name=\""+ child_frame_id +  "_joint\" type=\"fixed\">" + "\n" +
		 "     <origin rpy=\"" + buffer1.str() +  "\" xyz=\"" + buffer2.str() + "\"/>" + "\n" +
		 "     <parent link=\"" + parent_frame_id + "\"/>" + "\n" +
		 "     <child link=\"" + child_frame_id + "\"/>" + "\n" +
		 "  </joint>" + "\n" +
		 "  <link name=\""+ child_frame_id + "\" type=\"camera\"/>" + "\n";

		urdf_tree.insert(location, urdf_snippet);

		std::ofstream wr(output_filename.c_str(), std::ios::trunc); // in ~/.ros

		if (wr.is_open())
	  	{
			wr<<urdf_tree;
			printf("New urdf file generated.\n");
			printf("File path:  %s\n\n", output_filename.c_str());
	  	}
		else
	  	{
	    		printf("Error opening file.\n\n");
	 	}

		wr.close();

		

		// ---------------------------------------
		// checking the integrity of the urdf tree | warning only...

		std::string ss="\"" + parent_frame_id +"\""; 
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
			printf("The parent link of [%s_joint] is verifed in the updated .urdf file (at %d)\n", child_frame_id.c_str(), int(m.position()));
		}
		else
		{
	    		ROS_WARN("The parent link of [%s_joint] seems missing in the updated .urdf file.\n You will receive an error if you do\n\n $ rosrun urdf check_urdf your_urdf.urdf\n\n", 
					child_frame_id.c_str()  );
		}	


	}

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "urdf_writer");

	URDF_Writer my_urdf_writer;	

	ros::spin();


  	return 0;

}
