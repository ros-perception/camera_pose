

// yliu 07/30/2011

// playback calibrated transformations

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
#include <rosbag/view.h>
#include <boost/foreach.hpp>





static std::list<geometry_msgs::TransformStamped> T_list;


int main(int argc, char **argv)
{

	ros::init(argc, argv, "transform_playback_node");
        printf("%d\n", argc);
        std::string bag_name; 
        printf("transform_playback_node: play ");
        if ( argc < 2)
	{
        	bag_name = "/transform_list.bag";  
	}
	else
	{
		bag_name = argv[1];
		printf("%s\n", argv[1]);
	}
               
	ros::NodeHandle n;
	
	// read the bag
	rosbag::Bag bag;
    	bag.open(bag_name, rosbag::bagmode::Read);

        std::vector<std::string> topics;
    	topics.push_back(std::string("camera_pose_static_transform_update"));

    	rosbag::View view(bag, rosbag::TopicQuery(topics));

    	BOOST_FOREACH(rosbag::MessageInstance const m, view)
    	{
       		geometry_msgs::TransformStamped::ConstPtr ts = m.instantiate<geometry_msgs::TransformStamped>();
       	 	if (ts != NULL)
            		T_list.push_back( *ts);

	}

    	bag.close();


 	tf::TransformBroadcaster br;

	ros::Duration sleeper(100/1000.0); // 10 hz


	//
	while(n.ok() )  //to see whether it's yet time to exit.
	{
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

  	return 0;

}
