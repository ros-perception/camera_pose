// yliu 
// transform_finder
// urdf_cam_frame | new_cam_frame | mounting_frame
// 06/23/2011
// 08/03/2011

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
#include <iomanip>
#include <vector>





class T_Finder
{
public:


ros::NodeHandle n;
tf::TransformListener listener; // need some time to build up internal buffer
ros::Publisher pub;
//ros::Publisher reset_pub; 
//ros::ServiceClient reset_client;
ros::ServiceClient monitor_client;

std::string new_cam_ns; //camera name space
std::string urdf_cam_ns;
std::string new_cam_id; //camera frame id
std::string urdf_cam_id;
// **
// **
// **
std::string mounting_frame;
tf::StampedTransform prev_s_transform;
//int measurement_count;
int printOnce_f;
int printOnce_t;
int printOnce_a;
std::string f_log_file;
std::string t_log_file;
std::string avg_res_file;


ros::Time prev_stamp0;
bool is_1st_time;
bool reset_flag;
std::vector<KDL::Twist> best_prev_twists; // best twists for different tf configurations between mountin frame and urdf cam frame.
std::vector<KDL::Frame> best_prev_frames; 
std::vector<int> prev_weights; 
std::vector<KDL::Twist>::iterator it;
std::vector<KDL::Twist> best_prev_twists_rel1; // best previous twists relative to the the first best twist. 

KDL::Twist prev_twist;
KDL::Frame prev_frame;
int prev_weight;
double prev_p[3];
double prev_rpy[3];
int callback_count;

//
std::string prev_new_cam_id;
std::string prev_urdf_cam_id;
std::string prev_mounting_frame;


void MyCallback(const camera_pose_calibration::CameraCalibration::ConstPtr& msg)
{


	n.getParam("new_cam_ns", new_cam_ns);
	n.getParam("urdf_cam_ns", urdf_cam_ns);

	std::string tp1;
	tp1 = new_cam_ns + "/camera_info";
	sensor_msgs::CameraInfo::ConstPtr msg1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(tp1,n);
	new_cam_id = msg1->header.frame_id;

	std::string tp2;
	tp2 = urdf_cam_ns + "/camera_info";
	sensor_msgs::CameraInfo::ConstPtr msg2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(tp2, n);
	urdf_cam_id = msg2->header.frame_id;
	
	n.getParam("mounting_frame", mounting_frame);

	printf("New camera frame: %s\n", new_cam_id.c_str());
	printf("New camera's mounting frame: %s\n", mounting_frame.c_str());
	printf("Urdf camera frame: %s\n", urdf_cam_id.c_str());
	
        // ------------------------------------------------

        // reset the lists if the target frames are found different from last time
        if (callback_count > 0 )
	{
		if (  (prev_new_cam_id != new_cam_id) || (prev_mounting_frame != mounting_frame) || (prev_urdf_cam_id != urdf_cam_id) )
		{
			best_prev_twists.clear();
			best_prev_frames.clear();
			best_prev_twists_rel1.clear();
			prev_weights.clear();
		        callback_count = 0;
                        is_1st_time = true;
	                printOnce_f = 1;
	                printOnce_t = 1;
	                printOnce_a = 1;

		        printf("Note: change of frames!\n ");
		}
	}




	callback_count++;
	printf("Calibration: \033[32;1m%d\033[0m\n", callback_count);	
	
	printf("--%f\n", msg->time_stamp[0].toSec());
	printf("--%f\n", prev_stamp0.toSec());
	if ( (msg->time_stamp[0].toSec() != prev_stamp0.toSec()) && (is_1st_time == false) )   // check if the cache for previous measurements has been cleared.
	{
		reset_flag = true;
	}
	//prev_stamp0.sec = msg->time_stamp[0].sec;
	//prev_stamp0.nsec = msg->time_stamp[0].nsec;
	prev_stamp0 = msg->time_stamp[0];

	ros::Time TheMoment = msg->time_stamp[msg->m_count-1];
	
	//--------------------------------------------------------

    	tf::StampedTransform s_transform;
	int TransformFound = 0;	
	
	while(TransformFound==0)
	{	
		TransformFound=1;
		try{
      			// get the transform | from, to
			listener.lookupTransform(mounting_frame, urdf_cam_id, TheMoment, s_transform);
    		}
		catch (tf::TransformException ex){ 
			//ROS_ERROR("%s",ex.what());
			printf(".");
			TransformFound=0;
    		}
	}
	
	

	


	printf("----\n");

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

	//-debug-
	printf("TF Quaternion:\n %10f, %10f, %10f, %10f\n", s_transform.getRotation().x(), s_transform.getRotation().y(), s_transform.getRotation().z(), s_transform.getRotation().w() );

	//--------------------------------------------------------


	int cnt = msg->camera_id.size();
	//printf("Number of cameras received: %d\n", cnt);
	KDL::Rotation CamR[cnt];
	int new_cam_index = -1;
	int urdf_cam_index = -1;
	KDL::Rotation R2; // the rotation of the new cam w.r.t. the urdf cam
	KDL::Vector p2;  // the origin displacement of the new cam w.r.t to the calibrated (urdf) cam


	for (int i=0; i<cnt; i++)
	{
		//printf("\n");
		//printf("camera_id  :%s", msg->camera_id[i].c_str());

		if (msg->camera_id[i].compare(new_cam_id) == 0)
		{
			//printf("  --->  new camera ");
			new_cam_index = i;
		}
		//
		if (msg->camera_id[i].compare(urdf_cam_id) == 0)
		{
			//printf("  --->  urdf camera ");
			urdf_cam_index = i;
		}
		//
		//printf("\n");

		CamR[i]=KDL::Rotation::Quaternion(msg->camera_pose[i].orientation.x, msg->camera_pose[i].orientation.y, msg->camera_pose[i].orientation.z, msg->camera_pose[i].orientation.w); // KDL::Rotation objects
		
		//-debug-
		printf("CC-%d Quaternion:\n %10f, %10f, %10f, %10f\n",i, msg->camera_pose[i].orientation.x, msg->camera_pose[i].orientation.y, msg->camera_pose[i].orientation.z, msg->camera_pose[i].orientation.w );


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
	double px, py, pz;
	double qx,qy,qz,qw;
	R =    R1 * R2 ;     // single pose best
	p = p1 + R1 * p2;    // single pose best
	px = p.x(); py = p.y(); pz = p.z(); 
	R.GetRPY(kRoll, kPitch, kYaw);
	R.GetQuaternion( qx,qy,qz,qw);

	//-debug-
	printf("Combined Quaternion:\n %10f, %10f, %10f, %10f \n\n",qx,qy,qz,qw);
	printf("Rotation M:\n %10f, %10f, %10f, \n %10f, %10f, %10f \n %10f, %10f, %10f \n", R.data[0],R.data[1],R.data[2],   R.data[3],R.data[4],R.data[5],   R.data[6],R.data[7],R.data[8] );

	//--------------------------------------------------------
	// Convert to twist, so we can average bests from different urdf-cam-frame/mounting-frame configurations
	KDL::Frame fm = KDL::Frame(R, p);
	KDL::Twist tw;
	tw = KDL::diff(KDL::Frame::Identity(), fm);
	//if ( is_1st_time == false )
	//{
	//	if (tw.rot.data[0]*prev_twist.rot.data[0]<0)  // regulate the sign, since we need to perform averaging later.
	//	{
	//		tw.rot.data[0] = -tw.rot.data[0];
	//		tw.rot.data[1] = -tw.rot.data[1];
	//		tw.rot.data[2] = -tw.rot.data[2];
	//	}
	//}
	double vx, vy, vz, wx, wy, wz;
	vx = tw.vel.data[0];
	vy = tw.vel.data[1];
	vz = tw.vel.data[2];
	wx = tw.rot.data[0];
	wy = tw.rot.data[1];
	wz = tw.rot.data[2];

	//--------------------------------------------------------
	// logging 
	printf("\nlogging... (.ros/)\n");
		
	time_t rawtime;
  	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	 
	
	std::ofstream fwriter(f_log_file.c_str(), std::ios::out | std::ios::app);  // append 
	std::ofstream twriter(t_log_file.c_str(), std::ios::out | std::ios::app);   
	

	if (fwriter.is_open())
  	{
		if (reset_flag == true)
		{
			fwriter<<"--urdf_cam_frame moved, measurement cache reset!--"<<std::endl;
		}

		if ( 	printOnce_f == 1 )
		{
			fwriter<<"----------------------"<<std::endl;
			fwriter<<"parent_frame: "<< mounting_frame <<std::endl;
			fwriter<<"child_frame:  "<< new_cam_id <<std::endl;
			printOnce_f = 0;
		}
	
		fwriter<< std::setw(3) << callback_count<<". " << timeinfo->tm_year+1900<<"-"
	               << std::setfill('0') << std::setw(2) << timeinfo->tm_mon+1<<"-"
	               << std::setfill('0') << std::setw(2) << timeinfo->tm_mday <<" "
                       << std::setfill('0') << std::setw(2) << timeinfo->tm_hour
                       <<":"<< std::setfill('0') << std::setw(2) << timeinfo->tm_min
		       <<":"<< std::setfill('0') << std::setw(2) << timeinfo->tm_sec;
		
		fwriter<<std::setfill(' ');
		fwriter<< " p:"<<std::setw(15)<< px
		       <<std::setw(15)<< py
		       <<std::setw(15)<< pz<< "      Q:";
		fwriter<<std::setw(15)<< qx
		       <<std::setw(15)<< qy 
		       <<std::setw(15)<< qz
		       <<std::setw(15)<< qw<<std::endl;
  	}
	else
  	{
    		printf("Error opening file.\n\n");
 	}
	

	if (twriter.is_open())
  	{
		if (reset_flag == true)
		{
			twriter<<"--urdf_cam_frame moved, measurement cache reset!--"<<std::endl;
		}	

		if (  printOnce_t == 1  )
		{
			twriter<<"----------------------"<<std::endl;
			twriter<<"parent_frame: "<< mounting_frame <<std::endl;
			twriter<<"child_frame:  "<< new_cam_id <<std::endl;
		        printOnce_t = 0;
		}

		twriter<< std::setw(3) << callback_count<<". "<< timeinfo->tm_year+1900<<"-"
	               << std::setfill('0') << std::setw(2) << timeinfo->tm_mon+1<<"-"
	               << std::setfill('0') << std::setw(2) << timeinfo->tm_mday <<" "
                       << std::setfill('0') << std::setw(2) <<timeinfo->tm_hour
		       <<":"<< std::setfill ('0') << std::setw(2) <<timeinfo->tm_min
                       <<":"<< std::setfill ('0') << std::setw(2) <<timeinfo->tm_sec;
		
		twriter<<std::setfill(' ');
		twriter<< " v:"<< std::setw(15)<<vx
		       <<std::setw(15)<<vy
		       <<std::setw(15)<<vz << "      w:"
		       <<std::setw(15)<<wx
		       <<std::setw(15)<<wy
		       <<std::setw(15)<<wz<<std::endl;
  	}
	else
  	{
    		printf("Error opening file. \n\n");
 	}

	twriter.close();
	fwriter.close();



	

	//--------------------------------------------------------
	// Averaging 




	if ( reset_flag == true)
	{
		best_prev_twists.push_back(prev_twist);
		best_prev_frames.push_back(prev_frame);
		
		// 'rel1' means relative to the 1st calibration result

		KDL::Twist prev_twist_rel1 = KDL::diff(     best_prev_frames[0], prev_frame);
		//// You shall not use twist diff(twist&, twist&) here!!
 		//KDL::Twist prev_twist_rel1 = KDL::diff(     best_prev_twists[0],
		//		                            prev_twist  );
		
		

	        printf("rel_to_1: %10f, %10f, %10f, %10f, %10f, %10f\n", prev_twist_rel1.vel.data[0], prev_twist_rel1.vel.data[1], prev_twist_rel1.vel.data[2], prev_twist_rel1.rot.data[0], prev_twist_rel1.rot.data[1], prev_twist_rel1.rot.data[2]);
		best_prev_twists_rel1.push_back(prev_twist_rel1);
		
		//weights
		prev_weights.push_back(prev_weight);		

	        std::ofstream iwriter("debug_info", std::ios::app); 
		if (iwriter.is_open()) // i for intermediate
  		{

                        iwriter<<"F"<< best_prev_frames.size() <<":     "<<" p:"<< std::setw(15)<<px
					  << std::setw(15)<<py
					  << std::setw(15)<<pz <<"      Q:"
					  << std::setw(15)<<qx
					  << std::setw(15)<<qy
					  << std::setw(15)<<qz
					  << std::setw(15)<<qw <<"    |    weight:"<<prev_weight<<std::endl;
			
	  	}
		else
	  	{
	    		printf("Error opening file.\n\n");
	 	}
		iwriter.close();
		
	}





	if ( best_prev_twists.size() > 0)
	{	
		double rsum[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		int c = (int)best_prev_twists.size();
		
		double prev_total_w = 0.0;
		for ( int k = 0; k < c; k++)
		{
			prev_total_w += (double)prev_weights[k];
		}
	
                for ( int i = 0; i < c; i++)
		{
			double w = prev_weights[i];
			// w = 1;
			rsum[0] += best_prev_twists_rel1[i].vel.data[0]*w;
			rsum[1] += best_prev_twists_rel1[i].vel.data[1]*w;
			rsum[2] += best_prev_twists_rel1[i].vel.data[2]*w;
			rsum[3] += best_prev_twists_rel1[i].rot.data[0]*w;
			rsum[4] += best_prev_twists_rel1[i].rot.data[1]*w;
			rsum[5] += best_prev_twists_rel1[i].rot.data[2]*w;			
		}	


		
		double c_w = (double)msg->m_count;
		double total_w = prev_total_w + c_w;

		KDL::Twist tw_rel1 = KDL::diff(best_prev_frames[0], fm);  // !!		
 

		KDL::Twist tw_av_rel1 = KDL::Twist(KDL::Vector( (rsum[0] + tw_rel1.vel.data[0]*c_w)/total_w,  (rsum[1] + tw_rel1.vel.data[1]*c_w)/total_w,  (rsum[2] + tw_rel1.vel.data[2]*c_w)/total_w), 
                                                   KDL::Vector( (rsum[3] + tw_rel1.rot.data[0]*c_w)/total_w,  (rsum[4] + tw_rel1.rot.data[1]*c_w)/total_w,  (rsum[5] + tw_rel1.rot.data[2]*c_w)/total_w ) ); //?
		KDL::Twist tw_av = KDL::addDelta(best_prev_twists[0], tw_av_rel1);  //dt is always 1.
		
		KDL::Frame frame_av = KDL::addDelta(KDL::Frame::Identity(), tw_av);
		KDL::Rotation R_av = frame_av.M;
		KDL::Vector p_av = frame_av.p; 

		px = p_av.x(); py = p_av.y(); pz = p_av.z(); 
		R_av.GetRPY(kRoll, kPitch, kYaw);
		R_av.GetQuaternion( qx,qy,qz,qw);

	}

	//-----------------------------------------------------
	std::ofstream awriter(avg_res_file.c_str(), std::ios::out | std::ios::app); 
	if (awriter.is_open())
  	{
		if (reset_flag == true)
		{
			awriter<<"urdf_cam_frame movded, measurement cache reset!"<<std::endl;
		}	
		if (printOnce_a == 1)
		{
			awriter<<"----------------------"<<std::endl;
			awriter<<"parent_frame: "<< mounting_frame <<std::endl;
			awriter<<"child_frame:  "<< new_cam_id <<std::endl;
			printOnce_a = 0;
		}	
		awriter<<  std::setw(3) << callback_count<<". ";
		
		awriter<<" p:"<<std::setw(15)<< px
		       <<std::setw(15)<< py
		       <<std::setw(15)<< pz <<"      Q:";
		awriter<<std::setw(15)<< qx
		       <<std::setw(15)<< qy 
		       <<std::setw(15)<< qz
		       <<std::setw(15)<< qw<<std::endl;
  	}
	else
  	{
    		printf("Error opening file. \n\n");
 	}
	awriter.close();


	prev_twist = tw;
	prev_frame = fm;
	prev_p[0]=p[0]; prev_p[1]=p[1]; prev_p[2]=p[2];
	prev_rpy[0]=kRoll; prev_rpy[1]=kPitch; prev_rpy[2]= kYaw;

	//--------------------------------------------------------
	


	//--------------------------------------------------------
	// publish updated static transform information 
		
	geometry_msgs::TransformStamped T;
	T.transform.translation.x = px;
	T.transform.translation.y = py;
	T.transform.translation.z = pz;

	T.transform.rotation.x = qx;
	T.transform.rotation.y = qy;
	T.transform.rotation.z = qz; 
	T.transform.rotation.w = qw;

	T.header.frame_id = mounting_frame;
	T.child_frame_id = new_cam_id;
	pub.publish( T );


	//--------------------------------------------------------




	//----------------------------------------------------------------
	is_1st_time = false;
	reset_flag = false;
	prev_weight = msg->m_count;


	//----------------------------------------------------------------
	prev_new_cam_id = new_cam_id;
	prev_urdf_cam_id = urdf_cam_id;
	prev_mounting_frame = mounting_frame;


	//----------------------------------------------------------------
	printf("Done.\n");

}

};







int main(int argc, char **argv)
{
	ros::init(argc, argv, "transform_finder");
	
	T_Finder my_finder;

	//my_finder.measurement_count = 0;
	my_finder.prev_s_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	my_finder.prev_s_transform.setRotation( tf::Quaternion(1.0, 0.0, 0.0, 0.0));
		


	// ***


	my_finder.prev_new_cam_id = "";
	my_finder.prev_urdf_cam_id = "";
	my_finder.prev_mounting_frame = "";

	// ** 
	// **

	printf("\n\033[32;1m transform_finder ready...\033[0m\n\n");


	my_finder.printOnce_f = 1;
	my_finder.printOnce_t = 1;
	my_finder.printOnce_a = 1;

	my_finder.f_log_file = "calibration_log_frame";
	my_finder.t_log_file = "calibration_log_twist";
	my_finder.avg_res_file = "averaged_result";
	
	my_finder.prev_stamp0 = ros::Time();
	my_finder.is_1st_time = true;
	my_finder.reset_flag = false;

	       
        std::ofstream iwriter("debug_info", std::ios::out | std::ios::trunc); 
	iwriter.close();

        std::ofstream awriter(my_finder.avg_res_file.c_str(), std::ios::out | std::ios::trunc); 
	awriter.close();
	
	my_finder.callback_count = 0;


	//---------------------------------------------------
	// **


	//---------------------------------------------------
	//reset_pub = n.advertise<std_msgs::Empty>("reset", 10);

        // ***
        //my_finder.monitor_client = my_finder.n.serviceClient<camera_pose_calibration::FramePair>("need_to_monitor_tf");
	//ros::Duration(0.2).sleep();

	//my_finder.reset_client = my_finder.n.serviceClient<camera_pose_calibration::Reset>("reset_measurement_history");

 	my_finder.pub = my_finder.n.advertise<geometry_msgs::TransformStamped>("camera_pose_static_transform_update", 10); // this is to provide an immediate way to examine the newly added frame. 
	ros::Subscriber sub = my_finder.n.subscribe("camera_calibration", 1000, &T_Finder::MyCallback, &my_finder);

	//ros::Subscriber sub = my_finder.n.subscribe<camera_pose_calibration::CameraCalibration>("camera_calibration", 1000, boost::bind(&T_Finder::MyCallback, &my_finder, _1));
	//// Note that you need to explicitly specify the template parameter here because it cannot be deduced from the value returned by bind()
	//// For regular function pointers, it is optional to use the address-of operator (&) when taking the address of a function, but it is required for taking the address of member functions.
	ros::spin();





	return 0;
   }
