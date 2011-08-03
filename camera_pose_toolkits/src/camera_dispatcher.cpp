// yliu, Jul 26, 2011

// camera_dispatcher, relay selected topics

// camera_a/image_rect -
// camera_b/image_rect - 
// camera_c/image_rect -
// ...


#include <cstdio>
#include <vector>
#include <list>
#include "ros/console.h"
#include "std_msgs/String.h"
#include "camera_pose_toolkits/Switch.h"
#include "ros/ros.h"
#include "ros/topic.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
//#include <boost/bind.hpp>


static ros::NodeHandle *g_node = NULL;
static bool g_advertised1 = false;
static bool g_advertised2 = false;

static std::string g_output_topic1;
static std::string g_output_topic2;

static ros::Publisher g_pub_1;
static ros::Publisher g_pub_2;

static bool save_bandwidth = false;


static ros::Publisher g_pub_selected; //selected camera namespace

ros::Subscriber sub_1;
ros::Subscriber sub_2;

struct sub_pair
{
  ros::Subscriber sub1; //image_rect
  ros::Subscriber sub2; //camera_info
};

static std::list<struct sub_pair> g_sub_pairs;
static std::list<struct sub_pair>::iterator g_selected = g_sub_pairs.end();



void in_cb1(const sensor_msgs::CameraInfo::ConstPtr& msg, std::string & s1)
{
  if (!g_advertised1)
  {
    ROS_INFO("advertising 1");
    g_pub_1 = g_node->advertise<sensor_msgs::CameraInfo>(g_output_topic1, 10); //not latched
    g_advertised1 = true;
  }
  if (s1 == g_selected->sub1.getTopic() )
  {
    //printf("relaying 1...");
    g_pub_1.publish(msg);
  }
}

void in_cb2(const sensor_msgs::Image::ConstPtr& msg, std::string & s2)
{ 
  if (!g_advertised2)
  {
    ROS_INFO("advertising 2");
    g_pub_2 = g_node->advertise<sensor_msgs::Image>(g_output_topic2, 10); //not latched
    g_advertised2 = true;
  }
  if (s2 == g_selected->sub2.getTopic() )
  {
    //printf("relaying 2...");
    g_pub_2.publish(msg);
  }
}


bool switch_srv_cb(camera_pose_toolkits::Switch::Request& req,
		  camera_pose_toolkits::Switch::Response& res)
{
 
  // Check that it's not already in our list
  // spin through our vector of inputs and find this guy
  // if already there, select it
  // if not there, add it, and then select it
  for (std::list<sub_pair>::iterator it = g_sub_pairs.begin(); it != g_sub_pairs.end(); ++it)
  {
    if (ros::names::resolve(it->sub2.getTopic()) == ros::names::resolve(req.camera_ns + "/image_rect"))
    {
      printf(" subscribers to [%s] and [%s] aready in the list\n", it->sub1.getTopic().c_str(), it->sub2.getTopic().c_str());
      g_selected = it;
      printf("now listening to [%s] and [%s]\n", it->sub1.getTopic().c_str(), it->sub2.getTopic().c_str());

      std_msgs::String t;
      t.data = req.camera_ns;
      g_pub_selected.publish(t);
      return true;
    }
  }
  
  printf("adding subscribers to [%s/camera_info] and [%s/image_rect] to the list\n", req.camera_ns.c_str(), req.camera_ns.c_str());
  std::string s1 = req.camera_ns+"/camera_info";
  std::string s2 = req.camera_ns+"/image_rect";
 
  sub_pair sp;
  try
  {//sub_1
   sp.sub1  = g_node->subscribe<sensor_msgs::CameraInfo>(req.camera_ns+"/camera_info", 10, boost::bind(in_cb1, _1,  s1 ));
  }
  catch(ros::InvalidNameException& e)
  {
    ROS_WARN("failed to add subscriber to topic [%s/camera_info],  because it's an invalid name: %s", req.camera_ns.c_str(), e.what());
    return false;
  }

  try
  {//sub_2
   sp.sub2  = g_node->subscribe<sensor_msgs::Image>(req.camera_ns+"/image_rect", 10, boost::bind(in_cb2, _1,  s2 ));
  }
  catch(ros::InvalidNameException& e)
  {
    ROS_WARN("failed to add subscriber to topic [%s/image_rect],  because it's an invalid name: %s", req.camera_ns.c_str(), e.what());
    return false;
  }
  
 
  //sp.sub1 = sub_1;
  //sp.sub2 = sub_2;

  if (save_bandwidth == true)
  {
    if (g_sub_pairs.size() > 0) // for topics that are not relayed, unsubsribe.
    {  for ( std::list<sub_pair>::iterator it = g_sub_pairs.begin(); it != g_sub_pairs.end(); ++it)
       {
         it->sub1.shutdown(); 
         it->sub2.shutdown();
       }
    }
    g_sub_pairs.clear();
  }
  

  g_sub_pairs.push_back( sp );
  printf("%d\n", int(g_sub_pairs.size()));
  g_selected = g_sub_pairs.end();
  g_selected --; //important
 
  std_msgs::String t;
  t.data = req.camera_ns;
  g_pub_selected.publish(t);

  printf("now listening to [%s] and [%s]\n", g_selected->sub1.getTopic().c_str(), g_selected->sub2.getTopic().c_str());
  //printf("now listening to [%s] and [%s]\n", sp.sub1.getTopic().c_str(), sp.sub2.getTopic().c_str());

  return true;
}





int main(int argc, char **argv)
{
  //std::vector<std::string> args;
  //ros::removeROSArgs(argc, (const char**)argv, args);
  //printf("%s", args[1].c_str());
  //if (args.size() < 1)
  //{
  //  printf("\nusage: camera_dispatcher OUTPUT_NS\n\n");
  //  return 1;
  //}
  //std::string output_ns;
  //output_ns = args[1];

    
  ros::init(argc, argv, std::string("camera_dispatcher"), ros::init_options::AnonymousName);
  //Anonymize the node name. Adds a random number to the end of your node's name, to make it unique. 
  
  printf("%d\n", argc);
  printf("%s\n", argv[0]);
  printf("%s\n", argv[1]);
  
  if (argc < 2)
  {
    printf("\nusage: rosrun camera_pose_toolkits camera_dispatcher_node OUTPUT_NS\n\n");
    return 1;
  }
  
  ros::NodeHandle n;
  g_node = &n;
  
  std::string output_ns;
  output_ns = argv[1];
  // 
  g_output_topic1 = output_ns + "/camera_info";
  g_output_topic2 = output_ns + "/image_rect";


  // Put our API into the "camera_mux" namespace 
  ros::NodeHandle d_nh(argv[1]);//"camera_dispatcher"
  // Latched publisher for selected input topic name
  g_pub_selected = d_nh.advertise<std_msgs::String>(std::string("selected"), 1, true); // latched, slow changing
  //When a connection is latched, the last message published is saved and automatically sent to any future subscribers that connect.
  // New service
  ros::ServiceServer ss_switch = d_nh.advertiseService(std::string("switch"), switch_srv_cb);

  ros::spin();

  for (std::list<sub_pair>::iterator it = g_sub_pairs.begin(); it != g_sub_pairs.end(); ++it)
  {
    it->sub1.shutdown();
    it->sub2.shutdown();
  }

  g_sub_pairs.clear();
  return 0;
}






