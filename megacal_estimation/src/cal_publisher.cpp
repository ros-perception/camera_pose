


#include <sstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <megacal_estimation/CameraPose.h>
#include <megacal_estimation/transform_finder.h>

using namespace megacal_estimation;


struct CameraConfig
{
  geometry_msgs::TransformStamped orig_transform;
  std::string new_child_frame_id;
  std::string new_parent_frame_id;
};

void operator >> (const YAML::Node& node, geometry_msgs::CamConfig config)
{
  node["position"]["x"]    >> config.orig_transform.position.x;
  node["position"]["y"]    >> config.orig_transform.position.y;
  node["position"]["z"]    >> config.orig_transform.position.z;
  node["orientation"]["x"] >> config.orig_transform.orientation.x;
  node["orientation"]["y"] >> config.orig_transform.orientation.y;
  node["orientation"]["z"] >> config.orig_transform.orientation.z;
  node["orientation"]["w"] >> config.orig_transform.orientation.w;
  node["tf"]["calibrated_frame"] >> config.orig_transform.child_frame_id;
  node["tf"]["child_frame"] >> config.new_child_frame_id;
  node["tf"]["parent_frame"] >> config.new_child_frame_id;
}

void generateCameraList(const std::string& cam_yaml, std::vector<CameraConfig>& cameras)
{
  std::stringstream cal_yaml_stream(cal_yaml);
  YAML::Parser parser(cal_yaml_stream);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  cameras.resize(doc.size());
  for(unsigned int i=0; i<doc.size(); i++)
    doc[i] >> cameras[i];
}

struct ThreadInfo
{
  boost::shared_ptr<TransformFinder> tf_finder;
  boost::shared_ptr<boost::thread> thread_ptr;
};

class CalPublisher
{
private:
  ros::NodeHandle nh_;
  std::vector<CameraPose> cameras_;
  std::vector<ThreadInfo> threads_;
  boost::mutex mutex_;

public:
  CalPublisher()
  {
    ros::NodeHandle pnh("~");

    // Extract the cameras poses from the yaml string on param server
    std::string cal_yaml;
    if(!pnh.getParam("cal_estimate", cal_yaml))
      ROS_FATAL("Could not find parameter [~cal_estimate]. Shutting down cal_publisher node");
    generateCameraList(cal_yaml, cameras_);

    // Get the root frame from the calibrated transforms
    std::string root_id;
    pnh.param<std::string>("root_id", root_id, "world_frame");
    for (unsigned int i=0; i < cameras_.size(); i++)
      cameras_[i].orig_transform.header.frame_id = root_id;

    threads_.resize(cameras_.size());
    for (unsigned int i=0; i < threads_.size(); i++)
    {
      threads_.tf_finder.reset(new TransformFinder(cameras_[i].orig_transform,
						   cameras_[i].new_child_frame_id,
						   cameras_[i].new_parent_frame_id));
      threads_.thread_ptr.reset(new boost::thread( boost::bind(&TransformFinder::start, threads_[i].tf_finder)) );
    }

    timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(&CalPublisher::publishTF, this, _1));
  }

  void join()
  {
    for (unsigned int i=0; i < threads_.size(); i++)
    {
      threads_[i]->thread_ptr->join();
      printf("Thread %u: Joined\n", i);
    }
  }

private:
  void publishTF(const ros::TimerEvent& event)
  {
    printf("**** Publishing TF ****\n");
    for (unsigned int i=0; i < threads_.size(); i++)
    {
      if (threads_.tf_finder->found())
	printf("%u) Found\n", i);
      else
	printf("%u) Waiting\n", i);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cal_publisher");
  CalPublisher cal_publisher;
  ros::spin();
  cal_publisher.join();
}



