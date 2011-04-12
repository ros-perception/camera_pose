
#include <kdl/frames.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer_client.h>

namespace megacal_estimation
{

void TransformMsgToKDL(const geometry_msgs::Transform& t, KDL::Frame& k)
{
  k = KDL::Frame(KDL::Rotation::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
		 KDL::Vector(t.translation.x, t.translation.y, t.translation.z));
}

void TransformKDLToMsg(const KDL::Frame& k, geometry_msgs::Transform& t)
{
  t.translation.x = k.p[0];
  t.translation.y = k.p[1];
  t.translation.z = k.p[2];

  k.M.GetQuaternion(t.rotation.x,
		    t.rotation.y,
		    t.rotation.z,
		    t.rotation.w);
}

class TransformFinder
{

private:
  const geometry_msgs::TransformStamped orig_transform_;
  const std::string child_id_;
  const std::string parent_id_;

  geometry_msgs::TransformStamped transform_out_;
  bool found_;

public:

  /**
   * Assumed tree:
   * Orig.Parent -> Parent -> Child -> Orig.Child
   * Give this assumed tree, lookup Orig.Parent->Parent and Child->Orig.Child in order to publish Parent->Child
   **/
  TransformFinder(const geometry_msgs::TransformStamped& orig_transform,
                  const std::string& child_id,
                  const std::string parent_id) :
    orig_transform_(orig_transform),
    child_id_(child_id),
    parent_id_(parent_id)
    { }

  void start()
  {
    tf2::BufferClient tf_client("tf2");

    // Lookup Child -> Orig.Child
    geometry_msgs::TransformStamped child_to_orig_child;
    ROS_INFO("Trying to find [%s] -> [%s]", child_id_.c_str(), orig_transform_.child_frame_id.c_str());
    child_to_orig_child = tf_client.lookupTransform(orig_transform_.child_frame_id, child_id_, ros::Time(0.0), ros::Duration(30.0));
    // Lookup Orig.Parent to Parent
    geometry_msgs::TransformStamped orig_parent_to_parent;
    ROS_INFO("Trying to find [%s] -> [%s]", orig_transform_.header.frame_id.c_str(), parent_id_.c_str());
    orig_parent_to_parent = tf_client.lookupTransform(parent_id_, orig_transform_.header.frame_id, ros::Time(0.0), ros::Duration(30.0));

    // orig = orig_parent_to_parent * parent_to_child * child_to_orig_child
    KDL::Frame op2p, c2oc, op2oc;
    TransformMsgToKDL(orig_parent_to_parent.transform, op2p);
    TransformMsgToKDL(child_to_orig_child.transform, c2oc);
    TransformMsgToKDL(orig_transform_.transform, op2oc);
    KDL::Frame p2c = op2p.Inverse() * op2oc * c2oc.Inverse();

    TransformKDLToMsg(p2c, transform_out_.transform);
    transform_out_.header.frame_id = parent_id_;
    transform_out_.child_frame_id = child_id_;
    found_ = true;
  }

  bool found()
  {
    return found_;
  }

  bool getTransform(geometry_msgs::TransformStamped& transform_out)
  {
    if (!found_)
      return false;
    transform_out = transform_out_;
    return true;
  }
};

}
