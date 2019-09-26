#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace odometry_source_plugins{

  class OdomBroadcaster : public nodelet::Nodelet
  {
  private:
      ros::NodeHandle nh;
      ros::NodeHandle _nh;
  
      ros::Subscriber odom_yaw_sub;
      ros::Subscriber odom_x_sub;
      ros::Subscriber odom_y_sub;
  
      nav_msgs::Odometry odom_msg;
      ros::Publisher odom_pub;
  
      ros::Timer control_tim;
  
      std::string odom_frame;
      std::string base_frame;
  
      double _yaw;
      double _x;
      double _y;
  
      void odomYawCallback(const std_msgs::Float32::ConstPtr& yaw);
      void odomXCallback(const std_msgs::Float32::ConstPtr& x);
      void odomYCallback(const std_msgs::Float32::ConstPtr& y);
  
      void tf_publish(geometry_msgs::Pose);
  
      void TimerCallback(const ros::TimerEvent& event);
  
  public:
      virtual void onInit();
  };
  
  void OdomBroadcaster::onInit(){
    nh = getNodeHandle();
    //constructor
    _nh = getPrivateNodeHandle();
  
    _yaw = _x = _y = 0.0;
  
    odom_yaw_sub = nh.subscribe<std_msgs::Float32>("odom/yaw", 10, &OdomBroadcaster::odomYawCallback, this);
    odom_x_sub = nh.subscribe<std_msgs::Float32>("odom/x", 10, &OdomBroadcaster::odomXCallback, this);
    odom_y_sub = nh.subscribe<std_msgs::Float32>("odom/y", 10, &OdomBroadcaster::odomYCallback, this);
  
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  
    _nh.param<std::string>("odom_frame", odom_frame, "odom");
    _nh.param<std::string>("base_frame", base_frame, "base_link");
  
    int ctrl_freq;
    _nh.param("publish_rate", ctrl_freq, 20);
  
    control_tim = nh.createTimer(ros::Duration(1.0 / ctrl_freq), &OdomBroadcaster::TimerCallback, this);
  
    NODELET_INFO("odom_broadcaster node has started.");
  
  }
  
  void OdomBroadcaster::odomYawCallback(const std_msgs::Float32::ConstPtr& yaw)
  {
      _yaw = (double)yaw->data;
  }
  
  void OdomBroadcaster::odomXCallback(const std_msgs::Float32::ConstPtr& x)
  {
      _x = (double)x->data;
  }
  
  void OdomBroadcaster::odomYCallback(const std_msgs::Float32::ConstPtr& y)
  {
      _y = (double)y->data;
  }
  
  void OdomBroadcaster::tf_publish(geometry_msgs::Pose pose0){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::poseMsgToTF(pose0,transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, base_frame));
  }
  
  void OdomBroadcaster::TimerCallback(const ros::TimerEvent& event)
  {
  
      odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(_yaw);
  
      odom_msg.pose.pose.position.x = _x;
      odom_msg.pose.pose.position.y = _y;
  
      odom_msg.header.frame_id = odom_frame;
      odom_msg.child_frame_id= base_frame;
  
      odom_msg.header.stamp = ros::Time::now();
  
      odom_pub.publish(odom_msg);
      tf_publish(odom_msg.pose.pose);
  }
  
}// namespace can_plugins
PLUGINLIB_EXPORT_CLASS(odometry_source_plugins::OdomBroadcaster, nodelet::Nodelet);
