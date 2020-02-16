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
  
      double _yaw[2]={0,0};
      double _x[2]={0,0};
      double _y[2]={0,0};
      double x0;
      double y0;
      double yaw0;

      bool InvertX = false;
  	  bool InvertY = false;
  	  bool InvertZ = false;

      int ctrl_freq;
  
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
    _nh = getPrivateNodeHandle();
  
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

    _nh.param<std::string>("odom_frame", odom_frame, "odom");
    _nh.param<std::string>("base_frame", base_frame, "base_link");

    _nh.param<double>("x0", x0, 0);
    _nh.param<double>("y0", y0, 0);
    _nh.param<double>("yaw0", yaw0, 0);

    _nh.param<bool>("invert_x", this->InvertX, false);
    _nh.param<bool>("invert_y", this->InvertY, false);
    _nh.param<bool>("invert_z", this->InvertZ, false);
  
    _nh.param("publish_rate", ctrl_freq, 20);

    odom_yaw_sub = nh.subscribe<std_msgs::Float32>("odom/yaw", 10, &OdomBroadcaster::odomYawCallback, this);
    odom_x_sub = nh.subscribe<std_msgs::Float32>("odom/x", 10, &OdomBroadcaster::odomXCallback, this);
    odom_y_sub = nh.subscribe<std_msgs::Float32>("odom/y", 10, &OdomBroadcaster::odomYCallback, this);
  
    control_tim = nh.createTimer(ros::Duration(1.0 / ctrl_freq), &OdomBroadcaster::TimerCallback, this);
  
    NODELET_INFO("odom_broadcaster node has started.");
  }
  
  void OdomBroadcaster::odomYawCallback(const std_msgs::Float32::ConstPtr& yaw)
  {
      if(InvertZ)
      {
        _yaw[0] = (double)yaw->data*-1 + yaw0;
      }
      else
      {
        _yaw[0] = (double)yaw->data + yaw0;
      }
  }
  
  void OdomBroadcaster::odomXCallback(const std_msgs::Float32::ConstPtr& x)
  {
    if(InvertX)
    {
      _x[0] = (double)x->data*-1 + x0;
    }
    else
    {
      _x[0] = (double)x->data + x0;
    } 
  }
  
  void OdomBroadcaster::odomYCallback(const std_msgs::Float32::ConstPtr& y)
  {
    if(InvertY)
    {
      _y[0] = (double)y->data*-1 + y0;
    }
    else
    {
      _y[0] = (double)y->data + y0;
    }
  }
  
  void OdomBroadcaster::tf_publish(geometry_msgs::Pose pose0){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::poseMsgToTF(pose0,transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, base_frame));
  }
  
  void OdomBroadcaster::TimerCallback(const ros::TimerEvent& event)
  {
    double vx,vy;
    vx=(_x[0]-_x[1])/ctrl_freq;
    vy=(_y[0]-_y[1])/ctrl_freq;
    odom_msg.twist.twist.linear.x = vx*cos(_yaw[0])+vy*sin(_yaw[0]);
    odom_msg.twist.twist.linear.y = -vx*sin(_yaw[0])+vy*cos(_yaw[0]);
    odom_msg.twist.twist.angular.z = (_yaw[1]-_yaw[0])/ctrl_freq;

    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(_yaw[0]);
  
    odom_msg.pose.pose.position.x = _x[0];
    odom_msg.pose.pose.position.y = _y[0];
  
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id= base_frame;
  
    odom_msg.header.stamp = ros::Time::now();
  
    odom_pub.publish(odom_msg);
    tf_publish(odom_msg.pose.pose);

    _yaw[1]=_yaw[0];
    _x[1] = _x[0];
    _y[1] = _y[0];
  }
  
}// namespace can_plugins
PLUGINLIB_EXPORT_CLASS(odometry_source_plugins::OdomBroadcaster, nodelet::Nodelet);
