#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "goal_tool.h"

namespace rviz
{
Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';
  topic_property_ = new StringProperty( "Topic", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName( "3D Nav Goal" );
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1 );
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  ROS_WARN("3D Goal Set");
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  
  // 使用tf2的Quaternion替代tf::Quaternion
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  
  // 直接创建geometry_msgs::PoseStamped消息
  geometry_msgs::PoseStamped goal;
  
  // 设置消息头
  goal.header.frame_id = fixed_frame;
  goal.header.stamp = ros::Time::now();
  
  // 设置位置
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  
  // 设置姿态
  goal.pose.orientation.x = quat.x();
  goal.pose.orientation.y = quat.y();
  goal.pose.orientation.z = quat.z();
  goal.pose.orientation.w = quat.w();
  
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  
  pub_.publish(goal);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::Goal3DTool, rviz::Tool )