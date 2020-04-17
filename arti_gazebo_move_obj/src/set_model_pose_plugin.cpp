/*
 * Copyright 2013 Open Source Robotics Foundation
 * Copyright 2019 ARTI - Autonomous Robot Technology GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <arti_gazebo_move_obj/set_model_pose_plugin.h>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

GZ_REGISTER_MODEL_PLUGIN(arti_gazebo_move_obj::SetModelPosePlugin)

namespace arti_gazebo_move_obj
{

SetModelPosePlugin::SetModelPosePlugin()
  : robot_namespace_(), command_topic_("command_pose"), odometry_topic_("odom"), odometry_frame_("odom"),
    model_frame_(), odometry_rate_(20.0), alive_(true)
{
}

SetModelPosePlugin::~SetModelPosePlugin()
{
  FiniChild();
}

// Load the controller
void SetModelPosePlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  model_ = parent;

  if (!sdf->HasElement("robotNamespace"))
  {
    ROS_INFO_NAMED("arti_gazebo_move_obj", "SetModelPosePlugin missing <robotNamespace>, defaults to \"%s\"",
                   robot_namespace_.c_str());
  }
  else
  {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  if (!sdf->HasElement("commandTopic"))
  {
    ROS_WARN_NAMED("arti_gazebo_move_obj", "SetModelPosePlugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                   robot_namespace_.c_str(), command_topic_.c_str());
  }
  else
  {
    command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
  }

  if (!sdf->HasElement("odometryTopic"))
  {
    ROS_WARN_NAMED("arti_gazebo_move_obj", "SetModelPosePlugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                   robot_namespace_.c_str(), odometry_topic_.c_str());
  }
  else
  {
    odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
  }

  if (!sdf->HasElement("odometryFrame"))
  {
    ROS_WARN_NAMED("arti_gazebo_move_obj", "SetModelPosePlugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                   robot_namespace_.c_str(), odometry_frame_.c_str());
  }
  else
  {
    odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
  }

  model_frame_ = model_->GetName();
  if (!sdf->HasElement("modelFrame"))
  {
    ROS_INFO_NAMED("arti_gazebo_move_obj", "SetModelPosePlugin (ns = %s) missing <modelFrame>, defaults to \"%s\"",
                   robot_namespace_.c_str(), model_frame_.c_str());
  }
  else
  {
    model_frame_ = sdf->GetElement("modelFrame")->Get<std::string>();
  }

  if (!sdf->HasElement("odometryRate"))
  {
    ROS_WARN_NAMED("arti_gazebo_move_obj", "SetModelPosePlugin (ns = %s) missing <odometryRate>, defaults to %f",
                   robot_namespace_.c_str(), odometry_rate_);
  }
  else
  {
    odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
  }

#if GAZEBO_MAJOR_VERSION >= 8
  last_odom_publish_time_ = model_->GetWorld()->SimTime();
  offset_ = model_->WorldPose();
#else
  last_odom_publish_time_ = model_->GetWorld()->GetSimTime();
  offset_ = model_->GetWorldPose().Ign();
#endif

  if (sdf->HasElement("initPose"))
  {
    offset_ += sdf->GetElement("initPose")->Get<ignition::math::Pose3d>();
  }

  // Ensure that ROS has been initialized and subscribe to cmd_vel
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("arti_gazebo_move_obj", "SetModelPosePlugin (ns = " << robot_namespace_
                                                                               << "). A ROS node for Gazebo has not been initialized, "
                                                                               << "unable to load plugin. Load the Gazebo system plugin "
                                                                               << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ros_node_.reset(new ros::NodeHandle(robot_namespace_));
  last_command_time_ = ros::Time::now();

  ROS_DEBUG_NAMED("arti_gazebo_move_obj", "OCPlugin (%s) has started",
                  robot_namespace_.c_str());

  const std::string tf_prefix = tf::getPrefixParam(*ros_node_);
  odometry_frame_ = tf::resolve(tf_prefix, odometry_frame_);
  model_frame_ = tf::resolve(tf_prefix, model_frame_);

  transform_broadcaster_.reset(new tf::TransformBroadcaster());

  // subscribe to the command topic
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
    command_topic_, 1, boost::bind(&SetModelPosePlugin::processCommand, this, _1), ros::VoidPtr(), &callback_queue_);
  command_sub_ = ros_node_->subscribe(so);

  odometry_pub_ = ros_node_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

  // start custom queue for diff drive
  callback_queue_thread_ = boost::thread(boost::bind(&SetModelPosePlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  update_connection_ =
    gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SetModelPosePlugin::UpdateChild, this));
}

void SetModelPosePlugin::UpdateChild()
{
  boost::mutex::scoped_lock scoped_lock(lock_);
  if ((ros::Time::now() - last_command_time_).toSec() <= 1.0)
  {
    const ignition::math::Pose3d pose(
      last_command_.position.x + offset_.Pos().X(),
      last_command_.position.y + offset_.Pos().Y(),
      last_command_.position.z + offset_.Pos().Z(),
      last_command_.orientation.w,
      last_command_.orientation.x,
      last_command_.orientation.y,
      last_command_.orientation.z);
    model_->SetWorldPose(pose);
  }

  if (odometry_rate_ > 0.0)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    const gazebo::common::Time current_time = model_->GetWorld()->SimTime();
#else
    const gazebo::common::Time current_time = model_->GetWorld()->GetSimTime();
#endif
    if ((current_time - last_odom_publish_time_).Double() >= (1.0 / odometry_rate_))
    {
      publishOdometry();
      last_odom_publish_time_ = current_time;
    }
  }
}

// Finalize the controller
void SetModelPosePlugin::FiniChild()
{
  alive_ = false;
  callback_queue_.clear();
  callback_queue_.disable();
  ros_node_->shutdown();
  callback_queue_thread_.join();
}

void SetModelPosePlugin::processCommand(const geometry_msgs::Pose::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock_);
  last_command_ = *cmd_msg;
  last_command_time_ = ros::Time::now();
}

void SetModelPosePlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (alive_ && ros_node_->ok())
  {
    callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void SetModelPosePlugin::publishOdometry()
{
  const ros::Time current_time = ros::Time::now();

#if GAZEBO_MAJOR_VERSION >= 8
  const ignition::math::Pose3d pose = model_->WorldPose();
#else
  const ignition::math::Pose3d pose = model_->GetWorldPose().Ign();
#endif

  const tf::Transform transform(tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()),
                                tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));
  transform_broadcaster_->sendTransform(tf::StampedTransform(transform, current_time, odometry_frame_, model_frame_));

  // publish odom
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odometry_frame_;
  odom.child_frame_id = model_frame_;
  tf::poseTFToMsg(transform, odom.pose.pose);
  odometry_pub_.publish(odom);
}
}
