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

/*
 * Desc: Simple model controller that uses a twist message to move a robot on
 *       the xy plane.
 * Authors: Piyush Khandelwal, Michael Stradner, ALexander Buchegger
 */
#ifndef ARTI_GAZEBO_MOVE_OBJ_SET_MODEL_POSE_PLUGIN_H
#define ARTI_GAZEBO_MOVE_OBJ_SET_MODEL_POSE_PLUGIN_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace arti_gazebo_move_obj
{
class SetModelPosePlugin : public gazebo::ModelPlugin
{

public:
  SetModelPosePlugin();
  ~SetModelPosePlugin();
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

protected:
  virtual void UpdateChild();
  virtual void FiniChild();
  void publishOdometry();
  void QueueThread();
  void processCommand(const geometry_msgs::Pose::ConstPtr& cmd_msg);

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;

  boost::shared_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher odometry_pub_;
  ros::Subscriber command_sub_;
  boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

  boost::mutex lock_;

  std::string robot_namespace_;
  std::string command_topic_;
  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string model_frame_;
  double odometry_rate_;
  ignition::math::Pose3d offset_;

  // Custom Callback Queue
  ros::CallbackQueue callback_queue_;
  boost::thread callback_queue_thread_;

  geometry_msgs::Pose last_command_;
  ros::Time last_command_time_;
  volatile bool alive_;
  gazebo::common::Time last_odom_publish_time_;
};

}

#endif  //ARTI_GAZEBO_MOVE_OBJ_SET_MODEL_POSE_PLUGIN_H
