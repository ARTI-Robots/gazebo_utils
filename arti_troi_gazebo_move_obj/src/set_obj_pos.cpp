/*
 * Copyright 2013 Open Source Robotics Foundation
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
 * Author: Piyush Khandelwal
 * Date: 29 July 2013
 */
#include <arti_troi_gazebo_move_obj/set_obj_pos.h>

namespace gazebo
{

GazeboRosSetObjPos::GazeboRosSetObjPos()
{
  robot_namespace_ = "";
}

GazeboRosSetObjPos::~GazeboRosSetObjPos()
{

}

  // Load the controller
  void GazeboRosSetObjPos::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf)
  {

    parent_ = parent;

    /* Parse parameters */


    if (!sdf->HasElement("robotNamespace"))
    {
      ROS_INFO_NAMED("planar_move", "PlanarMovePlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else
    {
      robot_namespace_ =
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), command_topic_.c_str());
    }
    else
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_topic_.c_str());
    }
    else
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }


    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    }
    else
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }

    //offset_ = ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 0));
    offset_ = parent_->GetWorldPose().Ign();
    if (sdf->HasElement("initPose"))
    {
      offset_ += sdf->GetElement("initPose")->Get<ignition::math::Pose3d>();
      //offset_ = ignition::math::Quaterniond(sdf->GetElement("initPose")->Get<ignition::math::Vector3d>());
    }

#if GAZEBO_MAJOR_VERSION >= 8
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
#endif
#if GAZEBO_MAJOR_VERSION >= 8
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_pose_ = parent_->GetWorldPose().Ign();
#endif
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;
    last_cmd_ = ros::Time::now();


    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED("planar_move", "PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG_NAMED("planar_move", "OCPlugin (%s) has started",
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(command_topic_, 1,
          boost::bind(&GazeboRosSetObjPos::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosSetObjPos::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosSetObjPos::UpdateChild, this));


    //pub_odom_ = rosnode_->advertise<std_msgs::String>("postler_odom1", 10);
    //ros::Timer timer = rosnode_->createTimer(ros::Duration(0.1), &GazeboRosSetObjPos::pubOdomCallback, this);

  }


/*  void GazeboRosSetObjPos::pubOdomCallback(const ros::TimerEvent& e)
  {
    std_msgs::String msg;
    pub_odom_ = rosnode_->advertise<std_msgs::String>("postler_odom1", 10);
    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();
    pub_odom_.publish(msg);
    ros::spinOnce();
    pub_odom_.publish(msg);
  }*/


  // Update the controller
  void GazeboRosSetObjPos::UpdateChild()
  {
/*    boost::mutex::scoped_lock scoped_lock(lock);
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = parent_->WorldPose();
#else
    ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
#endif
    float yaw = pose.Rot().Yaw();
    parent_->SetLinearVel(ignition::math::Vector3d(
          x_ * cosf(yaw) - y_ * sinf(yaw),
          y_ * cosf(yaw) + x_ * sinf(yaw),
          0));
    parent_->SetAngularVel(ignition::math::Vector3d(0, 0, rot_));
    if (odometry_rate_ > 0.0) {
#if GAZEBO_MAJOR_VERSION >= 8
      common::Time current_time = parent_->GetWorld()->SimTime();
#else
      common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
      double seconds_since_last_update =
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }*/

    // Get the desired pose, here giving a random offset

    //if(set_new_pos_ == false)
    //  return;

    boost::mutex::scoped_lock scoped_lock(lock);
    //if(set_new_pos_ == true)
    if(ros::Time::now().toSec() - last_cmd_.toSec() <= 1.0)
    {
      //set_new_pos_ = false;

      ignition::math::Pose3d pose;// = parent_->GetWorldPose().Ign();



      pose =  ignition::math::Pose3d(x_ + offset_.Pos().X(),
                                     y_ + offset_.Pos().Y(),
                                     z_ + offset_.Pos().Y(),
                                     roll_,
                                     pitch_,
                                     yaw_);

      parent_->SetWorldPose(pose);
    }

    if (odometry_rate_ > 0.0)
    {
#if GAZEBO_MAJOR_VERSION >= 8
      common::Time current_time = parent_->GetWorld()->SimTime();
#else
      common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
      double seconds_since_last_update =
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_))
      {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void GazeboRosSetObjPos::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosSetObjPos::cmdVelCallback(
      const geometry_msgs::Pose::ConstPtr& cmd_msg)
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->position.x;
    y_ = cmd_msg->position.y;
    z_ = cmd_msg->position.z;
    tf::Quaternion quat(cmd_msg->orientation.x, cmd_msg->orientation.y, cmd_msg->orientation.z, cmd_msg->orientation.w);
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
    set_new_pos_ = true;
    last_cmd_ = ros::Time::now();
  }

  void GazeboRosSetObjPos::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosSetObjPos::publishOdometry(double step_time)
  {

    ros::Time current_time = ros::Time::now();
    /*std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);*/

    // getting data for base_footprint to odom transform
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->parent_->WorldPose();
#else
    ignition::math::Pose3d pose = this->parent_->GetWorldPose().Ign();
#endif

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3    vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    /*tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame,
            base_footprint_frame));*/

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    /*odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;*/

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
    linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
    if (rot_ > M_PI / step_time)
    {
      // we cannot calculate the angular velocity correctly
      odom_.twist.twist.angular.z = rot_;
    }
    else
    {
      float last_yaw = last_odom_pose_.Rot().Yaw();
      float current_yaw = pose.Rot().Yaw();
      while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
      while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
      float angular_diff = current_yaw - last_yaw;
      odom_.twist.twist.angular.z = angular_diff / step_time;
    }
    last_odom_pose_ = pose;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

    odom_.header.stamp = current_time;
    odom_.header.frame_id = "odom";//odom_frame;
    odom_.child_frame_id = "person";//base_footprint_frame;

    odometry_pub_.publish(odom_);

    tf::Transform transform;
    transform.setOrigin(vt);
    transform.setRotation(qt);
    transform_broadcaster_->sendTransform (
      tf::StampedTransform ( transform, current_time,
                             "ground_truth", "person" ) );




  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosSetObjPos)
}
