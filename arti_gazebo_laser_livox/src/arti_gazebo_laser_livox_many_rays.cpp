
#include <gazebo/gazebo.hh>
#include <tf/tf.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <gazebo_plugins/arti_gazebo_laser_livox.h>
#include <chrono>

namespace gazebo
{
// Register this plugin
GZ_REGISTER_SENSOR_PLUGIN(ArtiGazeboLaserLivox)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
ArtiGazeboLaserLivox::ArtiGazeboLaserLivox()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructor
ArtiGazeboLaserLivox::~ArtiGazeboLaserLivox()
{
  this->laser_queue_.clear();
  this->laser_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load the controller
void ArtiGazeboLaserLivox::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  RayPlugin::Load(_parent, this->sdf);

  this->parent_sensor_ = _parent;

  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);

  this->engine_ = this->world_->GetPhysicsEngine();
  this->engine_->InitForThread();

  this->sdf = _sdf;

  //this->last_update_time_ = common::Time(0);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
  gzthrow("ArtiGazeboLaserLivox controller requires a Ray-Sensor as it's parent");

  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <frameName> is missing, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <topicName> is missing, defaults to /laser/livox40");
    this->topic_name_ = "/laser/livox40";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <updateRate> is missing, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = this->sdf->Get<double>("updateRate");

  if (!this->sdf->HasElement("samples"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <samples> is missing, defaults to 30");
    this->samples_ = 30;
  }
  else
    this->samples_ = this->sdf->Get<int>("samples");

  if (!this->sdf->HasElement("num_ellipses"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <num_ellipses> is missing, defaults to 8");
    this->num_ellipses_ = 1;
  }
  else
    this->num_ellipses_ = this->sdf->Get<int>("num_ellipses");

  if (!this->sdf->HasElement("minRange"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <minRange> is missing, defaults to 0.0");
    this->min_range_ = 0.0;
  }
  else
    this->min_range_ = this->sdf->Get<float>("minRange");

  if (!this->sdf->HasElement("maxRange"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <maxRange> is missing, defaults to 30.0");
    this->max_range_ = 30.0;
  }
  else
    this->max_range_ = this->sdf->Get<float>("maxRange");

  if (!this->sdf->HasElement("rotationIncrement"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <rotationIncrement> is missing, defaults to 0.2 [rad].");
    this->rotation_increment_ = 0.2;
  }
  else
    this->rotation_increment_ = this->sdf->Get<float>("rotationIncrement");

  if (!this->sdf->HasElement("update_rate"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <update_rate> is missing, defaults to 10");
  }
  else
    ROS_ERROR("update_rate found");

  if (!this->sdf->HasElement("interpolation_points"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <interpolation_points> is missing, defaults to 10.");
    this->interpolation_points_ = 10;
  }
  else
    this->interpolation_points_ = this->sdf->Get<unsigned int>("interpolation_points");

  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0 / this->update_rate_;
  else
    this->update_period_ = 0.0;

  this->connect_count_ = 0;

  if (ros::isInitialized())
  {
    this->deferred_load_thread_ = boost::thread(boost::bind(&ArtiGazeboLaserLivox::LoadThread, this));
  }
  else
  {
    gzerr << "ERROR: ROS hasn't been initialized!\n";
  }

  collision_ptr_list_.clear();

  this->parent_ray_sensor_->SetActive(false);
  // Create number of ellipse-8-figures
//  double increment = 180.0 / this->num_ellipses_;
//  for (int i = 0; i < this->num_ellipses_; i++)
//  {
//    this->AddRayEllipseShape(0.0 + i * increment);
//  }

//  double increment = 180.0 / this->num_ellipses_;
//  for (int i = 0; i < 1; i++)
//  {
//    this->AddDemoRayEllipseShape(0.0 + i * increment);
//  }

  this->useable_rays_ = this->rays_.size();
  ROS_INFO("usable rays = %d", this->useable_rays_);

  this->last_update_time_ = common::Time(0);

  this->parent_ray_sensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load Thread
void ArtiGazeboLaserLivox::LoadThread()
{
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao;
    ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(this->topic_name_, 1,
                                                                 boost::bind(&ArtiGazeboLaserLivox::Connect, this),
                                                                 boost::bind(&ArtiGazeboLaserLivox::Disconnect, this),
                                                                 ros::VoidPtr(), &this->laser_queue_);

    this->pub_ = this->rosnode_->advertise(ao);
  }

  this->parent_ray_sensor_->SetActive(false);

  this->callback_queue_thread_ =
    boost::thread(boost::bind(&ArtiGazeboLaserLivox::LaserQueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Increment counter
void ArtiGazeboLaserLivox::Connect()
{
  this->connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Decrement counter
void ArtiGazeboLaserLivox::Disconnect()
{
  this->connect_count_--;

  if (this->connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create new Laserscan
void ArtiGazeboLaserLivox::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time cur_time = this->world_->SimTime();
#else
    common::Time cur_time = this->world_->GetSimTime();
#endif
    if (cur_time < this->last_update_time_)
    {
      ROS_WARN_NAMED("livox", "WARNING: Current time was negative (smaller than last-update-time).");
      this->last_update_time_ = cur_time;
    }

    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      common::Time sensor_update_time = this->parent_sensor_->LastUpdateTime();
      this->PutLaserData(sensor_update_time);
      this->last_update_time_ = cur_time;
    }
  }
  else
  {
    ROS_ERROR_NAMED("livox", "ERROR: Topic for Livox-lidar not set!");
  }
}


bool ArtiGazeboLaserLivox::createCollisions()
{
  int num_loops = 20;
  if(debug_counter < this->samples_ * this->num_ellipses_ / num_loops)
  {
    this->parent_ray_sensor_->SetActive(false);
    std::string parent_name = this->parent_ray_sensor_->ParentName();
    for(int l=0; l < num_loops; l++)
    {
      physics::CollisionPtr collision_ptr_1 = this->engine_->CreateCollision("ray", parent_name);
      collision_ptr_1->SetName("own_ray_sensor_collision1");
      collision_ptr_1->SetRelativePose(this->sensor_pose_);
      collision_ptr_1->SetInitialRelativePose(this->sensor_pose_);
      collision_ptr_list_.push_back(collision_ptr_1);

//      gazebo::physics::RayShapePtr ray1 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//        collision_ptr_1->GetShape());

      physics::CollisionPtr collision_ptr_2 = this->engine_->CreateCollision("ray", parent_name);
      collision_ptr_2->SetName("own_ray_sensor_collision2");
      collision_ptr_2->SetRelativePose(this->sensor_pose_);
      collision_ptr_2->SetInitialRelativePose(this->sensor_pose_);
      collision_ptr_list_.push_back(collision_ptr_2);

//      gazebo::physics::RayShapePtr ray2 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//        collision_ptr_2->GetShape());

      physics::CollisionPtr collision_ptr_3 = this->engine_->CreateCollision("ray", parent_name);
      collision_ptr_3->SetName("own_ray_sensor_collision3");
      collision_ptr_3->SetRelativePose(this->sensor_pose_);
      collision_ptr_3->SetInitialRelativePose(this->sensor_pose_);
      collision_ptr_list_.push_back(collision_ptr_3);

//      gazebo::physics::RayShapePtr ray3 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//        collision_ptr_3->GetShape());

      physics::CollisionPtr collision_ptr_4 = this->engine_->CreateCollision("ray", parent_name);
      collision_ptr_4->SetName("own_ray_sensor_collision4");
      collision_ptr_4->SetRelativePose(this->sensor_pose_);
      collision_ptr_4->SetInitialRelativePose(this->sensor_pose_);
      collision_ptr_list_.push_back(collision_ptr_4);

//      gazebo::physics::RayShapePtr ray4 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//        collision_ptr_4->GetShape());
//
//      this->rays_.push_back(ray1);
//      this->rays_.push_back(ray2);
//      this->rays_.push_back(ray3);
//      this->rays_.push_back(ray4);
    }
    this->parent_ray_sensor_->SetActive(true);
    debug_counter++;
    ROS_ERROR("collision_ptr_list_.size = %ld (counter = %d)", collision_ptr_list_.size(), debug_counter);
    return false;
  }
  return true;
}
bool ArtiGazeboLaserLivox::setupRayPattern()
{
//  if(createCollisions())
//  {
//    if(create_pattern_counter_ == 0)
//    {
//      this->parent_ray_sensor_->SetActive(false);
//      // Create number of ellipse-8-figures
//      double increment = 180.0 / this->num_ellipses_;
//      for (int i = 0; i < this->num_ellipses_; i++)
//      {
//        //this->AddRayEllipseShape(0.0 + i * increment);
//      }
//
//      this->last_update_time_ = common::Time(0);
//
//      this->parent_ray_sensor_->SetActive(true);
//      create_pattern_counter_++;
//      return true;
//    }
//    return false;
//  }
//  else
//  {
//    return false;
//  }

    if(create_pattern_counter_ < this->num_ellipses_)
    {
      this->parent_ray_sensor_->SetActive(false);
      // Create number of ellipse-8-figures
      double increment = 180.0 / this->num_ellipses_;

      this->AddRayEllipseShape(0.0 + create_pattern_counter_ * increment);

      //this->last_update_time_ = common::Time(0);

      this->parent_ray_sensor_->SetActive(true);
      create_pattern_counter_++;
      return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Add points from raytracing into Pointcloud
void ArtiGazeboLaserLivox::PutLaserData(common::Time &_updateTime)
{

  if(!createCollisions())
  {
    return;
  }

  if(setupRayPattern())
  {
    return;
  }
  auto tstart = std::chrono::steady_clock::now();
  //if(!setupRayPattern())
  //  return;

  //ROS_INFO("I am here");
//  if(debug_counter == 0)
//  {
//    double increment = 180.0 / this->num_ellipses_;
//    this->AddRayEllipseShape(0.0 + 0 * increment);
//    debug_counter++;
//    return;
//  }
//  else if(debug_counter == 1000)
//  {
//    double increment = 180.0 / this->num_ellipses_;
//    this->AddRayEllipseShape(0.0 + 1 * increment);
//    debug_counter++;
//    return;
//  }
//  else if(debug_counter == 2000)
//  {
//    double increment = 180.0 / this->num_ellipses_;
//    this->AddRayEllipseShape(0.0 + 2 * increment);
//    debug_counter++;
//    return;
//  }
//  else if(debug_counter == 3000)
//  {
//    double increment = 180.0 / this->num_ellipses_;
//    this->AddRayEllipseShape(0.0 + 3 * increment);
//    debug_counter++;
//    return;
//  }
//  else if(debug_counter == 4000)
//  {
//    double increment = 180.0 / this->num_ellipses_;
//    this->AddRayEllipseShape(0.0 + 4 * increment);
//    debug_counter++;
//    return;
//  }
//  else if(debug_counter == 5000)
//  {
//    double increment = 180.0 / this->num_ellipses_;
//    this->AddRayEllipseShape(0.0 + 5 * increment);
//    debug_counter++;
//    return;
//  }
//  else if(debug_counter == 6000)
//  {
//    double increment = 180.0 / this->num_ellipses_;
//    this->AddRayEllipseShape(0.0 + 6 * increment);
//    debug_counter++;
//    return;
//  }
//  else if(debug_counter == 7000)
//  {
//    double increment = 180.0 / this->num_ellipses_;
//    this->AddRayEllipseShape(0.0 + 7 * increment);
//    debug_counter++;
//    //debug_counter = -1;
//    return;
//  }
//
//
//  if(debug_counter < 8000)
//    debug_counter++;


  this->parent_ray_sensor_->SetActive(false);

  double dist;
  std::string entityName;

  this->cloud_msg_.points.clear();
  this->cloud_msg_.channels.clear();
  //this->cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

  this->cloud_msg_.header.frame_id = this->frame_name_;
  this->cloud_msg_.header.stamp.sec = _updateTime.sec;
  this->cloud_msg_.header.stamp.nsec = _updateTime.nsec;

  geometry_msgs::Point32 point;
  geometry_msgs::Point32 rot_point;
  sensor_msgs::ChannelFloat32 intensity = sensor_msgs::ChannelFloat32();
  intensity.values.clear();
  intensity.name = "intensity";

  math::Quaternion ray;
  math::Vector3 axis;
  math::Pose offset;
  math::Vector3 endpoint;
  offset = this->collision_ptr_->GetRelativePose();

  math::Vector3 offset_rot = offset.rot.GetAsEuler();

  math::Quaternion rot_only;
  if (this->current_rot_angle_ + this->rotation_increment_ >= M_PI)
    rot_only.SetFromEuler(math::Vector3(this->current_rot_angle_ + this->rotation_increment_ - M_PI + offset_rot.x,
                                        offset_rot.y, offset_rot.z));
  else
    rot_only.SetFromEuler(math::Vector3(this->current_rot_angle_ + this->rotation_increment_ + offset_rot.x,
                                        offset_rot.y, offset_rot.z));
  offset.rot = rot_only;

  //ROS_INFO("rays-size: %d (usable: %d)",this->rays_.size(), this->useable_rays_);
  for (int i = 0; i < this->rays_.size(); i++)
  //for (int i = 0; i < this->useable_rays_; i++)
  {
    // Get distance of collision for the current ray
    this->rays_[i]->GetIntersection(dist, entityName);
    // If the distance is below 1000.0, than a collision with an object happened
    if (dist < 999.9)
    {
      // Calculate point from ellipse-shaped-rays without considering the rotation-angle
      point.x = cos(this->vertical_ray_angles_[i]) * cos(this->horizontal_ray_angles_[i]) * dist;
      point.y = sin(this->horizontal_ray_angles_[i]) * cos(this->vertical_ray_angles_[i]) * dist;
      point.z = sin(this->vertical_ray_angles_[i]) * dist;
      // Use constant intensity-value
      intensity.values.push_back(1);
      // Rotate the calculated point from the pointcloud with respect to the current rotation-angle.
      rot_point.x = point.x;
      rot_point.y = point.y * cos(this->current_rot_angle_) - point.z * sin(this->current_rot_angle_);
      rot_point.z = point.y * sin(this->current_rot_angle_) + point.z * cos(this->current_rot_angle_);
      // Add point to pointcloud-list
      this->cloud_msg_.points.push_back(rot_point);
      //ROS_ERROR("point: x = %f | y = %f | z = %f", rot_point.x, rot_point.y, rot_point.z);

//      if (cloud_msg_.points.size() > 1)
//      {
//        geometry_msgs::Point32 int_point_1 = *(cloud_msg_.points.begin() + cloud_msg_.points.size() - 1);
//        geometry_msgs::Point32 int_point_2 = *(cloud_msg_.points.begin() + cloud_msg_.points.size() - 2);
//
//        //interpolate additionl points to avoid raytracing overhead
//        for (size_t j = 0; j < interpolation_points_; ++j)
//        {
//          double scaling = static_cast<double>(j) / static_cast<double>(interpolation_points_);
//          geometry_msgs::Point32 interpolation;
//          interpolation.x = int_point_1.x * scaling + (1. - scaling) * int_point_2.x;
//          interpolation.y = int_point_1.y * scaling + (1. - scaling) * int_point_2.y;
//          interpolation.z = int_point_1.z * scaling + (1. - scaling) * int_point_2.z;
//
//          intensity.values.push_back(1);
//          this->cloud_msg_.points.push_back(interpolation);
//          //ROS_ERROR("interp: x = %f | y = %f | z = %f", interpolation.x, interpolation.y, interpolation.z);
//        }
//      }
    }

    ray.SetFromEuler(math::Vector3(0.0, -this->vertical_ray_angles_[i], this->horizontal_ray_angles_[i]));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    endpoint = (axis * this->max_range_) + offset.pos;

    this->rays_[i]->SetPoints(this->ray_startpoint_, endpoint);
    this->rays_[i]->Update();
  }
  // Add intensity-field to the pointcloud
  this->cloud_msg_.channels.push_back(intensity);

  // Increment the current rotation angle
  this->current_rot_angle_ += this->rotation_increment_;
  // Don't exceed pi
  if (this->current_rot_angle_ >= M_PI)
    this->current_rot_angle_ -= M_PI;

  // Transform PointCloud to PointCloud2
  sensor_msgs::convertPointCloudToPointCloud2(this->cloud_msg_, this->pc2_msgs_);

  // The first message published is always invalid, therefore don't publish it
  if(this->init_finished_ == false)
  {
    this->init_finished_ = true;
  }
  else
  {
    // Publish ROS-pointcloud
    this->pub_.publish(this->pc2_msgs_);
  }
  this->parent_ray_sensor_->SetActive(true);

  auto tend = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count();
  ROS_INFO_STREAM_THROTTLE(10, "raytracing took " << tend << "ms." );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ArtiGazeboLaserLivox::LaserQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->laser_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create an eight-figure suing two ellipses as a ray-pattern for raytracing
bool ArtiGazeboLaserLivox::AddRayEllipseShape(double rotation_degrees)
{
  this->parent_ray_sensor_->SetActive(false);
  auto tstart = std::chrono::steady_clock::now();

  double samples_a = this->samples_;
  double ell_a = 0.3492; // length of ellipse parameter a in a distance of 1 m with a FOV of 38.4°
  double ell_b = 0.07275; // length of ellipse parameter b in a distance of 1 m with a FOV of 38.4°
  double dx = 2.0 * ell_a / samples_a;
  math::Quaternion ray;
  math::Vector3 axis;
  math::Pose offset;
  math::Vector3 start, end1, end2, end3, end4;

  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;

  std::string parent_name = this->parent_ray_sensor_->ParentName();
  this->collision_ptr_ = this->engine_->CreateCollision("ray", parent_name);
  this->collision_ptr_->SetName("own_ray_sensor_collision");
  this->sensor_pose_ = this->parent_ray_sensor_->Pose();
  this->collision_ptr_->SetRelativePose(this->sensor_pose_);
  this->collision_ptr_->SetInitialRelativePose(this->sensor_pose_);

  offset = this->collision_ptr_->GetRelativePose();
  math::Vector3 offset_rot = offset.rot.GetAsEuler();
  ray.SetFromEuler(math::Vector3(offset_rot.x, offset_rot.y, offset_rot.z));
  axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);

  // Get the position of the pose of the parent_ray_sensor and add its position to the new ray (its relative to the parent-frame)
  start = (axis * this->min_range_) + offset.pos;
  this->ray_startpoint_ = start;



  std::cout << "Time 1: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;

  for (int i = 0; i < samples_a; i++)
  {
    // use x as indexed variable that moves along the ellipse
    double val_x = ((double) i) * dx - ell_a;
    // calculate the two y-coordinates from the x-coordinate of the ellipse
    double val_y_1 = sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
    double val_y_2 = -sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));

    // The point (0,0) is in the middle of the ellipse, therefore we move the ellipse to one side by the amount of ell_a
    double val_x_1 = val_x + ell_a;
    // And a second time to the other side. Now we have four ellipse value pairs
    double val_x_2 = val_x - ell_a;

    // The four ellipse points are:
    // (val_x_1, val_y_1)
    // (val_x_1, val_y_2)
    // (val_x_2, val_y_1)
    // (val_x_2, val_y_2)

    double rot_rad = rotation_degrees / 180.0 * M_PI;
    double rot_mat[2][2] = {{cos(rot_rad), -sin(rot_rad)}, {sin(rot_rad), cos(rot_rad)}};

    double ell_x1 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_1;
    double ell_y1 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_1;

    double ell_x2 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_2;
    double ell_y2 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_2;

    double ell_x3 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_1;
    double ell_y3 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_1;

    double ell_x4 = 0.0;
    double ell_y4 = 0.0;
    // for the first sample there are 4 rays and 2 of them are equal. Modify one to get one ray in the middle
    // and 2 on the outer side of the ellipse.
    if (i == 0)
    {
      ell_x4 = rot_mat[0][0] * -val_x_2 + rot_mat[0][1] * val_y_2;
      ell_y4 = rot_mat[1][0] * -val_x_2 + rot_mat[1][1] * val_y_2;
    }
    else
    {
      ell_x4 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_2;
      ell_y4 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_2;
    }

    // Note: The current ellipse has the following axis: y is up and x is in the middle and pointing to the right.
    //       Therefore we need to map the y-axis of the ellipse to the Ray-z-axis and the x-axis of the ellipse to the
    //       -Ray-y-axis.

    end1.x = (1.0 + offset.pos.x);
    end1.y = (-ell_x1 + offset.pos.y);
    end1.z = (ell_y1 + offset.pos.z);

    end2.x = (1.0 + offset.pos.x);
    end2.y = (-ell_x2 + offset.pos.y);
    end2.z = (ell_y2 + offset.pos.z);

    end3.x = (1.0 + offset.pos.x);
    end3.y = (-ell_x3 + offset.pos.y);
    end3.z = (ell_y3 + offset.pos.z);

    end4.x = (1.0 + offset.pos.x);
    end4.y = (-ell_x4 + offset.pos.y);
    end4.z = (ell_y4 + offset.pos.z);

    double beta1 = (atan2(end1.y - start.y, end1.x - start.x));
    double beta2 = (atan2(end2.y - start.y, end2.x - start.x));
    double beta3 = (atan2(end3.y - start.y, end3.x - start.x));
    double beta4 = (atan2(end4.y - start.y, end4.x - start.x));
    double alpha1 = (atan2(end1.z - start.z, end1.x - start.x));
    double alpha2 = (atan2(end2.z - start.z, end2.x - start.x));
    double alpha3 = (atan2(end3.z - start.z, end3.x - start.x));
    double alpha4 = (atan2(end4.z - start.z, end4.x - start.x));

    end1.x = (1.0 + offset.pos.x) * this->max_range_;
    end1.y = (-ell_x1 + offset.pos.y) * this->max_range_;
    end1.z = (ell_y1 + offset.pos.z) * this->max_range_;
    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha1 + offset_rot.y, beta1 + offset_rot.z));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    end1 = (axis * this->max_range_) + offset.pos;

    end2.x = (1.0 + offset.pos.x) * this->max_range_;
    end2.y = (-ell_x2 + offset.pos.y) * this->max_range_;
    end2.z = (ell_y2 + offset.pos.z) * this->max_range_;
    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha2 + offset_rot.y, beta2 + offset_rot.z));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    end2 = (axis * this->max_range_) + offset.pos;

    end3.x = (1.0 + offset.pos.x) * this->max_range_;
    end3.y = (-ell_x3 + offset.pos.y) * this->max_range_;
    end3.z = (ell_y3 + offset.pos.z) * this->max_range_;
    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha3 + offset_rot.y, beta3 + offset_rot.z));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    end3 = (axis * this->max_range_) + offset.pos;

    end4.x = (1.0 + offset.pos.x) * this->max_range_;
    end4.y = (-ell_x4 + offset.pos.y) * this->max_range_;
    end4.z = (ell_y4 + offset.pos.z) * this->max_range_;
    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha4 + offset_rot.y, beta4 + offset_rot.z));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    end4 = (axis * this->max_range_) + offset.pos;

    std::string parent_name = this->parent_ray_sensor_->ParentName();

    //this->parent_ray_sensor_->LaserShape()->shared_from_this();

//    physics::CollisionPtr collision_ptr_1 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_1->SetName("own_ray_sensor_collision1");
//    collision_ptr_1->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_1);
//
//    gazebo::physics::RayShapePtr ray1 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//      collision_ptr_1->GetShape());
//
//    physics::CollisionPtr collision_ptr_2 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_2->SetName("own_ray_sensor_collision2");
//    collision_ptr_2->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_2);
//
//    gazebo::physics::RayShapePtr ray2 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//      collision_ptr_2->GetShape());
//
//    physics::CollisionPtr collision_ptr_3 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_3->SetName("own_ray_sensor_collision3");
//    collision_ptr_3->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_3);
//
//    gazebo::physics::RayShapePtr ray3 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//      collision_ptr_3->GetShape());
//
//    physics::CollisionPtr collision_ptr_4 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_4->SetName("own_ray_sensor_collision4");
//    collision_ptr_4->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_4);
//
//    gazebo::physics::RayShapePtr ray4 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//      collision_ptr_4->GetShape());
//
//    ray1->SetPoints(start, end1);
//    ray2->SetPoints(start, end2);
//    ray3->SetPoints(start, end3);
//    ray4->SetPoints(start, end4);
//
//    this->rays_.push_back(ray1);
//    this->rays_.push_back(ray2);
//    this->rays_.push_back(ray3);
//    this->rays_.push_back(ray4);

//    this->rays_[used_ray_count_++]->SetPoints(start, end1);
//    this->rays_[used_ray_count_++]->SetPoints(start, end2);
//    this->rays_[used_ray_count_++]->SetPoints(start, end3);
//    this->rays_[used_ray_count_++]->SetPoints(start, end4);

    gazebo::physics::RayShapePtr ray1 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      collision_ptr_list_[used_ray_count_++]->GetShape());
    gazebo::physics::RayShapePtr ray2 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      collision_ptr_list_[used_ray_count_++]->GetShape());
    gazebo::physics::RayShapePtr ray3 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      collision_ptr_list_[used_ray_count_++]->GetShape());
    gazebo::physics::RayShapePtr ray4 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      collision_ptr_list_[used_ray_count_++]->GetShape());

    ray1->SetPoints(start, end1);
    ray2->SetPoints(start, end2);
    ray3->SetPoints(start, end3);
    ray4->SetPoints(start, end4);

    this->rays_.push_back(ray1);
    this->rays_.push_back(ray2);
    this->rays_.push_back(ray3);
    this->rays_.push_back(ray4);

    // Push back the angles that are not transformed due to the pose because we create the pointcloud from the local frame.
    // The psoe-orientation only plays a part in setting the ray-positions.
    this->horizontal_ray_angles_.push_back(float(beta1));
    this->horizontal_ray_angles_.push_back(float(beta2));
    this->horizontal_ray_angles_.push_back(float(beta3));
    this->horizontal_ray_angles_.push_back(float(beta4));
    this->vertical_ray_angles_.push_back(float(alpha1));
    this->vertical_ray_angles_.push_back(float(alpha2));
    this->vertical_ray_angles_.push_back(float(alpha3));
    this->vertical_ray_angles_.push_back(float(alpha4));
  }

  //--------------------------------------------------------------------------------------------------------------------
//  for (int i = 0; i < samples_a; i++)
//  {
//    // use x as indexed variable that moves along the ellipse
//    double val_x = ((double) i) * dx - ell_a;
//    // calculate the two y-coordinates from the x-coordinate of the ellipse
//    double val_y_1 = sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
//    double val_y_2 = -sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
//
//    // The point (0,0) is in the middle of the ellipse, therefore we move the ellipse to one side by the amount of ell_a
//    double val_x_1 = val_x + ell_a;
//    // And a second time to the other side. Now we have four ellipse value pairs
//    double val_x_2 = val_x - ell_a;
//
//    // The four ellipse points are:
//    // (val_x_1, val_y_1)
//    // (val_x_1, val_y_2)
//    // (val_x_2, val_y_1)
//    // (val_x_2, val_y_2)
//
//    double rot_rad = rotation_degrees / 180.0 * M_PI;
//    double rot_mat[2][2] = {{cos(rot_rad), -sin(rot_rad)}, {sin(rot_rad), cos(rot_rad)}};
//
//    double ell_x1 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_1;
//    double ell_y1 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_1;
//
//    double ell_x2 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_2;
//    double ell_y2 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_2;
//
//    double ell_x3 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_1;
//    double ell_y3 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_1;
//
//    double ell_x4 = 0.0;
//    double ell_y4 = 0.0;
//    // for the first sample there are 4 rays and 2 of them are equal. Modify one to get one ray in the middle
//    // and 2 on the outer side of the ellipse.
//    if (i == 0)
//    {
//      ell_x4 = rot_mat[0][0] * -val_x_2 + rot_mat[0][1] * val_y_2;
//      ell_y4 = rot_mat[1][0] * -val_x_2 + rot_mat[1][1] * val_y_2;
//    }
//    else
//    {
//      ell_x4 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_2;
//      ell_y4 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_2;
//    }
//
//    // Note: The current ellipse has the following axis: y is up and x is in the middle and pointing to the right.
//    //       Therefore we need to map the y-axis of the ellipse to the Ray-z-axis and the x-axis of the ellipse to the
//    //       -Ray-y-axis.
//
//    end1.x = (1.0 + offset.pos.x);
//    end1.y = (-ell_x1 + offset.pos.y);
//    end1.z = (ell_y1 + offset.pos.z);
//
//    end2.x = (1.0 + offset.pos.x);
//    end2.y = (-ell_x2 + offset.pos.y);
//    end2.z = (ell_y2 + offset.pos.z);
//
//    end3.x = (1.0 + offset.pos.x);
//    end3.y = (-ell_x3 + offset.pos.y);
//    end3.z = (ell_y3 + offset.pos.z);
//
//    end4.x = (1.0 + offset.pos.x);
//    end4.y = (-ell_x4 + offset.pos.y);
//    end4.z = (ell_y4 + offset.pos.z);
//
//  }
//  std::cout << "Time 2: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;
//
//  for (int i = 0; i < samples_a; i++)
//  {
//    // use x as indexed variable that moves along the ellipse
//    double val_x = ((double) i) * dx - ell_a;
//    // calculate the two y-coordinates from the x-coordinate of the ellipse
//    double val_y_1 = sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
//    double val_y_2 = -sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
//
//    // The point (0,0) is in the middle of the ellipse, therefore we move the ellipse to one side by the amount of ell_a
//    double val_x_1 = val_x + ell_a;
//    // And a second time to the other side. Now we have four ellipse value pairs
//    double val_x_2 = val_x - ell_a;
//
//    // The four ellipse points are:
//    // (val_x_1, val_y_1)
//    // (val_x_1, val_y_2)
//    // (val_x_2, val_y_1)
//    // (val_x_2, val_y_2)
//
//    double rot_rad = rotation_degrees / 180.0 * M_PI;
//    double rot_mat[2][2] = {{cos(rot_rad), -sin(rot_rad)}, {sin(rot_rad), cos(rot_rad)}};
//
//    double ell_x1 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_1;
//    double ell_y1 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_1;
//
//    double ell_x2 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_2;
//    double ell_y2 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_2;
//
//    double ell_x3 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_1;
//    double ell_y3 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_1;
//
//    double ell_x4 = 0.0;
//    double ell_y4 = 0.0;
//    // for the first sample there are 4 rays and 2 of them are equal. Modify one to get one ray in the middle
//    // and 2 on the outer side of the ellipse.
//    if (i == 0)
//    {
//      ell_x4 = rot_mat[0][0] * -val_x_2 + rot_mat[0][1] * val_y_2;
//      ell_y4 = rot_mat[1][0] * -val_x_2 + rot_mat[1][1] * val_y_2;
//    }
//    else
//    {
//      ell_x4 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_2;
//      ell_y4 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_2;
//    }
//
//    // Note: The current ellipse has the following axis: y is up and x is in the middle and pointing to the right.
//    //       Therefore we need to map the y-axis of the ellipse to the Ray-z-axis and the x-axis of the ellipse to the
//    //       -Ray-y-axis.
//
//    end1.x = (1.0 + offset.pos.x);
//    end1.y = (-ell_x1 + offset.pos.y);
//    end1.z = (ell_y1 + offset.pos.z);
//
//    end2.x = (1.0 + offset.pos.x);
//    end2.y = (-ell_x2 + offset.pos.y);
//    end2.z = (ell_y2 + offset.pos.z);
//
//    end3.x = (1.0 + offset.pos.x);
//    end3.y = (-ell_x3 + offset.pos.y);
//    end3.z = (ell_y3 + offset.pos.z);
//
//    end4.x = (1.0 + offset.pos.x);
//    end4.y = (-ell_x4 + offset.pos.y);
//    end4.z = (ell_y4 + offset.pos.z);
//    double beta1 = (atan2(end1.y - start.y, end1.x - start.x));
//    double beta2 = (atan2(end2.y - start.y, end2.x - start.x));
//    double beta3 = (atan2(end3.y - start.y, end3.x - start.x));
//    double beta4 = (atan2(end4.y - start.y, end4.x - start.x));
//    double alpha1 = (atan2(end1.z - start.z, end1.x - start.x));
//    double alpha2 = (atan2(end2.z - start.z, end2.x - start.x));
//    double alpha3 = (atan2(end3.z - start.z, end3.x - start.x));
//    double alpha4 = (atan2(end4.z - start.z, end4.x - start.x));
//
//    end1.x = (1.0 + offset.pos.x) * this->max_range_;
//    end1.y = (-ell_x1 + offset.pos.y) * this->max_range_;
//    end1.z = (ell_y1 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha1 + offset_rot.y, beta1 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end1 = (axis * this->max_range_) + offset.pos;
//
//    end2.x = (1.0 + offset.pos.x) * this->max_range_;
//    end2.y = (-ell_x2 + offset.pos.y) * this->max_range_;
//    end2.z = (ell_y2 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha2 + offset_rot.y, beta2 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end2 = (axis * this->max_range_) + offset.pos;
//
//    end3.x = (1.0 + offset.pos.x) * this->max_range_;
//    end3.y = (-ell_x3 + offset.pos.y) * this->max_range_;
//    end3.z = (ell_y3 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha3 + offset_rot.y, beta3 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end3 = (axis * this->max_range_) + offset.pos;
//
//    end4.x = (1.0 + offset.pos.x) * this->max_range_;
//    end4.y = (-ell_x4 + offset.pos.y) * this->max_range_;
//    end4.z = (ell_y4 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha4 + offset_rot.y, beta4 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end4 = (axis * this->max_range_) + offset.pos;
//
//  }
//
//
//  std::cout << "Time 3: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;
//
//  for (int i = 0; i < samples_a; i++)
//  {
//    // use x as indexed variable that moves along the ellipse
//    double val_x = ((double) i) * dx - ell_a;
//    // calculate the two y-coordinates from the x-coordinate of the ellipse
//    double val_y_1 = sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
//    double val_y_2 = -sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
//
//    // The point (0,0) is in the middle of the ellipse, therefore we move the ellipse to one side by the amount of ell_a
//    double val_x_1 = val_x + ell_a;
//    // And a second time to the other side. Now we have four ellipse value pairs
//    double val_x_2 = val_x - ell_a;
//
//    // The four ellipse points are:
//    // (val_x_1, val_y_1)
//    // (val_x_1, val_y_2)
//    // (val_x_2, val_y_1)
//    // (val_x_2, val_y_2)
//
//    double rot_rad = rotation_degrees / 180.0 * M_PI;
//    double rot_mat[2][2] = {{cos(rot_rad), -sin(rot_rad)}, {sin(rot_rad), cos(rot_rad)}};
//
//    double ell_x1 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_1;
//    double ell_y1 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_1;
//
//    double ell_x2 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_2;
//    double ell_y2 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_2;
//
//    double ell_x3 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_1;
//    double ell_y3 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_1;
//
//    double ell_x4 = 0.0;
//    double ell_y4 = 0.0;
//    // for the first sample there are 4 rays and 2 of them are equal. Modify one to get one ray in the middle
//    // and 2 on the outer side of the ellipse.
//    if (i == 0)
//    {
//      ell_x4 = rot_mat[0][0] * -val_x_2 + rot_mat[0][1] * val_y_2;
//      ell_y4 = rot_mat[1][0] * -val_x_2 + rot_mat[1][1] * val_y_2;
//    }
//    else
//    {
//      ell_x4 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_2;
//      ell_y4 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_2;
//    }
//
//    // Note: The current ellipse has the following axis: y is up and x is in the middle and pointing to the right.
//    //       Therefore we need to map the y-axis of the ellipse to the Ray-z-axis and the x-axis of the ellipse to the
//    //       -Ray-y-axis.
//
//    end1.x = (1.0 + offset.pos.x);
//    end1.y = (-ell_x1 + offset.pos.y);
//    end1.z = (ell_y1 + offset.pos.z);
//
//    end2.x = (1.0 + offset.pos.x);
//    end2.y = (-ell_x2 + offset.pos.y);
//    end2.z = (ell_y2 + offset.pos.z);
//
//    end3.x = (1.0 + offset.pos.x);
//    end3.y = (-ell_x3 + offset.pos.y);
//    end3.z = (ell_y3 + offset.pos.z);
//
//    end4.x = (1.0 + offset.pos.x);
//    end4.y = (-ell_x4 + offset.pos.y);
//    end4.z = (ell_y4 + offset.pos.z);
//    double beta1 = (atan2(end1.y - start.y, end1.x - start.x));
//    double beta2 = (atan2(end2.y - start.y, end2.x - start.x));
//    double beta3 = (atan2(end3.y - start.y, end3.x - start.x));
//    double beta4 = (atan2(end4.y - start.y, end4.x - start.x));
//    double alpha1 = (atan2(end1.z - start.z, end1.x - start.x));
//    double alpha2 = (atan2(end2.z - start.z, end2.x - start.x));
//    double alpha3 = (atan2(end3.z - start.z, end3.x - start.x));
//    double alpha4 = (atan2(end4.z - start.z, end4.x - start.x));
//
//    end1.x = (1.0 + offset.pos.x) * this->max_range_;
//    end1.y = (-ell_x1 + offset.pos.y) * this->max_range_;
//    end1.z = (ell_y1 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha1 + offset_rot.y, beta1 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end1 = (axis * this->max_range_) + offset.pos;
//
//    end2.x = (1.0 + offset.pos.x) * this->max_range_;
//    end2.y = (-ell_x2 + offset.pos.y) * this->max_range_;
//    end2.z = (ell_y2 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha2 + offset_rot.y, beta2 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end2 = (axis * this->max_range_) + offset.pos;
//
//    end3.x = (1.0 + offset.pos.x) * this->max_range_;
//    end3.y = (-ell_x3 + offset.pos.y) * this->max_range_;
//    end3.z = (ell_y3 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha3 + offset_rot.y, beta3 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end3 = (axis * this->max_range_) + offset.pos;
//
//    end4.x = (1.0 + offset.pos.x) * this->max_range_;
//    end4.y = (-ell_x4 + offset.pos.y) * this->max_range_;
//    end4.z = (ell_y4 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha4 + offset_rot.y, beta4 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end4 = (axis * this->max_range_) + offset.pos;
//    std::string parent_name = this->parent_ray_sensor_->ParentName();
//
//    physics::CollisionPtr collision_ptr_1 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_1->SetName("own_ray_sensor_collision1");
//    collision_ptr_1->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_1);
//
//    //gazebo::physics::RayShapePtr ray1 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//    //  collision_ptr_1->GetShape());
//
//    physics::CollisionPtr collision_ptr_2 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_2->SetName("own_ray_sensor_collision2");
//    collision_ptr_2->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_2);
//
//    //gazebo::physics::RayShapePtr ray2 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//    //  collision_ptr_2->GetShape());
//
//    physics::CollisionPtr collision_ptr_3 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_3->SetName("own_ray_sensor_collision3");
//    collision_ptr_3->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_3);
//
//    //gazebo::physics::RayShapePtr ray3 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//    //  collision_ptr_3->GetShape());
//
//    physics::CollisionPtr collision_ptr_4 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_4->SetName("own_ray_sensor_collision4");
//    collision_ptr_4->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_4);
//
//    //gazebo::physics::RayShapePtr ray4 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//    //  collision_ptr_4->GetShape());
//
//    //ray1->SetPoints(start, end1);
//    //ray2->SetPoints(start, end2);
//    //ray3->SetPoints(start, end3);
//    //ray4->SetPoints(start, end4);
//
//    //Update: it seems the creation of the collision-ptr takes a lot of time.
//  }
//
//  std::cout << "Time 4: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;
//
//  for (int i = 0; i < samples_a; i++)
//  {
//    // use x as indexed variable that moves along the ellipse
//    double val_x = ((double) i) * dx - ell_a;
//    // calculate the two y-coordinates from the x-coordinate of the ellipse
//    double val_y_1 = sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
//    double val_y_2 = -sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
//
//    // The point (0,0) is in the middle of the ellipse, therefore we move the ellipse to one side by the amount of ell_a
//    double val_x_1 = val_x + ell_a;
//    // And a second time to the other side. Now we have four ellipse value pairs
//    double val_x_2 = val_x - ell_a;
//
//    // The four ellipse points are:
//    // (val_x_1, val_y_1)
//    // (val_x_1, val_y_2)
//    // (val_x_2, val_y_1)
//    // (val_x_2, val_y_2)
//
//    double rot_rad = rotation_degrees / 180.0 * M_PI;
//    double rot_mat[2][2] = {{cos(rot_rad), -sin(rot_rad)}, {sin(rot_rad), cos(rot_rad)}};
//
//    double ell_x1 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_1;
//    double ell_y1 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_1;
//
//    double ell_x2 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_2;
//    double ell_y2 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_2;
//
//    double ell_x3 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_1;
//    double ell_y3 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_1;
//
//    double ell_x4 = 0.0;
//    double ell_y4 = 0.0;
//    // for the first sample there are 4 rays and 2 of them are equal. Modify one to get one ray in the middle
//    // and 2 on the outer side of the ellipse.
//    if (i == 0)
//    {
//      ell_x4 = rot_mat[0][0] * -val_x_2 + rot_mat[0][1] * val_y_2;
//      ell_y4 = rot_mat[1][0] * -val_x_2 + rot_mat[1][1] * val_y_2;
//    }
//    else
//    {
//      ell_x4 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_2;
//      ell_y4 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_2;
//    }
//
//    // Note: The current ellipse has the following axis: y is up and x is in the middle and pointing to the right.
//    //       Therefore we need to map the y-axis of the ellipse to the Ray-z-axis and the x-axis of the ellipse to the
//    //       -Ray-y-axis.
//
//    end1.x = (1.0 + offset.pos.x);
//    end1.y = (-ell_x1 + offset.pos.y);
//    end1.z = (ell_y1 + offset.pos.z);
//
//    end2.x = (1.0 + offset.pos.x);
//    end2.y = (-ell_x2 + offset.pos.y);
//    end2.z = (ell_y2 + offset.pos.z);
//
//    end3.x = (1.0 + offset.pos.x);
//    end3.y = (-ell_x3 + offset.pos.y);
//    end3.z = (ell_y3 + offset.pos.z);
//
//    end4.x = (1.0 + offset.pos.x);
//    end4.y = (-ell_x4 + offset.pos.y);
//    end4.z = (ell_y4 + offset.pos.z);
//    double beta1 = (atan2(end1.y - start.y, end1.x - start.x));
//    double beta2 = (atan2(end2.y - start.y, end2.x - start.x));
//    double beta3 = (atan2(end3.y - start.y, end3.x - start.x));
//    double beta4 = (atan2(end4.y - start.y, end4.x - start.x));
//    double alpha1 = (atan2(end1.z - start.z, end1.x - start.x));
//    double alpha2 = (atan2(end2.z - start.z, end2.x - start.x));
//    double alpha3 = (atan2(end3.z - start.z, end3.x - start.x));
//    double alpha4 = (atan2(end4.z - start.z, end4.x - start.x));
//
//    end1.x = (1.0 + offset.pos.x) * this->max_range_;
//    end1.y = (-ell_x1 + offset.pos.y) * this->max_range_;
//    end1.z = (ell_y1 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha1 + offset_rot.y, beta1 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end1 = (axis * this->max_range_) + offset.pos;
//
//    end2.x = (1.0 + offset.pos.x) * this->max_range_;
//    end2.y = (-ell_x2 + offset.pos.y) * this->max_range_;
//    end2.z = (ell_y2 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha2 + offset_rot.y, beta2 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end2 = (axis * this->max_range_) + offset.pos;
//
//    end3.x = (1.0 + offset.pos.x) * this->max_range_;
//    end3.y = (-ell_x3 + offset.pos.y) * this->max_range_;
//    end3.z = (ell_y3 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha3 + offset_rot.y, beta3 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end3 = (axis * this->max_range_) + offset.pos;
//
//    end4.x = (1.0 + offset.pos.x) * this->max_range_;
//    end4.y = (-ell_x4 + offset.pos.y) * this->max_range_;
//    end4.z = (ell_y4 + offset.pos.z) * this->max_range_;
//    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha4 + offset_rot.y, beta4 + offset_rot.z));
//    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
//    end4 = (axis * this->max_range_) + offset.pos;
//    std::string parent_name = this->parent_ray_sensor_->ParentName();
//
//    physics::CollisionPtr collision_ptr_1 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_1->SetName("own_ray_sensor_collision1");
//    collision_ptr_1->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_1);
//
//    gazebo::physics::RayShapePtr ray1 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//      collision_ptr_1->GetShape());
//
//    physics::CollisionPtr collision_ptr_2 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_2->SetName("own_ray_sensor_collision2");
//    collision_ptr_2->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_2);
//
//    gazebo::physics::RayShapePtr ray2 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//      collision_ptr_2->GetShape());
//
//    physics::CollisionPtr collision_ptr_3 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_3->SetName("own_ray_sensor_collision3");
//    collision_ptr_3->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_3);
//
//    gazebo::physics::RayShapePtr ray3 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//      collision_ptr_3->GetShape());
//
//    physics::CollisionPtr collision_ptr_4 = this->engine_->CreateCollision("ray", parent_name);
//    collision_ptr_4->SetName("own_ray_sensor_collision4");
//    collision_ptr_4->SetRelativePose(this->sensor_pose_);
//    collision_ptr_list_.push_back(collision_ptr_4);
//
//    gazebo::physics::RayShapePtr ray4 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
//      collision_ptr_4->GetShape());
//
//    ray1->SetPoints(start, end1);
//    ray2->SetPoints(start, end2);
//    ray3->SetPoints(start, end3);
//    ray4->SetPoints(start, end4);
//
//    this->rays_.push_back(ray1);
//    this->rays_.push_back(ray2);
//    this->rays_.push_back(ray3);
//    this->rays_.push_back(ray4);
//
//    // Push back the angles that are not transformed due to the pose because we create the pointcloud from the local frame.
//    // The psoe-orientation only plays a part in setting the ray-positions.
//    this->horizontal_ray_angles_.push_back(float(beta1));
//    this->horizontal_ray_angles_.push_back(float(beta2));
//    this->horizontal_ray_angles_.push_back(float(beta3));
//    this->horizontal_ray_angles_.push_back(float(beta4));
//    this->vertical_ray_angles_.push_back(float(alpha1));
//    this->vertical_ray_angles_.push_back(float(alpha2));
//    this->vertical_ray_angles_.push_back(float(alpha3));
//    this->vertical_ray_angles_.push_back(float(alpha4));
//  }

  this->parent_ray_sensor_->SetActive(true);
  //std::cout << "Time 5: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;
}

bool ArtiGazeboLaserLivox::AddDemoRayEllipseShape(double rotation_degrees)
{
  this->parent_ray_sensor_->SetActive(false);
  auto tstart = std::chrono::steady_clock::now();

  double samples_a = this->samples_;
  double ell_a = 0.3492; // length of ellipse parameter a in a distance of 1 m with a FOV of 38.4°
  double ell_b = 0.07275; // length of ellipse parameter b in a distance of 1 m with a FOV of 38.4°
  double dx = 2.0 * ell_a / samples_a;
  math::Quaternion ray;
  math::Vector3 axis;
  math::Pose offset;
  math::Vector3 start, end1, end2, end3, end4;

  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;

  std::string parent_name = this->parent_ray_sensor_->ParentName();
  this->collision_ptr_ = this->engine_->CreateCollision("ray", parent_name);
  this->collision_ptr_->SetName("own_ray_sensor_collision");
  this->sensor_pose_ = this->parent_ray_sensor_->Pose();
  this->collision_ptr_->SetRelativePose(this->sensor_pose_);
  this->collision_ptr_->SetInitialRelativePose(this->sensor_pose_);

  offset = this->collision_ptr_->GetRelativePose();
  math::Vector3 offset_rot = offset.rot.GetAsEuler();
  ray.SetFromEuler(math::Vector3(offset_rot.x, offset_rot.y, offset_rot.z));
  axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);

  // Get the position of the pose of the parent_ray_sensor and add its position to the new ray (its relative to the parent-frame)
  start = (axis * this->min_range_) + offset.pos;
  this->ray_startpoint_ = start;



  std::cout << "Time 1: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;

  for (int i = 0; i < samples_a; i++)
  {
    // use x as indexed variable that moves along the ellipse
    double val_x = ((double) i) * dx - ell_a;
    // calculate the two y-coordinates from the x-coordinate of the ellipse
    double val_y_1 = sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
    double val_y_2 = -sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));

    // The point (0,0) is in the middle of the ellipse, therefore we move the ellipse to one side by the amount of ell_a
    double val_x_1 = val_x + ell_a;
    // And a second time to the other side. Now we have four ellipse value pairs
    double val_x_2 = val_x - ell_a;

    // The four ellipse points are:
    // (val_x_1, val_y_1)
    // (val_x_1, val_y_2)
    // (val_x_2, val_y_1)
    // (val_x_2, val_y_2)

    double rot_rad = rotation_degrees / 180.0 * M_PI;
    double rot_mat[2][2] = {{cos(rot_rad), -sin(rot_rad)}, {sin(rot_rad), cos(rot_rad)}};

    double ell_x1 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_1;
    double ell_y1 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_1;

    double ell_x2 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_2;
    double ell_y2 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_2;

    double ell_x3 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_1;
    double ell_y3 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_1;

    double ell_x4 = 0.0;
    double ell_y4 = 0.0;
    // for the first sample there are 4 rays and 2 of them are equal. Modify one to get one ray in the middle
    // and 2 on the outer side of the ellipse.
    if (i == 0)
    {
      ell_x4 = rot_mat[0][0] * -val_x_2 + rot_mat[0][1] * val_y_2;
      ell_y4 = rot_mat[1][0] * -val_x_2 + rot_mat[1][1] * val_y_2;
    }
    else
    {
      ell_x4 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_2;
      ell_y4 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_2;
    }

    // Note: The current ellipse has the following axis: y is up and x is in the middle and pointing to the right.
    //       Therefore we need to map the y-axis of the ellipse to the Ray-z-axis and the x-axis of the ellipse to the
    //       -Ray-y-axis.

    end1.x = (1.0 + offset.pos.x);
    end1.y = (-ell_x1 + offset.pos.y);
    end1.z = (ell_y1 + offset.pos.z);

    end2.x = (1.0 + offset.pos.x);
    end2.y = (-ell_x2 + offset.pos.y);
    end2.z = (ell_y2 + offset.pos.z);

    end3.x = (1.0 + offset.pos.x);
    end3.y = (-ell_x3 + offset.pos.y);
    end3.z = (ell_y3 + offset.pos.z);

    end4.x = (1.0 + offset.pos.x);
    end4.y = (-ell_x4 + offset.pos.y);
    end4.z = (ell_y4 + offset.pos.z);

    double beta1 = (atan2(end1.y - start.y, end1.x - start.x));
    double beta2 = (atan2(end2.y - start.y, end2.x - start.x));
    double beta3 = (atan2(end3.y - start.y, end3.x - start.x));
    double beta4 = (atan2(end4.y - start.y, end4.x - start.x));
    double alpha1 = (atan2(end1.z - start.z, end1.x - start.x));
    double alpha2 = (atan2(end2.z - start.z, end2.x - start.x));
    double alpha3 = (atan2(end3.z - start.z, end3.x - start.x));
    double alpha4 = (atan2(end4.z - start.z, end4.x - start.x));

    end1.x = (1.0 + offset.pos.x) * this->max_range_;
    end1.y = (-ell_x1 + offset.pos.y) * this->max_range_;
    end1.z = (ell_y1 + offset.pos.z) * this->max_range_;
    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha1 + offset_rot.y, beta1 + offset_rot.z));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    end1 = (axis * this->max_range_) + offset.pos;

    end2.x = (1.0 + offset.pos.x) * this->max_range_;
    end2.y = (-ell_x2 + offset.pos.y) * this->max_range_;
    end2.z = (ell_y2 + offset.pos.z) * this->max_range_;
    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha2 + offset_rot.y, beta2 + offset_rot.z));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    end2 = (axis * this->max_range_) + offset.pos;

    end3.x = (1.0 + offset.pos.x) * this->max_range_;
    end3.y = (-ell_x3 + offset.pos.y) * this->max_range_;
    end3.z = (ell_y3 + offset.pos.z) * this->max_range_;
    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha3 + offset_rot.y, beta3 + offset_rot.z));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    end3 = (axis * this->max_range_) + offset.pos;

    end4.x = (1.0 + offset.pos.x) * this->max_range_;
    end4.y = (-ell_x4 + offset.pos.y) * this->max_range_;
    end4.z = (ell_y4 + offset.pos.z) * this->max_range_;
    ray.SetFromEuler(math::Vector3(0.0 + offset_rot.x, -alpha4 + offset_rot.y, beta4 + offset_rot.z));
    axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
    end4 = (axis * this->max_range_) + offset.pos;

    std::string parent_name = this->parent_ray_sensor_->ParentName();


    physics::CollisionPtr collision_ptr_1 = this->engine_->CreateCollision("ray", parent_name);
    collision_ptr_1->SetName("own_ray_sensor_collision1");
    collision_ptr_1->SetRelativePose(this->sensor_pose_);
    collision_ptr_list_.push_back(collision_ptr_1);

    gazebo::physics::RayShapePtr ray1 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      collision_ptr_1->GetShape());

    physics::CollisionPtr collision_ptr_2 = this->engine_->CreateCollision("ray", parent_name);
    collision_ptr_2->SetName("own_ray_sensor_collision2");
    collision_ptr_2->SetRelativePose(this->sensor_pose_);
    collision_ptr_list_.push_back(collision_ptr_2);

    gazebo::physics::RayShapePtr ray2 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      collision_ptr_2->GetShape());

    physics::CollisionPtr collision_ptr_3 = this->engine_->CreateCollision("ray", parent_name);
    collision_ptr_3->SetName("own_ray_sensor_collision3");
    collision_ptr_3->SetRelativePose(this->sensor_pose_);
    collision_ptr_list_.push_back(collision_ptr_3);

    gazebo::physics::RayShapePtr ray3 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      collision_ptr_3->GetShape());

    physics::CollisionPtr collision_ptr_4 = this->engine_->CreateCollision("ray", parent_name);
    collision_ptr_4->SetName("own_ray_sensor_collision4");
    collision_ptr_4->SetRelativePose(this->sensor_pose_);
    collision_ptr_list_.push_back(collision_ptr_4);

    gazebo::physics::RayShapePtr ray4 = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      collision_ptr_4->GetShape());

    ray1->SetPoints(start, end1);
    ray2->SetPoints(start, end2);
    ray3->SetPoints(start, end3);
    ray4->SetPoints(start, end4);

    this->rays_.push_back(ray1);
    this->rays_.push_back(ray2);
    this->rays_.push_back(ray3);
    this->rays_.push_back(ray4);


    // Push back the angles that are not transformed due to the pose because we create the pointcloud from the local frame.
    // The psoe-orientation only plays a part in setting the ray-positions.
    this->horizontal_ray_angles_.push_back(float(beta1));
    this->horizontal_ray_angles_.push_back(float(beta2));
    this->horizontal_ray_angles_.push_back(float(beta3));
    this->horizontal_ray_angles_.push_back(float(beta4));
    this->vertical_ray_angles_.push_back(float(alpha1));
    this->vertical_ray_angles_.push_back(float(alpha2));
    this->vertical_ray_angles_.push_back(float(alpha3));
    this->vertical_ray_angles_.push_back(float(alpha4));
  }

  this->parent_ray_sensor_->SetActive(true);
  //std::cout << "Time 5: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;
}

}
