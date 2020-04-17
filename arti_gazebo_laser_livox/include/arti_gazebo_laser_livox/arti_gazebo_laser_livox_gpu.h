#ifndef ARTI_GAZEBO_LASER_LIVOX_H
#define ARTI_GAZEBO_LASER_LIVOX_H

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud2.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/GpuRayPlugin.hh>

#include <sdf/Param.hh>

namespace gazebo
{

class ArtiGazeboLaserLivox: public GpuRayPlugin
{
public:
  // Public functions
  ArtiGazeboLaserLivox();
  ~ArtiGazeboLaserLivox();
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

protected:
  // Protected functions
  virtual void OnNewLaserScans();

private:
  // Functions
  void PutLaserData(common::Time &_updateTime);
  int connect_count_;
  void Connect();
  void Disconnect();
  void LoadThread();
  void LaserQueueThread();
  bool AddRayEllipseShape(double rotation_degrees);

  // Variables
  physics::WorldPtr world_;
  sensors::SensorPtr parent_sensor_;
  sensors::GpuRaySensorPtr parent_ray_sensor_;

  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  std::string topic_name_;
  std::string frame_name_;
  double update_rate_;
  double update_period_;
  common::Time last_update_time_;
  std::string robot_namespace_;
  ros::CallbackQueue laser_queue_;
  boost::thread callback_queue_thread_;
  sdf::ElementPtr sdf;

  boost::thread deferred_load_thread_;
  gazebo::physics::PhysicsEnginePtr engine_;

  std::vector<gazebo::physics::RayShapePtr> rays_;
  physics::CollisionPtr collision_ptr_;
  std::vector<physics::CollisionPtr> collision_ptr_list_;
  ignition::math::Pose3d sensor_pose_;
  std::vector<float> vertical_ray_angles_;
  std::vector<float> horizontal_ray_angles_;
  math::Vector3 ray_startpoint_;
  sensor_msgs::PointCloud cloud_msg_;
  sensor_msgs::PointCloud2 pc2_msgs_;

  int samples_;
  int num_ellipses_;
  float min_range_;
  float max_range_;
  float rotation_increment_;
  double current_rot_angle_;
  size_t interpolation_points_;

  bool init_finished_ = false;

  int debug_counter = 0;

};
}
#endif // ARTI_GAZEBO_LASER_LIVOX_H
