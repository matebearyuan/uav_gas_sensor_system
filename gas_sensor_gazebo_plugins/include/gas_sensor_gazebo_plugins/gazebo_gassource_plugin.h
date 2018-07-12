

/*
 * Desc: Gassource
 * Author: Haiwen Yuan
 */

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_GASSOURCE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_GASSOURCE_PLUGIN_H
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/GasParameters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>

#include <dynamic_reconfigure/server.h>
#include <hector_gazebo_plugins/GNSSConfig.h>

namespace gazebo
{

class GazeboRosGasSource : public ModelPlugin
{
public:
  GazeboRosGasSource();
  virtual ~GazeboRosGasSource();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();
  typedef hector_gazebo_plugins::GNSSConfig GNSSConfig;
  void dynamicReconfigureCallback(GNSSConfig &config, uint32_t level);

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;
  
  ros::NodeHandle* node_handle_;
  ros::Publisher gassource_publisher_;
  ros::Publisher fix_publisher_;
  sensor_msgs::GasParameters gassource_;
  sensor_msgs::NavSatFix fix_;

  std::string namespace_;
  std::string link_name_;
  std::string frame_id_;
  std::string gassource_topic_;
  std::string fix_topic_;


  double gussian_sigma_x = 4;
  double gussian_sigma_y = 6;
  double gussian_A = 1;
  int gas_id = 1;

  double reference_latitude_;
  double reference_longitude_;
  double reference_heading_;
  double reference_altitude_;

  double radius_north_;
  double radius_east_;
  SensorModel3 position_error_model_;
  SensorModel3 velocity_error_model_;
  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;

  boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_position_, dynamic_reconfigure_server_velocity_;
  boost::shared_ptr<dynamic_reconfigure::Server<GNSSConfig> > dynamic_reconfigure_server_status_;
};

} // namespace gazebo

#endif 
