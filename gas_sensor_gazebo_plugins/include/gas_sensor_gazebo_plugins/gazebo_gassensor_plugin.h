

/*
 * Desc: Gassensor
 * Author: Haiwen Yuan
 */

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_GASSENSOR_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_GASSENSOR_PLUGIN_H
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/GasParameters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>
#include <sensor_msgs/Concentration.h>
#include <dynamic_reconfigure/server.h>
#include <hector_gazebo_plugins/GNSSConfig.h>
// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

class GazeboRosGasSensor : public ModelPlugin
{
public:
  GazeboRosGasSensor();
  virtual ~GazeboRosGasSensor();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();

  virtual void Update();


private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;
  
  // Custom Callback Queue
  ros::CallbackQueue queue_;
  ros::NodeHandle* node_handle_;

  ros::Publisher concentration_publisher_;
  ros::Subscriber gassource_sub_;
  

  
  void SourceCallback(const sensor_msgs::GasParametersConstPtr &gassensor_msg);
  sensor_msgs::GasParameters::ConstPtr current_gassensor_msg;
  /// \brief    Gas Sensor message to be published on sensor update.
  sensor_msgs::Concentration concentration_;

  std::string namespace_;
  std::string link_name_;
  std::string frame_id_;

  std::string concentration_topic_;
  std::string sourcetopicName;
  double reference_latitude_;
  double reference_longitude_;
  double reference_heading_;
  double reference_altitude_;


  SensorModel3 position_error_model_;
  SensorModel3 velocity_error_model_;
  
  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;

};

} // namespace gazebo

#endif 

