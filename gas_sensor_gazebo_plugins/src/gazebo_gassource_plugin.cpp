/*
 * Desc: Gassensor
 * Author: Haiwen Yuan
 */


#include "hector_gazebo_plugins/gazebo_gassource_plugin.h"
#include <gazebo/physics/physics.hh>

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;
namespace gazebo {

GazeboRosGasSource::GazeboRosGasSource()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGasSource::~GazeboRosGasSource()
{
  updateTimer.Disconnect(updateConnection);

  dynamic_reconfigure_server_position_.reset();
  dynamic_reconfigure_server_velocity_.reset();
  dynamic_reconfigure_server_status_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGasSource::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("GazeboRosSource plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = "/world";

  gassource_topic_ = "source_parameters";
  fix_topic_ = "fix";

  reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
  reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
  reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

  fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
  fix_.status.service = sensor_msgs::NavSatStatus::STATUS_FIX;

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    gassource_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
  if (_sdf->HasElement("gasId"))
    _sdf->GetElement("gasId")->GetValue()->Get(gas_id);
  if (_sdf->HasElement("sigmaX"))
    _sdf->GetElement("sigmaX")->GetValue()->Get(gussian_sigma_x);
  if (_sdf->HasElement("sigmaY"))
    _sdf->GetElement("sigmaY")->GetValue()->Get(gussian_sigma_y);
  if (_sdf->HasElement("gaussianA"))
    _sdf->GetElement("gaussianA")->GetValue()->Get(gussian_A);
  
  position_error_model_.Load(_sdf);
  velocity_error_model_.Load(_sdf, "velocity");
  gassource_.header.frame_id = frame_id_;
  fix_.header.frame_id = frame_id_;

  // calculate earth radii
  double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
  double prime_vertical_radius = equatorial_radius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
  radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);
  gassource_publisher_ = node_handle_->advertise<sensor_msgs::GasParameters>(gassource_topic_, 10);
  

 // setup dynamic_reconfigure servers
  dynamic_reconfigure_server_position_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/position")));
  dynamic_reconfigure_server_velocity_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/velocity")));
  dynamic_reconfigure_server_status_.reset(new dynamic_reconfigure::Server<GNSSConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/status")));
  dynamic_reconfigure_server_position_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &position_error_model_, _1, _2));
  dynamic_reconfigure_server_velocity_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &velocity_error_model_, _1, _2));
  dynamic_reconfigure_server_status_->setCallback(boost::bind(&GazeboRosGasSource::dynamicReconfigureCallback, this, _1, _2));

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(10.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGasSource::Update, this));
}

void GazeboRosGasSource::Reset()
{
  updateTimer.Reset();
  position_error_model_.reset();
  velocity_error_model_.reset();
}

void GazeboRosGasSource::dynamicReconfigureCallback(GazeboRosGasSource::GNSSConfig &config, uint32_t level)
{
  using sensor_msgs::NavSatStatus;

  if (level == 1) {
    if (!config.STATUS_FIX) {
      fix_.status.status = NavSatStatus::STATUS_NO_FIX;
    } else {
      fix_.status.status = (config.STATUS_SBAS_FIX ? NavSatStatus::STATUS_SBAS_FIX : 0) |
                           (config.STATUS_GBAS_FIX ? NavSatStatus::STATUS_GBAS_FIX : 0);
    }
    fix_.status.service = (config.SERVICE_GPS     ? NavSatStatus::SERVICE_GPS : 0) |
                          (config.SERVICE_GLONASS ? NavSatStatus::SERVICE_GLONASS : 0) |
                          (config.SERVICE_COMPASS ? NavSatStatus::SERVICE_COMPASS : 0) |
                          (config.SERVICE_GALILEO ? NavSatStatus::SERVICE_GALILEO : 0);
  } else {
    config.STATUS_FIX      = (fix_.status.status != NavSatStatus::STATUS_NO_FIX);
    config.STATUS_SBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_SBAS_FIX);
    config.STATUS_GBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_GBAS_FIX);
    config.SERVICE_GPS     = (fix_.status.service & NavSatStatus::SERVICE_GPS);
    config.SERVICE_GLONASS = (fix_.status.service & NavSatStatus::SERVICE_GLONASS);
    config.SERVICE_COMPASS = (fix_.status.service & NavSatStatus::SERVICE_COMPASS);
    config.SERVICE_GALILEO = (fix_.status.service & NavSatStatus::SERVICE_GALILEO);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGasSource::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  math::Pose pose = link->GetWorldPose();
  
  gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
  gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);

  // An offset error in the velocity is integrated into the position error for the next timestep.
  // Note: Usually GNSS receivers have almost no drift in the velocity signal.
  position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());
  fix_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
   fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.x + sin(reference_heading_) * position.y) / radius_north_ * 180.0/M_PI;
  fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.x + cos(reference_heading_) * position.y) / radius_east_  * 180.0/M_PI;
  fix_.altitude  = reference_altitude_  + position.z;
  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_.position_covariance[0] = position_error_model_.drift.x*position_error_model_.drift.x + position_error_model_.gaussian_noise.x*position_error_model_.gaussian_noise.x;
  fix_.position_covariance[4] = position_error_model_.drift.y*position_error_model_.drift.y + position_error_model_.gaussian_noise.y*position_error_model_.gaussian_noise.y;
  fix_.position_covariance[8] = position_error_model_.drift.z*position_error_model_.drift.z + position_error_model_.gaussian_noise.z*position_error_model_.gaussian_noise.z;

  fix_publisher_.publish(fix_);
  gassource_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  gassource_.id  = gas_id;
  gassource_.center[0]  = pose.pos.x;
  gassource_.center[1] = pose.pos.y;
  gassource_.sigma[0]  = gussian_sigma_x;
  gassource_.sigma[1]  = gussian_sigma_y;
  gassource_.A  = gussian_A;

  gassource_publisher_.publish(gassource_);

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGasSource)

} // namespace gazebo
