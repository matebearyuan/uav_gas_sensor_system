/*
 * Desc: Gassensor
 * Author: Haiwen Yuan
 */


#include "hector_gazebo_plugins/gazebo_gassensor_plugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>



namespace gazebo {

GazeboRosGasSensor::GazeboRosGasSensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGasSensor::~GazeboRosGasSensor()
{
  updateTimer.Disconnect(updateConnection);
  
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGasSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
    ROS_FATAL("GazeboRosGps plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = "/world";
  concentration_topic_ = "concentration";
  sourcetopicName = "/smallbox_target_red/GasParameters";

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    concentration_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
  if (_sdf->HasElement("sourcetopicName")) 
    sourcetopicName = _sdf->Get<std::string>("sourcetopicName");


  concentration_.header.frame_id = frame_id_;


  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  
  concentration_publisher_ = node_handle_->advertise<sensor_msgs::Concentration>(concentration_topic_, 10);

   ros::SubscribeOptions so = ros::SubscribeOptions::create<sensor_msgs::GasParameters>(sourcetopicName, 1, 
         boost::bind(&GazeboRosGasSensor::SourceCallback, this, _1), ros::VoidPtr(), &queue_);
  gassource_sub_ = node_handle_->subscribe(so);
  
  
  Reset();
  // connect Update function
  updateTimer.setUpdateRate(10.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGasSensor::Update, this));
 
}

void GazeboRosGasSensor::SourceCallback(const sensor_msgs::GasParametersConstPtr &gassensor_msg)
{
   common::Time sim_time = world->GetSimTime();
   double dt = updateTimer.getTimeSinceLastUpdate().Double();

   math::Pose pose = link->GetWorldPose();

   gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);

   concentration_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);

   float gaussian_x_ = 0;
   gaussian_x_ = gassensor_msg->center[0];
   float gaussian_y_ = 0;
   gaussian_y_ = gassensor_msg->center[1];
   float sigma_x = gassensor_msg->sigma[0];
   float sigma_y = gassensor_msg->sigma[1];
  concentration_.concentration = gassensor_msg->A*exp(-(((gaussian_x_-position.x)*(gaussian_x_-position.x))/(2*sigma_x*sigma_x))-(((gaussian_y_-position.y)*(gaussian_y_-position.y))/(2*sigma_y*sigma_y)));
  concentration_.id = gassensor_msg->id;
  concentration_.position[0] = position.x;
  concentration_.position[1] = position.y;
  concentration_.position[2] = position.z;
  concentration_.position[3] = pose.rot.GetYaw();
  concentration_publisher_.publish(concentration_);
}

void GazeboRosGasSensor::Reset()
{
  updateTimer.Reset();
  position_error_model_.reset();
  velocity_error_model_.reset();
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGasSensor::Update()
{
    // handle callbacks
  queue_.callAvailable();


}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGasSensor)

} // namespace gazebo
