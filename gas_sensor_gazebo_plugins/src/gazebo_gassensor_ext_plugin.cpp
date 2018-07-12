/*
 * Desc: Gassensor
 * Author: Haiwen Yuan
 */


#include "hector_gazebo_plugins/gazebo_gassensor_ext_plugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>


using namespace message_filters;

namespace gazebo {

GazeboRosGas::GazeboRosGas()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGas::~GazeboRosGas()
{
  updateTimer.Disconnect(updateConnection);
  
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGas::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
  concentrations_topic_ = "concentrations";
  sourcetopicName_1 = "/gas_model_1/Gas1Parameters";
  sourcetopicName_2 = "/gas_model_2/Gas2Parameters";
  sourcetopicName_3 = "/gas_model_3/Gas3Parameters";
  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    concentrations_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
  if (_sdf->HasElement("sourcetopicName1")) 
    sourcetopicName_1 = _sdf->Get<std::string>("sourcetopicName1");
  if (_sdf->HasElement("sourcetopicName2")) 
    sourcetopicName_2 = _sdf->Get<std::string>("sourcetopicName2");
  if (_sdf->HasElement("sourcetopicName3")) 
    sourcetopicName_3 = _sdf->Get<std::string>("sourcetopicName3");

  concentrations_.header.frame_id = frame_id_;


  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  
  concentration_publisher_ = node_handle_->advertise<sensor_msgs::MultipleConcentrations>(concentrations_topic_, 10);

  // ros::SubscribeOptions so1 = ros::SubscribeOptions::create<sensor_msgs::GasParameters>(sourcetopicName_1, 10, 
  //       boost::bind(GazeboRosGas::SourceCallback, _1, "Gas1Parameters"));
 // gassource1_sub_ = node_handle_->subscribe(so1);
 //  ros::SubscribeOptions so2 = ros::SubscribeOptions::create<sensor_msgs::GasParameters>(sourcetopicName_2, 10, 
 //        boost::bind(GazeboRosGas::SourceCallback, _1, "Gas2Parameters"));
 // gassource2_sub_ = node_handle_->subscribe(so2);
 //  ros::SubscribeOptions so3 = ros::SubscribeOptions::create<sensor_msgs::GasParameters>(sourcetopicName_3, 10, 
 //        boost::bind(GazeboRosGas::SourceCallback,  _1, "Gas3Parameters"));
 // gassource3_sub_ = node_handle_->subscribe(so3);
  
  message_filters::Subscriber<sensor_msgs::GasParameters> gassource1_sub_(node_handle_, sourcetopicName_1, 1);
  message_filters::Subscriber<sensor_msgs::GasParameters> gassource2_sub_(node_handle_, sourcetopicName_2, 1);
  message_filters::Subscriber<sensor_msgs::GasParameters> gassource3_sub_(node_handle_, sourcetopicName_3, 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::GasParameters, sensor_msgs::GasParameters, sensor_msgs::GasParameters> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gassource1_sub_, gassource2_sub_, gassource3_sub_);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  Reset();
  // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGas::Update, this));
 
}

void GazeboRosGas::SourceCallback(const sensor_msgs::GasParametersConstPtr &gassensor_msg1, const sensor_msgs::GasParametersConstPtr &gassensor_msg2, const sensor_msgs::GasParametersConstPtr &gassensor_msg3)
{
   common::Time sim_time = world->GetSimTime();
   double dt = updateTimer.getTimeSinceLastUpdate().Double();

   math::Pose pose = link->GetWorldPose();

   gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);

   concentrations_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);

   float gaussian_x_ = gassensor_msg1->x;
   float gaussian_y_ = gassensor_msg1->y;
   float sigma_x = gassensor_msg1->sigma_x;
   float sigma_y = gassensor_msg1->sigma_y;
   float A = gassensor_msg1->A;
  concentrations_.concentrations[1] = gassensor_msg->A*exp(-(((gaussian_x_-position.x)*(gaussian_x_-position.x))/(2*sigma_x*sigma_x))-(((gaussian_y_-position.y)*(gaussian_y_-position.y))/(2*sigma_y*sigma_y)));
  concentrations_.ids[1] = gassensor_msg1->id;
   

   gaussian_x_ = gassensor_msg2->x;
   gaussian_y_ = gassensor_msg2->y;
   sigma_x = gassensor_msg2->sigma_x;
   sigma_y = gassensor_msg2->sigma_y;
   A = gassensor_msg2->A;
  concentrations_.concentrations[2] = gassensor_msg->A*exp(-(((gaussian_x_-position.x)*(gaussian_x_-position.x))/(2*sigma_x*sigma_x))-(((gaussian_y_-position.y)*(gaussian_y_-position.y))/(2*sigma_y*sigma_y)));
  concentrations_.ids[2] = gassensor_msg2->id;
   

   gaussian_x_ = gassensor_msg3->x;
   gaussian_y_ = gassensor_msg3->y;
   sigma_x = gassensor_msg3->sigma_x;
   sigma_y = gassensor_msg3->sigma_y;
   A = gassensor_msg3->A;
  concentrations_.concentrations[3] = gassensor_msg->A*exp(-(((gaussian_x_-position.x)*(gaussian_x_-position.x))/(2*sigma_x*sigma_x))-(((gaussian_y_-position.y)*(gaussian_y_-position.y))/(2*sigma_y*sigma_y)));
  concentrations_.ids[3] = gassensor_msg3->id;

  concentration_publisher_.publish(concentrations_);
}

void GazeboRosGas::Reset()
{
  updateTimer.Reset();
  position_error_model_.reset();
  velocity_error_model_.reset();
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGas::Update()
{
    // handle callbacks
  queue_.callAvailable();


}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGas)

} // namespace gazebo
