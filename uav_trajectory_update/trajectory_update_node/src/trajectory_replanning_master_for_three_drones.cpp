
#include <mav_trajectory_generation_ros/trajectory_replanning_master_for_three_drones.h>

using namespace std; 

using namespace message_filters; 
using namespace Eigen;
const double v_max = 1.0;
const double a_max = 5.0;
const double magic_fabian_constant = 6.5; // A tuning parameter.
const int dimension = 4;
const int N = 10;
const float DEG_2_RAD = M_PI / 180.0;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
const float Threshold = 0.05;


TrajectoryReplanningMasterForThreedrones::TrajectoryReplanningMasterForThreedrones(const ros::NodeHandle& nh,
                                                               const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      stop_x(10.0), 
      stop_y(0.0), 
      stop_z(3.0),
      publish_whole_trajectory_(false),
      dt_(0.01),
      current_sample_time_1(0.0),
      current_sample_time_2(0.0),
      current_sample_time_3(0.0){
    string firefly1_command_trajectory_topic_;
    string firefly2_command_trajectory_topic_;
    string firefly3_command_trajectory_topic_;
    string firefly1_concentration_topic_;
    string firefly2_concentration_topic_;
    string firefly3_concentration_topic_;
    pnh_.param<string>("firefly1_command_trajectory_topic", firefly1_command_trajectory_topic_, "/firefly1/command/trajectory");
    pnh_.param<string>("firefly2_command_trajectory_topic", firefly2_command_trajectory_topic_, "/firefly2/command/trajectory");
    pnh_.param<string>("firefly3_command_trajectory_topic", firefly3_command_trajectory_topic_, "/firefly3/command/trajectory"); 
    pnh_.param<string>("left_concentration_topic", firefly1_concentration_topic_, "/firefly1/gas1_concentration");
    pnh_.param<string>("middle_concentration_topic", firefly2_concentration_topic_, "/firefly2/gas1_concentration");
    pnh_.param<string>("right_concentration_topic", firefly3_concentration_topic_, "/firefly3/gas1_concentration");
    pnh_.param("stop_x", stop_x, 0.0);
    pnh_.param("stop_y", stop_y, 0.0);
    pnh_.param("stop_z", stop_z, 3.0);
    pnh_.param("stop_yaw", stop_yaw, 0.0);
    pnh_.param("publish_whole_trajectory", publish_whole_trajectory_,  publish_whole_trajectory_);
    pnh_.param("dt", dt_, dt_);
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"stop_x = "<<stop_x<<", stop_z = "<<stop_z);
    command_pub_1_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(firefly1_command_trajectory_topic_, 1);
    command_pub_2_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(firefly2_command_trajectory_topic_, 1);
    command_pub_3_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(firefly3_command_trajectory_topic_, 1);
    stop_srv_ = nh_.advertiseService("stop_sampling", &TrajectoryReplanningMasterForThreedrones::stopSamplingCallback, this);
    publish_timer_1_ = nh_.createTimer(ros::Duration(dt_), &TrajectoryReplanningMasterForThreedrones::commandTimerCallback1, this, false);
    publish_timer_2_ = nh_.createTimer(ros::Duration(dt_), &TrajectoryReplanningMasterForThreedrones::commandTimerCallback2, this, false);
    publish_timer_3_ = nh_.createTimer(ros::Duration(dt_), &TrajectoryReplanningMasterForThreedrones::commandTimerCallback3, this, false);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_1(nh_, firefly1_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_2(nh_, firefly2_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_3(nh_, firefly3_concentration_topic_, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Concentration, sensor_msgs::Concentration, sensor_msgs::Concentration> sync(concentration_sub_1, concentration_sub_2, concentration_sub_3, 10);
    sync.registerCallback(boost::bind(&TrajectoryReplanningMasterForThreedrones::gasInformationCallback, this, _1, _2, _3));
    ros::spin();
}

TrajectoryReplanningMasterForThreedrones::~TrajectoryReplanningMasterForThreedrones(){
   publish_timer_1_.stop();
   publish_timer_2_.stop();
   publish_timer_3_.stop();
}
/////////////////////////////
bool TrajectoryReplanningMasterForThreedrones::stopSamplingCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  publish_timer_1_.stop();
  publish_timer_2_.stop();
  publish_timer_3_.stop();
  return true;
}
/////////////////////////////
void TrajectoryReplanningMasterForThreedrones::commandTimerCallback1(const ros::TimerEvent&) {

  if (current_sample_time_1 <= trajectory_1_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint flat_state;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_1_, current_sample_time_1, &flat_state);
    if (!success) {
      publish_timer_1_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state, &msg);
    msg.points[0].time_from_start = ros::Duration(current_sample_time_1);
    command_pub_1_.publish(msg);
    current_sample_time_1 += dt_;
  } else {
    publish_timer_1_.stop();
  }
}
void TrajectoryReplanningMasterForThreedrones::commandTimerCallback2(const ros::TimerEvent&) {

  if ( current_sample_time_2 <= trajectory_2_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint flat_state;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_2_, current_sample_time_2, &flat_state);
    if (!success) {
      publish_timer_2_.stop();
    }

    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state, &msg);
    msg.points[0].time_from_start = ros::Duration(current_sample_time_2);
    command_pub_2_.publish(msg);
    current_sample_time_2 += dt_;
  } else {
    publish_timer_2_.stop();
  }
}
void TrajectoryReplanningMasterForThreedrones::commandTimerCallback3(const ros::TimerEvent&) {

  if (current_sample_time_3 <= trajectory_3_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint flat_state;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_3_, current_sample_time_3, &flat_state);
    if (!success) {
      publish_timer_3_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state, &msg);
    msg.points[0].time_from_start = ros::Duration(current_sample_time_3);
    command_pub_3_.publish(msg);
    current_sample_time_3 += dt_;
  } else {
    publish_timer_3_.stop();
  }
}
/////////////////////////////
void TrajectoryReplanningMasterForThreedrones::WaypointUpdate(const double x, const double y, const sensor_msgs::Concentration::ConstPtr& firefly_gas, mav_trajectory_generation::Trajectory& trajectory){
          std::vector<Eigen::Vector4d> waypoints;
          waypoints.push_back(Eigen::Vector4d(x, y, firefly_gas->position[2], firefly_gas->position[3] * DEG_2_RAD));
          waypoints.push_back(Eigen::Vector4d(stop_x, y, stop_z, stop_yaw*DEG_2_RAD));
          mav_trajectory_generation::Vertex::Vector vertices;
          for (size_t j = 0; j < waypoints.size(); ++j) {
              if (j == 0){
                mav_trajectory_generation::Vertex start(dimension);
                start.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints[j]);//makeStartOrEnd(waypoints_1[i], derivative_to_optimize);
                vertices.push_back(start);
              }
              else if (j == waypoints.size()-1) {
                mav_trajectory_generation::Vertex end(dimension);
                end.makeStartOrEnd(waypoints[j], derivative_to_optimize);
                vertices.push_back(end);
              }
              else {
                mav_trajectory_generation::Vertex middle(dimension);
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints[j]);
                vertices.push_back(middle);
              }
          }
          std::vector<double> segment_times;
          segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);
          mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
          opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
          opt.solveLinear();
          opt.getTrajectory(&trajectory);
}

void TrajectoryReplanningMasterForThreedrones::gasInformationCallback(const sensor_msgs::Concentration::ConstPtr& firefly1_gas, const sensor_msgs::Concentration::ConstPtr& firefly2_gas, const sensor_msgs::Concentration::ConstPtr& firefly3_gas)
{
    //ROS_INFO("recieved gas information");

    double concentration_[3];
    double position_[3];
    concentration_[0] = firefly1_gas->concentration;
    concentration_[1] = firefly2_gas->concentration;
    concentration_[2] = firefly3_gas->concentration;
    position_[0] = position_[0];
    position_[1] = position_[1];
    position_[2] = position_[2];

    double x[3];
    double y[3];
    double d1, d2;
    if ((concentration_[0]-concentration_[1])>=Threshold && (concentration_[1]-concentration_[2])>=Threshold){
       d1 = (position_[0]-position_[1])*(0.5+1/(4*(concentration_[0]-concentration_[1])+2));
       d2 = (position_[1]-position_[2])*(0.5+1/(4*(concentration_[1]-concentration_[2])+2));
       if (d1 < 2) d1 = 2;
       else if (d1 > 10) d1 = 10;
       if (d2 < 2) d2 = 2;
       else if (d2 > 10) d2 = 10;
       y[1] = position_[0];
       y[0] = y[1] + d1;
       y[2] = y[1] - d2;
   }
   else if ((concentration_[1]-concentration_[0])>=Threshold && (concentration_[2]-concentration_[1])>=Threshold){
       d1 = (position_[0]-position_[1])*(0.5+1/(4*(concentration_[1]-concentration_[0])+2));
       d2 = (position_[1]-position_[2])*(0.5+1/(4*(concentration_[2]-concentration_[1])+2));
       if (d1 < 2) d1 = 2;
       else if (d1 > 10) d1 = 10;
       if (d2 < 2) d2 = 2;
       else if (d2 > 10) d2 = 10;
       y[1] = position_[2];
       y[0] = y[1] + d1;
       y[2] = y[1] - d2;
    }
    else if ((concentration_[1]-concentration_[0])>=Threshold && (concentration_[1]-concentration_[2])>=Threshold){
       d1 = (position_[0]-position_[1])*(0.5+1/(4*(concentration_[1]-concentration_[0])+2));
       d2 = (position_[1]-position_[2])*(0.5+1/(4*(concentration_[1]-concentration_[0])+2));
       if (d1 < 2) d1 = 2;
       else if (d1 > 10) d1 = 10;
       if (d2 < 2) d2 = 2;
       else if (d2 > 10) d2 = 10;
       y[1] = position_[1];
       y[0] = y[1] + d1;
       y[2] = y[1] - d2;
    }
    else if ((concentration_[0]-concentration_[1])>=Threshold && abs(concentration_[1]-concentration_[2])<Threshold){              ////#4
       d1 = (position_[0]-position_[1])*(0.5+1/(4*(concentration_[0]-concentration_[1])+2));
       d2 = (position_[1]-position_[2]);
       if (d1 < 2) d1 = 2;
       else if (d1 > 10) d1 = 10;
       if (d2 < 2) d2 = 2;
       else if (d2 > 10) d2 = 10;
       y[1] = position_[0];
       y[0] = y[1] + d1;
       y[2] = y[1] - d2;
    }
    else if (abs(concentration_[1]-concentration_[0])<Threshold && (concentration_[2]-concentration_[1])>=Threshold){              ////#5
       d1 = (position_[0]-position_[1])*(0.5+1/(4*(concentration_[1]-concentration_[0])+2));
       d2 = (position_[1]-position_[2]);
       if (d1 < 2) d1 = 2;
       else if (d1 > 10) d1 = 10;
       if (d2 < 2) d2 = 2;
       else if (d2 > 10) d2 = 10;
       y[1] = position_[2] + 0.5*d2;
       y[0] = y[1] + d1;
       y[2] = y[1] - d2;
    }
    else if (abs(concentration_[0]-concentration_[1])<Threshold && (concentration_[0]-concentration_[2])>=Threshold){              ////#6
       d1 = (position_[0]-position_[1]);
       d2 = (position_[1]-position_[2])*(0.5+1/(4*(concentration_[1]-concentration_[2])+2));
       if (d1 < 2) d1 = 2;
       else if (d1 > 10) d1 = 10;
       if (d2 < 2) d2 = 2;
       else if (d2 > 10) d2 = 10;
       y[1] = position_[0] - 0.5*d1;
       y[0] = y[1] + d1;
       y[2] = y[1] - d2;
    }
    else if ((concentration_[2]-concentration_[1])>=Threshold && abs(concentration_[0]-concentration_[1])<Threshold){              ////#7
       d1 = (position_[0]-position_[1]);
       d2 = (position_[1]-position_[2])*(0.5+1/(4*(concentration_[2]-concentration_[1])+2));
       if (d1 < 2) d1 = 2;
       else if (d1 > 10) d1 = 10;
       if (d2 < 2) d2 = 2;
       else if (d2 > 10) d2 = 10;
       y[1] = position_[2];
       y[0] = y[1] + d1;
       y[2] = y[1] - d2;
    }
    else{
       y[1] = position_[1];
       y[0] = y[1] + 5;
       y[2] = y[1] - 5;
    } 

       x[0] = firefly1_gas->position[0]+0.2;
       x[1] = firefly2_gas->position[0]+0.2;
       x[2] = firefly3_gas->position[0]+0.2;
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"x1 ="<<x[0]<<", y[0] ="<<y[0]<<"; "<<"x2= "<<x[1]<<", y[1]= "<<y[1]<<"; "<<"x3= "<<x[2]<<", y[2]= "<<y[2]<<".");


       if (x[0]<stop_x | x[1]<stop_x | x[2]<stop_x){
          TrajectoryReplanningMasterForThreedrones::WaypointUpdate(x[0],  y[0], firefly1_gas, trajectory_1_);
          TrajectoryReplanningMasterForThreedrones::WaypointUpdate(x[1],  y[1], firefly2_gas, trajectory_2_);
          TrajectoryReplanningMasterForThreedrones::WaypointUpdate(x[2],  y[2], firefly3_gas, trajectory_3_);
       /*   std::vector<Eigen::Vector4d> waypoints_1;
          waypoints_1.push_back(Eigen::Vector4d(x[0], y[0], firefly1_gas->position[2], firefly1_gas->position[3] * DEG_2_RAD));
          waypoints_1.push_back(Eigen::Vector4d(stop_x, y[0], stop_z, stop_yaw*DEG_2_RAD));
          mav_trajectory_generation::Vertex::Vector vertices_1;
          for (size_t j = 0; j < waypoints_1.size(); ++j) {
              if (j == 0){
                mav_trajectory_generation::Vertex start(dimension);
                start.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints_1[j]);//makeStartOrEnd(waypoints_1[i], derivative_to_optimize);
                vertices_1.push_back(start);
              }
              else if (j == waypoints_1.size()-1) {
                mav_trajectory_generation::Vertex end(dimension);
                end.makeStartOrEnd(waypoints_1[j], derivative_to_optimize);
                vertices_1.push_back(end);
              }
              else {
                mav_trajectory_generation::Vertex middle(dimension);
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints_1[j]);
                vertices_1.push_back(middle);
              }
          }
          std::vector<double> segment_times_1;
          segment_times_1 = estimateSegmentTimes(vertices_1, v_max, a_max, magic_fabian_constant);
          mav_trajectory_generation::PolynomialOptimization<N> opt1(dimension);
          opt1.setupFromVertices(vertices_1, segment_times_1, derivative_to_optimize);
          opt1.solveLinear();
          opt1.getTrajectory(&trajectory_1_);


     std::vector<Eigen::Vector4d> waypoints_2;
     waypoints_2.push_back(Eigen::Vector4d(x[1], y[1], firefly2_gas->position[2], firefly2_gas->position[3] * DEG_2_RAD));
     waypoints_2.push_back(Eigen::Vector4d(stop_x, y[1], stop_z, stop_yaw*DEG_2_RAD));
     mav_trajectory_generation::Vertex::Vector vertices_2;
     for (size_t i = 0; i < waypoints_2.size(); ++i) {
      if (i == 0){
         mav_trajectory_generation::Vertex start(dimension);
         start.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints_2[i]);//makeStartOrEnd(waypoints_2[i], derivative_to_optimize);
         vertices_2.push_back(start);
      }
      else if (i == waypoints_2.size()-1) {
         mav_trajectory_generation::Vertex end(dimension);
         end.makeStartOrEnd(waypoints_2[i], derivative_to_optimize);
         vertices_2.push_back(end);
      }
      else {
         mav_trajectory_generation::Vertex middle(dimension);
         middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints_2[i]);
         vertices_2.push_back(middle);
      }
    }
    std::vector<double> segment_times_2;
    segment_times_2 = estimateSegmentTimes(vertices_2, v_max, a_max, magic_fabian_constant);
      mav_trajectory_generation::PolynomialOptimization<N> opt2(dimension);
    opt2.setupFromVertices(vertices_2, segment_times_2, derivative_to_optimize);
    opt2.solveLinear();
    opt2.getTrajectory(&trajectory_2_);

    std::vector<Eigen::Vector4d> waypoints_3;
    waypoints_3.push_back(Eigen::Vector4d(x[2], y[2], firefly3_gas->position[2], firefly3_gas->position[3] * DEG_2_RAD));
    waypoints_3.push_back(Eigen::Vector4d(stop_x, y[2], stop_z, stop_yaw*DEG_2_RAD));
    mav_trajectory_generation::Vertex::Vector vertices_3;
    for (size_t i = 0; i < waypoints_3.size(); ++i) {
      if (i == 0){
         mav_trajectory_generation::Vertex start(dimension);
         start.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints_3[i]);//makeStartOrEnd(waypoints_3[i], derivative_to_optimize);
         vertices_3.push_back(start);
      }
      else if (i == waypoints_3.size()-1) {
         mav_trajectory_generation::Vertex end(dimension);
         end.makeStartOrEnd(waypoints_3[i], derivative_to_optimize);
         vertices_3.push_back(end);
      }
      else {
         mav_trajectory_generation::Vertex middle(dimension);
         middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints_3[i]);
         vertices_3.push_back(middle);
      }
    }
    std::vector<double> segment_times_3;
    segment_times_3 = estimateSegmentTimes(vertices_3, v_max, a_max, magic_fabian_constant);
    mav_trajectory_generation::PolynomialOptimization<N> opt3(dimension);
    opt3.setupFromVertices(vertices_3, segment_times_3, derivative_to_optimize);
    opt3.solveLinear();
    opt3.getTrajectory(&trajectory_3_);
*/
  
    if (publish_whole_trajectory_) {
    // Publish the entire trajectory at once.
       mav_msgs::EigenTrajectoryPoint::Vector flat_states1, flat_states2, flat_states3;

       mav_trajectory_generation::sampleWholeTrajectory(trajectory_1_, dt_,
                                                     &flat_states1);
       mav_trajectory_generation::sampleWholeTrajectory(trajectory_2_, dt_,
                                                     &flat_states2);
       mav_trajectory_generation::sampleWholeTrajectory(trajectory_3_, dt_,
                                                     &flat_states3);
       trajectory_msgs::MultiDOFJointTrajectory msg_pub_1, msg_pub_2, msg_pub_3;
       msgMultiDofJointTrajectoryFromEigen(flat_states1, &msg_pub_1);
       msgMultiDofJointTrajectoryFromEigen(flat_states2, &msg_pub_2);
       msgMultiDofJointTrajectoryFromEigen(flat_states3, &msg_pub_3);
       command_pub_1_.publish(msg_pub_1);
       command_pub_2_.publish(msg_pub_2);
       command_pub_3_.publish(msg_pub_3);
    } else {
      publish_timer_1_.start();
      publish_timer_2_.start();
      publish_timer_3_.start();
      current_sample_time_1 = 0;
      current_sample_time_2 = 0;
      current_sample_time_3 = 0;
      start_time_ = ros::Time::now();
    }
  }
}



int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_replanning_timeupdate");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
    ros::Duration(30).sleep();
    TrajectoryReplanningMasterForThreedrones trajectory_replanning_master_for_three_drones(nh, pnh);
    ROS_INFO("Initialized trajectory replanning node.");


}
