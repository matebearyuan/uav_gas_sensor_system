#ifndef TRAJECTORY_REPLANNING_MASTER_FOR_THREE_DRONES_H
#define TRAJECTORY_REPLANNING_MASTER_FOR_THREE_DRONES_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Concentration.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <planning_msgs/PolynomialSegment4D.h>
#include <planning_msgs/PolynomialTrajectory4D.h>

#include <thread>
#include <chrono>
#include <map>
#include <Eigen/Eigen>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class TrajectoryReplanningMasterForThreedrones {
 public:
  TrajectoryReplanningMasterForThreedrones(const ros::NodeHandle& nh,  const ros::NodeHandle& pnh);
  ~TrajectoryReplanningMasterForThreedrones();

 private:
   void gasInformationCallback(const sensor_msgs::Concentration::ConstPtr& firefly1_gas, const sensor_msgs::Concentration::ConstPtr& firefly2_gas, const sensor_msgs::Concentration::ConstPtr& firefly3_gas);
  void WaypointUpdate(const double x, const double y, const sensor_msgs::Concentration::ConstPtr& firefly_gas, mav_trajectory_generation::Trajectory& trajectory);
  bool stopSamplingCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);
  void commandTimerCallback1(const ros::TimerEvent&);
  void commandTimerCallback2(const ros::TimerEvent&);
  void commandTimerCallback3(const ros::TimerEvent&);
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher command_pub_1_;
  ros::Publisher command_pub_2_;
  ros::Publisher command_pub_3_;
  ros::Timer publish_timer_1_;
  ros::Timer publish_timer_2_;
  ros::Timer publish_timer_3_;
  ros::ServiceServer stop_srv_;
  ros::Time start_time_;

  // Flag whether to publish entire trajectory at once or not.
  bool publish_whole_trajectory_;
  // Trajectory sampling interval.
  double dt_;
  // Time at currently published trajectory sample.
  double current_sample_time_1;
  double current_sample_time_2;
  double current_sample_time_3;
  double stop_x, stop_y, stop_z, stop_yaw;
  mav_trajectory_generation::Trajectory trajectory_1_;
  mav_trajectory_generation::Trajectory trajectory_2_;
  mav_trajectory_generation::Trajectory trajectory_3_;
  mav_trajectory_generation::Trajectory::Vector trajectory_;
 
};

#endif  // TRAJECTORY_REPLANNING_MASTER_FOR_THREE_DRONES_H