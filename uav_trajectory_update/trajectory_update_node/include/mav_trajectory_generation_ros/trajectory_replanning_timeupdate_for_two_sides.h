#ifndef TRAJECTORY_REPLANNING_TIMEUPDATE_H
#define TRAJECTORY_REPLANNING_TIMEUPDATE_H

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
#include <Eigen/Core>
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

class TrajectoryReplanningTimeupdateForTwosides {
 public:
  TrajectoryReplanningTimeupdateForTwosides(const ros::NodeHandle& nh,  const ros::NodeHandle& pnh);
  ~TrajectoryReplanningTimeupdateForTwosides();

 private:
   void gasInformationCallback(const sensor_msgs::Concentration::ConstPtr& master_gas, const sensor_msgs::Concentration::ConstPtr& near_gas, const sensor_msgs::Concentration::ConstPtr& far_gas);
  bool stopSamplingCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);
  void commandTimerCallback(const ros::TimerEvent&);
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher command_pub_;

  ros::Timer publish_timer_;
  ros::ServiceServer stop_srv_;
  ros::Time start_time_;

  // Flag whether to publish entire trajectory at once or not.
  bool publish_whole_trajectory_;
  // Trajectory sampling interval.
  double dt_;
  // Time at currently published trajectory sample.
  double current_sample_time_;
  double start_x, start_y, start_z, start_yaw;
  double middle_x, middle_y, middle_z, middle_yaw;
  double stop_x, stop_y, stop_z, stop_yaw;
  mav_trajectory_generation::Trajectory trajectory_;
};

#endif  // TRAJECTORY_REPLANNING_TIMEUPDATE_H
