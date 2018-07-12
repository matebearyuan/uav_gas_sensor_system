/*
 * Copyright (c) 2017, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mav_trajectory_generation_ros/trajectory_sampler_node.h>

bool sim_running = false;
void callback(const sensor_msgs::ImuPtr& /*msg*/) {
  sim_running = true;
}

TrajectorySamplerNode::TrajectorySamplerNode(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      publish_whole_trajectory_(false),
      dt_(0.01),
      current_sample_time_(0.0) {
  nh_private_.param("publish_whole_trajectory", publish_whole_trajectory_,
                    publish_whole_trajectory_);
  nh_private_.param("dt", dt_, dt_);

  command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  trajectory_sub_ = nh_.subscribe(
      "path_segments", 10, &TrajectorySamplerNode::pathSegmentsCallback, this);
  stop_srv_ = nh_.advertiseService(
      "stop_sampling", &TrajectorySamplerNode::stopSamplingCallback, this);
  publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                   &TrajectorySamplerNode::commandTimerCallback,
                                   this, false);
}

TrajectorySamplerNode::~TrajectorySamplerNode() { publish_timer_.stop(); }

void TrajectorySamplerNode::pathSegmentsCallback(
    const planning_msgs::PolynomialTrajectory4D& segments_message) {
  if (segments_message.segments.empty()) {
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  } else {
    ROS_INFO("Trajectory sampler: received %lu waypoints",
             segments_message.segments.size());
  }

  bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
      segments_message, &trajectory_);
  if (!success) {
    return;
  }

  if (publish_whole_trajectory_) {
    // Publish the entire trajectory at once.
    mav_msgs::EigenTrajectoryPoint::Vector flat_states;
    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_,
                                                     &flat_states);
    trajectory_msgs::MultiDOFJointTrajectory msg_pub;
    msgMultiDofJointTrajectoryFromEigen(flat_states, &msg_pub);
    command_pub_.publish(msg_pub);
  } else {
    publish_timer_.start();
    current_sample_time_ = 0;
    start_time_ = ros::Time::now();
  }
}

bool TrajectorySamplerNode::stopSamplingCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  publish_timer_.stop();
  return true;
}

void TrajectorySamplerNode::commandTimerCallback(const ros::TimerEvent&) {
  if (current_sample_time_ <= trajectory_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint flat_state;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &flat_state);
    if (!success) {
      publish_timer_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state, &msg);
    msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    command_pub_.publish(msg);
    current_sample_time_ += dt_;
  } else {
    publish_timer_.stop();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_sampler_node");
  ros::NodeHandle nh;
  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);
  ros::Publisher command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);
  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...ok");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
  ros::Duration(30).sleep();
  ROS_INFO("Start publishing waypoints.");
  


  if (args.size() != 2 && args.size() != 3) {
    ROS_ERROR("Usage: trajectory_sampler_node <waypoint_file>"
        "\nThe waypoint file should be structured as: space separated: [m] y[m] z[m] yaw[deg])");
    return -1;
  }

  std::vector<Eigen::Vector4d> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;
  std::ifstream wp_file(args.at(1).c_str());
  if (wp_file.is_open()) {
    double x, y, z, yaw;
    // Only read complete waypoints.
    while (wp_file >> x >> y >> z >> yaw) {
      waypoints.push_back(Eigen::Vector4d(x, y, z, yaw * DEG_2_RAD));
    }
    wp_file.close();
    ROS_INFO("Read %d waypoints.", (int )waypoints.size());
  }

  else {
    ROS_ERROR_STREAM("Unable to open poses file: " << args.at(1));
    return -1;
  }
  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 4;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
 
  for (size_t i = 0; i < waypoints.size(); ++i) {
    
    if (i == 0){
    mav_trajectory_generation::Vertex start(dimension);
    start.makeStartOrEnd(waypoints[i], derivative_to_optimize);
    vertices.push_back(start);
    }
    else if (i == waypoints.size()-1) {
    mav_trajectory_generation::Vertex end(dimension);
    end.makeStartOrEnd(waypoints[i], derivative_to_optimize);
    vertices.push_back(end);
    }
    else {
    mav_trajectory_generation::Vertex middle(dimension);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints[i]);
    vertices.push_back(middle);
    }
  }

  std::vector<double> segment_times;
  const double v_max = 5.0;
  const double a_max = 8.0;
  const double magic_fabian_constant = 6.5; // A tuning parameter.
  segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();
  mav_trajectory_generation::Segment::Vector segments;
  opt.getSegments(&segments);
  mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);
  
  mav_msgs::EigenTrajectoryPoint::Vector states;
  double sampling_interval = 0.01;
  bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
  
  visualization_msgs::MarkerArray markers;
  double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  // From Trajectory class:
  mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

  // From mav_msgs::EigenTrajectoryPoint::Vector states:
  mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers);
  trajectory_msgs::MultiDOFJointTrajectory msg_pub;
  msgMultiDofJointTrajectoryFromEigen(states, &msg_pub);
  ROS_INFO("trajectory sampler.");
  command_pub.publish(msg_pub);


  ros::spin();
  return 0;
}
