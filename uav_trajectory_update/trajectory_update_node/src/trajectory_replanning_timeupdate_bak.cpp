
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

using namespace std; 

using namespace message_filters; 

const double v_max = 8.0;
const double a_max = 5.0;
const double magic_fabian_constant = 6.5; // A tuning parameter.
const int dimension = 4;
const int N = 10;
const float DEG_2_RAD = M_PI / 180.0;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
const float Threshold = 0.05;
    double start_x, start_y, start_z, start_yaw;
    double middle_x, middle_y, middle_z, middle_yaw;
    double stop_x, stop_y, stop_z, stop_yaw;
ros::Publisher command_pub;

void gasInformationCallback(const sensor_msgs::Concentration::ConstPtr& master_gas, const sensor_msgs::Concentration::ConstPtr& left_gas, const sensor_msgs::Concentration::ConstPtr& right_gas)
{
    //ROS_INFO("recieved gas information");

    double c1,c2,c3;
    c1 = master_gas->concentration;
    c2 = left_gas->concentration;
    c3 = right_gas->concentration;
    double x, y, d;
    if ((c2-c1)>=Threshold && (c1-c3)>=Threshold){
       d = abs(left_gas->position[1]-master_gas->position[1])*(0.5+1/(4*(c2-c1)+2));
       if (d < 2) y = left_gas->position[1]-2;
       else if (d > 10) y = left_gas->position[1]-10;
       else y = left_gas->position[1]-d;
   }
   else if ((c1-c2)>=Threshold && (c3-c1)>=Threshold){
       d = abs(right_gas->position[1]-master_gas->position[1])*(0.5+1/(4*(c2-c1)+2));
       if (d < 2) y = right_gas->position[1]+2;
       else if (d > 10) y = right_gas->position[1]+10;
       else y = right_gas->position[1]+d;
    }
    else if ((c1-c2)>=Threshold && (c1-c3)>=Threshold){
       y = master_gas->position[1];
    }
    else if ((c2-c1)>=Threshold && abs(c1-c3)<Threshold){              ////#4
       d = abs(left_gas->position[1]-master_gas->position[1])*(0.5+1/(4*(c2-c1)+2));
       if (d < 2) y = left_gas->position[1]-2;
       else if (d > 10) y = left_gas->position[1]-10;
       else y = left_gas->position[1]-d;
    }
    else if (abs(c1-c2)<Threshold && (c3-c1)>=Threshold){              ////#5
       d = abs(right_gas->position[1]-master_gas->position[1])*(0.5+1/(4*(c2-c1)+2));
       if (d < 2) y = right_gas->position[1]-2;
       else if (d > 10) y = right_gas->position[1]-10;
       else y = right_gas->position[1]-d;
    }
    else if (abs(c1-c2)<Threshold && (c1-c3)>=Threshold){              ////#6
       d = abs(right_gas->position[1]-master_gas->position[1])*(0.5+1/(4*(c1-c3)+2));
       if (d < 2) y = right_gas->position[1]+2;
       else if (d > 10) y = right_gas->position[1]+10;
       else y = right_gas->position[1]+d;
    }
    else if ((c1-c2)>=Threshold && abs(c1-c3)<Threshold){              ////#7
       d = abs(left_gas->position[1]-master_gas->position[1])*(0.5+1/(4*(c2-c1)+2));
       if (d < 2) y = left_gas->position[1]-2;
       else if (d > 10) y = left_gas->position[1]-10;
       else y = left_gas->position[1]-d;
    }
    else{
       y = (left_gas->position[1]-right_gas->position[1])*0.5;
    }
    x = master_gas->position[0]+1;
   
    std::vector<Eigen::Vector4d> waypoints;
    waypoints.push_back(Eigen::Vector4d(master_gas->position[0], master_gas->position[1], master_gas->position[2], master_gas->position[3] * DEG_2_RAD));
    waypoints.push_back(Eigen::Vector4d(x, y, master_gas->position[2], master_gas->position[3] * DEG_2_RAD));
    waypoints.push_back(Eigen::Vector4d(stop_x, y, stop_z, stop_yaw*DEG_2_RAD));
    mav_trajectory_generation::Vertex::Vector vertices;
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
    segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);
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
    trajectory_msgs::MultiDOFJointTrajectory msg_pub;
    msgMultiDofJointTrajectoryFromEigen(states, &msg_pub);
    command_pub.publish(msg_pub);

}



int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_replanning_timeupdate");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    

    string command_trajectory_topic_;
    string left_concentration_topic_;
    string middle_concentration_topic_;
    string right_concentration_topic_;
    pnh.param<string>("command_trajectory_topic", command_trajectory_topic_, "/firefly1/command/trajectory"); 
    pnh.param<string>("left_concentration_topic", left_concentration_topic_, "/firefly2/gas1_concentration");
    pnh.param<string>("middle_concentration_topic", middle_concentration_topic_, "/firefly1/gas1_concentration");
    pnh.param<string>("right_concentration_topic", right_concentration_topic_, "/firefly3/gas1_concentration");
    pnh.param("start_x", start_x, 0.0);
    pnh.param("start_y", start_y, 0.0);
    pnh.param("start_z", start_z, 3.0);
    pnh.param("start_yaw", start_yaw, 0.0);
    pnh.param("middle_x", middle_x, 0.0);
    pnh.param("middle_y", middle_y, 0.0);
    pnh.param("middle_z", middle_z, 3.0);
    pnh.param("middle_yaw", middle_yaw, 0.0);
    pnh.param("stop_x", stop_x, 0.0);
    pnh.param("stop_y", stop_y, 0.0);
    pnh.param("stop_z", stop_z, 3.0);
    pnh.param("stop_yaw", stop_yaw, 0.0);

    command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(command_trajectory_topic_, 10);
  
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_1(nh, middle_concentration_topic_, 100);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_2(nh, left_concentration_topic_, 100);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_3(nh, right_concentration_topic_, 100);
    TimeSynchronizer<sensor_msgs::Concentration, sensor_msgs::Concentration, sensor_msgs::Concentration> sync(concentration_sub_1, concentration_sub_2, concentration_sub_3,10);
    sync.registerCallback(boost::bind(&gasInformationCallback, _1, _2, _3));


    std::vector<Eigen::Vector4d> waypoints;
    waypoints.push_back(Eigen::Vector4d(start_x, start_y, start_z, start_yaw * DEG_2_RAD));
    waypoints.push_back(Eigen::Vector4d(middle_x, middle_y, middle_z, middle_yaw * DEG_2_RAD));
    waypoints.push_back(Eigen::Vector4d(stop_x, stop_y, stop_z, stop_yaw * DEG_2_RAD));

    mav_trajectory_generation::Vertex::Vector vertices;
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
  
    segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);
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
    trajectory_msgs::MultiDOFJointTrajectory msg_pub;
    msgMultiDofJointTrajectoryFromEigen(states, &msg_pub);
    command_pub.publish(msg_pub);
    ros::Duration(5.0).sleep();

    //ros::Timer publish_timer = nh.createTimer(ros::Duration(0.5), sendCommandCallback);
    ros::spin();
    return 0;

}
