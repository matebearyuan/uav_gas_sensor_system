
#include <mav_trajectory_generation_ros/trajectory_replanning_timeupdate.h>

using namespace std; 

using namespace message_filters; 

const double v_max = 1.0;
const double a_max = 5.0;
const double magic_fabian_constant = 6.5; // A tuning parameter.
const int dimension = 4;
const int N = 10;
const float DEG_2_RAD = M_PI / 180.0;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
const float Threshold = 0.05;


TrajectoryReplanningTimeupdate::TrajectoryReplanningTimeupdate(const ros::NodeHandle& nh,
                                                               const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      start_x(0.0),
      start_y(0.0), 
      start_z(3.0), 
      middle_x(0.0), 
      middle_y(0.0), 
      middle_z(3.0), 
      stop_x(10.0), 
      stop_y(0.0), 
      stop_z(3.0),
      publish_whole_trajectory_(false),
      dt_(0.01),
      current_sample_time_(0.0){
    string command_trajectory_topic_;
    string left_concentration_topic_;
    string middle_concentration_topic_;
    string right_concentration_topic_;
    pnh_.param<string>("command_trajectory_topic", command_trajectory_topic_, "/firefly1/command/trajectory"); 
    pnh_.param<string>("left_concentration_topic", left_concentration_topic_, "/firefly2/gas1_concentration");
    pnh_.param<string>("middle_concentration_topic", middle_concentration_topic_, "/firefly1/gas1_concentration");
    pnh_.param<string>("right_concentration_topic", right_concentration_topic_, "/firefly3/gas1_concentration");
    pnh_.param("start_x", start_x, 0.0);
    pnh_.param("start_y", start_y, 0.0);
    pnh_.param("start_z", start_z, 3.0);
    pnh_.param("start_yaw", start_yaw, 0.0);
    pnh_.param("middle_x", middle_x, 0.0);
    pnh_.param("middle_y", middle_y, 0.0);
    pnh_.param("middle_z", middle_z, 3.0);
    pnh_.param("middle_yaw", middle_yaw, 0.0);
    pnh_.param("stop_x", stop_x, 0.0);
    pnh_.param("stop_y", stop_y, 0.0);
    pnh_.param("stop_z", stop_z, 3.0);
    pnh_.param("stop_yaw", stop_yaw, 0.0);
    pnh_.param("publish_whole_trajectory", publish_whole_trajectory_,  publish_whole_trajectory_);
    pnh_.param("dt", dt_, dt_);
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"stop_x = "<<stop_x<<", stop_z = "<<stop_z);
    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(command_trajectory_topic_, 1);
    stop_srv_ = nh_.advertiseService("stop_sampling", &TrajectoryReplanningTimeupdate::stopSamplingCallback, this);
    publish_timer_ = nh_.createTimer(ros::Duration(dt_), &TrajectoryReplanningTimeupdate::commandTimerCallback, this, false);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_1(nh_, middle_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_2(nh_, left_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_3(nh_, right_concentration_topic_, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Concentration, sensor_msgs::Concentration, sensor_msgs::Concentration> sync(concentration_sub_1, concentration_sub_2, concentration_sub_3, 10);
    sync.registerCallback(boost::bind(&TrajectoryReplanningTimeupdate::gasInformationCallback, this, _1, _2, _3));
    ros::spin();
}

TrajectoryReplanningTimeupdate::~TrajectoryReplanningTimeupdate(){ publish_timer_.stop();}
/////////////////////////////
bool TrajectoryReplanningTimeupdate::stopSamplingCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  publish_timer_.stop();
  return true;
}
/////////////////////////////
void TrajectoryReplanningTimeupdate::commandTimerCallback(const ros::TimerEvent&) {
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

/////////////////////////////
void TrajectoryReplanningTimeupdate::gasInformationCallback(const sensor_msgs::Concentration::ConstPtr& master_gas, const sensor_msgs::Concentration::ConstPtr& left_gas, const sensor_msgs::Concentration::ConstPtr& right_gas)
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
       if (d < 2) y = right_gas->position[1]+2;
       else if (d > 10) y = right_gas->position[1]+10;
       else y = right_gas->position[1]+d;
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
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"x = "<<x<<", y = "<<y);
    std::vector<Eigen::Vector4d> waypoints;
    //waypoints.push_back(Eigen::Vector4d(master_gas->position[0], master_gas->position[1], master_gas->position[2], master_gas->position[3] * DEG_2_RAD));
    waypoints.push_back(Eigen::Vector4d(x, y, master_gas->position[2], master_gas->position[3] * DEG_2_RAD));
    waypoints.push_back(Eigen::Vector4d(stop_x, y, stop_z, stop_yaw*DEG_2_RAD));
    mav_trajectory_generation::Vertex::Vector vertices;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (i == 0){
         mav_trajectory_generation::Vertex start(dimension);
         start.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints[i]);//start.makeStartOrEnd(waypoints[i], derivative_to_optimize);
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
    opt.getTrajectory(&trajectory_);
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



int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_replanning_timeupdate");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
    ros::Duration(30).sleep();
    TrajectoryReplanningTimeupdate trajectory_replanning_timeupdate(nh, pnh);
    ROS_INFO("Initialized trajectory replanning node.");


}
