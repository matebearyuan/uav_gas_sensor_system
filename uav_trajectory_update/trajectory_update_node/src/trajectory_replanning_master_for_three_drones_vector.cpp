
#include <mav_trajectory_generation_ros/trajectory_replanning_master_for_three_drones_vector.h>

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
      current_sample_time_1(0.0){
    string firefly1_command_trajectory_topic_;
    string firefly2_command_trajectory_topic_;
    string firefly3_command_trajectory_topic_;
    string firefly1_concentration_topic_;
    string firefly2_concentration_topic_;
    string firefly3_concentration_topic_;
    pnh_.param<string>("firefly1_command_trajectory_topic", firefly1_command_trajectory_topic_, "/firefly1/command/trajectory");
    pnh_.param<string>("firefly2_command_trajectory_topic", firefly2_command_trajectory_topic_, "/firefly2/command/trajectory");
    pnh_.param<string>("firefly3_command_trajectory_topic", firefly3_command_trajectory_topic_, "/firefly3/command/trajectory"); 
    pnh_.param<string>("left_concentration_topic", firefly2_concentration_topic_, "/firefly2/gas1_concentration");
    pnh_.param<string>("middle_concentration_topic", firefly1_concentration_topic_, "/firefly1/gas1_concentration");
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

    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_1(nh_, firefly1_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_2(nh_, firefly2_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_3(nh_, firefly3_concentration_topic_, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Concentration, sensor_msgs::Concentration, sensor_msgs::Concentration> sync(concentration_sub_1, concentration_sub_2, concentration_sub_3, 10);
    sync.registerCallback(boost::bind(&TrajectoryReplanningMasterForThreedrones::gasInformationCallback, this, _1, _2, _3));
    ros::spin();
}

TrajectoryReplanningMasterForThreedrones::~TrajectoryReplanningMasterForThreedrones(){
   publish_timer_1_.stop();

}
/////////////////////////////
bool TrajectoryReplanningMasterForThreedrones::stopSamplingCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  publish_timer_1_.stop();

  return true;
}
/////////////////////////////
void TrajectoryReplanningMasterForThreedrones::commandTimerCallback1(const ros::TimerEvent&) {
  if (current_sample_time_1 <= trajectory_1_.getMaxTime() | current_sample_time_1 <= trajectory_2_.getMaxTime() | current_sample_time_1 <= trajectory_3_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg1, msg2, msg3;
    mav_msgs::EigenTrajectoryPoint flat_state1, flat_state2, flat_state3;
    bool success1 = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_1_, current_sample_time_1, &flat_state1);
    bool success2 = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_2_, current_sample_time_1, &flat_state2);
    bool success3 = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_3_, current_sample_time_1, &flat_state3);
    if (!success1 | !success2 | !success3) {
      publish_timer_1_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state1, &msg1);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state2, &msg2);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state3, &msg3);
    msg1.points[0].time_from_start = ros::Duration(current_sample_time_1);
    msg2.points[0].time_from_start = ros::Duration(current_sample_time_1);
    msg3.points[0].time_from_start = ros::Duration(current_sample_time_1);
    command_pub_1_.publish(msg1);
    command_pub_2_.publish(msg2);
    command_pub_3_.publish(msg3);
    current_sample_time_1 += dt_;
  } else {
    publish_timer_1_.stop();
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
    int i, j, num, max_id;
    concentration_[0] = firefly1_gas->concentration;
    concentration_[1] = firefly2_gas->concentration;
    concentration_[2] = firefly3_gas->concentration;
    position_[0] = firefly1_gas->position[1];
    position_[1] = firefly2_gas->position[1];
    position_[2] = firefly3_gas->position[1];
    num = 0;
    double x[3];
    double y[3];
    double max =0;
      for(i=0;i<3;i++){
       if(concentration_[i]>Threshold) num=num+1;
       if(concentration_[i]>max){
          max = concentration_[i];
          max_id = i;
       }
    }
    if(num>2){
       MatrixXf A(3,3);
       A=MatrixXf::Ones(3,3);
       MatrixXf B(3,1);
       MatrixXf C(3,1);
       B=MatrixXf::Ones(3,1);
       C=MatrixXf::Ones(3,1);
       double miu, sigma;

       for(i=0;i<3;i++) C(i,0)=log(concentration_[i]);
       for(i=0;i<3;i++){
          for(j=0;j<3;j++){
             if(j=0)  A(i,j) = 1.0;
             if(j=1)  A(i,j) = position_[i];
             if(j=2)  A(i,j) = position_[i]*position_[i];
          }
       }
       MatrixXf D(3,3);
       D=MatrixXf::Ones(3,3);
       MatrixXf F(3,3);
       F=MatrixXf::Ones(3,3);
       ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"A ="<<A<<".");
       ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"C ="<<C<<".");
       D = A.transpose()*A;

       ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"D ="<<D<<".");
       F = D.inverse()*A.transpose();
       B = F*C;
       ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"F ="<<F<<".");
       ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"B ="<<B<<".");
       miu = -B(1,0)/(2*B(2,0));
       sigma = sqrt(-1/B(2,0));
       ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"miu ="<<miu<<"sigma="<<sigma<<".");
       if(isnan(miu) | isnan(sigma) ){
           double d1 = (position_[0]-position_[1])*(0.5+1/(4*abs(concentration_[0]-concentration_[1])+2));
           double d2 = (position_[1]-position_[2])*(0.5+1/(4*abs(concentration_[0]-concentration_[1])+2));
           if (d1 < 2) d1 = 2;
           else if (d1 > 10) d1 = 10;
           if (d2 < 2) d2 = 2;
           else if (d2 > 10) d2 = 10;
           y[1] = position_[max_id];
           y[0] = y[1] + d1;
           y[2] = y[1] - d2;
       }else if(sigma>20){
           y[0] = miu+0.5*20;
           y[1] = miu;
           y[2] = miu-0.5*20;

       }else{
           y[0] = miu+0.5*sigma;
           y[1] = miu;
           y[2] = miu-0.5*sigma;

       } 
    }else if(num<3 && num>0 ){
       double d1 = (position_[0]-position_[1])*(0.5+1/(4*abs(concentration_[0]-concentration_[1])+2));
       double d2 = (position_[1]-position_[2])*(0.5+1/(4*abs(concentration_[0]-concentration_[1])+2));
       if (d1 < 2) d1 = 2;
       else if (d1 > 10) d1 = 10;
       if (d2 < 2) d2 = 2;
       else if (d2 > 10) d2 = 10;
       y[1] = position_[max_id];
       y[0] = y[1] + d1;
       y[2] = y[1] - d2;
   }
   else if (num = 0){
       y[0] = firefly1_gas->position[1];
       y[1] = firefly2_gas->position[1];
       y[2] = firefly3_gas->position[1];

   }
   /*
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
    } */
   x[0] = firefly1_gas->position[0]+0.2;
   x[1] = firefly2_gas->position[0]+0.2;
   x[2] = firefly3_gas->position[0]+0.2;


    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"x1 ="<<x[0]<<", y1 ="<<y[0]<<"; "<<"x2= "<<x[1]<<", y2= "<<y[1]<<"; "<<"x3= "<<x[2]<<", y3= "<<y[2]<<".");
    if (x[0]<stop_x | x[1]<stop_x | x[2]<stop_x){
          TrajectoryReplanningMasterForThreedrones::WaypointUpdate(x[0],  y[0], firefly1_gas, trajectory_1_);
          TrajectoryReplanningMasterForThreedrones::WaypointUpdate(x[1],  y[1], firefly2_gas, trajectory_2_);
          TrajectoryReplanningMasterForThreedrones::WaypointUpdate(x[2],  y[2], firefly3_gas, trajectory_3_);
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

              current_sample_time_1 = 0;

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
