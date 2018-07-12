
#include <mav_trajectory_generation_ros/trajectory_replanning_master_for_five_drones_vector.h>

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


TrajectoryReplanningMasterForFivedrones::TrajectoryReplanningMasterForFivedrones(const ros::NodeHandle& nh,
                                                               const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      stop_x(10.0), 
      stop_y(0.0), 
      stop_z(3.0),
      publish_whole_trajectory_(false),
      end_searching_(false),
      finding_(true),
      dt_(0.01),
      current_sample_time_(0.0){
    string firefly1_command_trajectory_topic_;
    string firefly2_command_trajectory_topic_;
    string firefly3_command_trajectory_topic_;
    string firefly4_command_trajectory_topic_;
    string firefly5_command_trajectory_topic_;
    string firefly1_concentration_topic_;
    string firefly2_concentration_topic_;
    string firefly3_concentration_topic_;
    string firefly4_concentration_topic_;
    string firefly5_concentration_topic_;
    pnh_.param<string>("firefly1_command_trajectory_topic", firefly1_command_trajectory_topic_, "/firefly1/command/trajectory");
    pnh_.param<string>("firefly2_command_trajectory_topic", firefly2_command_trajectory_topic_, "/firefly2/command/trajectory");
    pnh_.param<string>("firefly3_command_trajectory_topic", firefly3_command_trajectory_topic_, "/firefly3/command/trajectory"); 
    pnh_.param<string>("firefly4_command_trajectory_topic", firefly4_command_trajectory_topic_, "/firefly4/command/trajectory"); 
    pnh_.param<string>("firefly5_command_trajectory_topic", firefly5_command_trajectory_topic_, "/firefly5/command/trajectory"); 
    pnh_.param<string>("firefly2_concentration_topic", firefly2_concentration_topic_, "/firefly2/gas1_concentration");
    pnh_.param<string>("firefly1_concentration_topic", firefly1_concentration_topic_, "/firefly1/gas1_concentration");
    pnh_.param<string>("firefly3_concentration_topic", firefly3_concentration_topic_, "/firefly3/gas1_concentration");
    pnh_.param<string>("firefly4_concentration_topic", firefly4_concentration_topic_, "/firefly4/gas1_concentration");
    pnh_.param<string>("firefly5_concentration_topic", firefly5_concentration_topic_, "/firefly5/gas1_concentration");
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
    command_pub_4_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(firefly4_command_trajectory_topic_, 1);
    command_pub_5_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(firefly5_command_trajectory_topic_, 1);
    stop_srv_ = nh_.advertiseService("stop_sampling", &TrajectoryReplanningMasterForFivedrones::stopSamplingCallback, this);
    publish_timer_ = nh_.createTimer(ros::Duration(dt_), &TrajectoryReplanningMasterForFivedrones::commandTimerCallback, this, false);

    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_1(nh_, firefly1_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_2(nh_, firefly2_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_3(nh_, firefly3_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_4(nh_, firefly4_concentration_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Concentration> concentration_sub_5(nh_, firefly5_concentration_topic_, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Concentration, sensor_msgs::Concentration, sensor_msgs::Concentration, sensor_msgs::Concentration, sensor_msgs::Concentration> sync(concentration_sub_1, concentration_sub_2, concentration_sub_3, concentration_sub_4, concentration_sub_5, 10);
    sync.registerCallback(boost::bind(&TrajectoryReplanningMasterForFivedrones::gasInformationCallback, this, _1, _2, _3, _4, _5));
    ros::spin();
}

TrajectoryReplanningMasterForFivedrones::~TrajectoryReplanningMasterForFivedrones(){
   publish_timer_.stop();

}
/////////////////////////////
bool TrajectoryReplanningMasterForFivedrones::stopSamplingCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  publish_timer_.stop();

  return true;
}
/////////////////////////////
void TrajectoryReplanningMasterForFivedrones::commandTimerCallback(const ros::TimerEvent&) {
  if (current_sample_time_ <= trajectory_1_.getMaxTime() | current_sample_time_ <= trajectory_2_.getMaxTime() | current_sample_time_ <= trajectory_3_.getMaxTime() | current_sample_time_ <= trajectory_4_.getMaxTime() | current_sample_time_ <= trajectory_5_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg1, msg2, msg3, msg4, msg5;
    mav_msgs::EigenTrajectoryPoint flat_state1, flat_state2, flat_state3, flat_state4, flat_state5;
    bool success1 = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_1_, current_sample_time_, &flat_state1);
    bool success2 = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_2_, current_sample_time_, &flat_state2);
    bool success3 = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_3_, current_sample_time_, &flat_state3);
    bool success4 = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_4_, current_sample_time_, &flat_state4);
    bool success5 = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_5_, current_sample_time_, &flat_state5);
    if (!success1 | !success2 | !success3 | !success4 | !success5) {
      publish_timer_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state1, &msg1);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state2, &msg2);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state3, &msg3);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state4, &msg4);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state5, &msg5);
    msg1.points[0].time_from_start = ros::Duration(current_sample_time_);
    msg2.points[0].time_from_start = ros::Duration(current_sample_time_);
    msg3.points[0].time_from_start = ros::Duration(current_sample_time_);
    msg4.points[0].time_from_start = ros::Duration(current_sample_time_);
    msg5.points[0].time_from_start = ros::Duration(current_sample_time_);
    command_pub_1_.publish(msg1);
    command_pub_2_.publish(msg2);
    command_pub_3_.publish(msg3);
    command_pub_4_.publish(msg4);
    command_pub_5_.publish(msg5);
    current_sample_time_ += dt_;
  } else {
    publish_timer_.stop();
  }
}

/////////////////////////////
void TrajectoryReplanningMasterForFivedrones::WaypointUpdate(const double x, const double y, const sensor_msgs::Concentration::ConstPtr& firefly_gas, mav_trajectory_generation::Trajectory& trajectory){
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
/////////////////////////////
void TrajectoryReplanningMasterForFivedrones::FindTarget(const double x, const double y, const double end_x, const double end_y, mav_trajectory_generation::Trajectory& trajectory){
          std::vector<Eigen::Vector4d> waypoints;
          waypoints.push_back(Eigen::Vector4d(x, y, stop_z, stop_yaw * DEG_2_RAD));
          waypoints.push_back(Eigen::Vector4d(end_x, end_y, stop_z, stop_yaw*DEG_2_RAD));
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
////////////////////////////////////////////////////////////////
void TrajectoryReplanningMasterForFivedrones::gasInformationCallback(const sensor_msgs::Concentration::ConstPtr& firefly1_gas, const sensor_msgs::Concentration::ConstPtr& firefly2_gas, const sensor_msgs::Concentration::ConstPtr& firefly3_gas, const sensor_msgs::Concentration::ConstPtr& firefly4_gas, const sensor_msgs::Concentration::ConstPtr& firefly5_gas)
{
    //ROS_INFO("recieved gas information");

    double concentration_[5];
    double position_[5];
    int i, j, num, max_id, min_id;
    concentration_[0] = firefly1_gas->concentration;
    concentration_[1] = firefly2_gas->concentration;
    concentration_[2] = firefly3_gas->concentration;
    concentration_[3] = firefly4_gas->concentration;
    concentration_[4] = firefly5_gas->concentration;
    position_[0] = firefly1_gas->position[1];
    position_[1] = firefly2_gas->position[1];
    position_[2] = firefly3_gas->position[1];
    position_[3] = firefly4_gas->position[1];
    position_[4] = firefly5_gas->position[1];
    num = 0;
    double x[5];
    double y[5];
    double max =0;
    double min =0;
    for(i=0;i<5;i++){
       if(concentration_[i]>Threshold) num=num+1;
       if(concentration_[i]>max){
          max = concentration_[i];
          max_id = i;
       }
       if(concentration_[i]<min){
          min = concentration_[i];
          min_id = i;
       }
    }
    if(num>2){
       MatrixXf A(5,3);
       A=MatrixXf::Ones(5,3);
       MatrixXf B(3,1);
       MatrixXf C(5,1);
       B=MatrixXf::Ones(3,1);
       C=MatrixXf::Ones(5,1);
       double miu, sigma;

       for(i=0;i<5;i++) C(i,0)=log(concentration_[i]);
       for(i=0;i<5;i++){
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
           double d = (position_[max_id]-position_[min_id])*(0.5+1/(4*abs(concentration_[max_id]-concentration_[min_id])+2));
           if (d < 8) d = 8;
           else if (d > 40) d = 40;
           for(i=0;i<5;i++) y[i] = position_[max_id]+(1/2-double(i)/4)*d;

       }else if(sigma>20){
           for(i=0;i<5;i++) y[i] = miu+(1/2-double(i)/4)*40;

       }else if(sigma<4){
           for(i=0;i<5;i++) y[i] = miu+(1/2-double(i)/4)*8;
       }else{
           for(i=0;i<5;i++) y[i] = miu+(1-double(i)/2)*sigma;
       }
    }else if(num<3 && num>0 ){
       double d = (position_[max_id]-position_[min_id])*(0.5+1/(4*abs(concentration_[max_id]-concentration_[min_id])+2));
       if (d < 8) d = 8;
       else if (d > 40) d = 40;
       for(i=0;i<5;i++) y[i] = position_[max_id]+(1/2-double(i)/4)*d;
       ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"num ="<<num<<"d="<<d<<".");
   }
   else if(num == 0){
       for(i=0;i<5;i++) y[i] = position_[2]+(position_[i]-position_[2])*1.08;
       ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"num ="<<num<<".");

   }
   x[0] = firefly1_gas->position[0]+0.2;
   x[1] = firefly2_gas->position[0]+0.2;
   x[2] = firefly3_gas->position[0]+0.2;
   x[3] = firefly4_gas->position[0]+0.2;
   x[4] = firefly5_gas->position[0]+0.2;

    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<"x1 ="<<x[0]<<", y1 ="<<y[0]<<"; "<<"x2= "<<x[1]<<", y2= "<<y[1]<<"; "<<"x3= "<<x[2]<<", y3= "<<y[2]<<".");
    if (x[0]<stop_x && x[1]<stop_x && x[2]<stop_x && x[3]<stop_x && x[4]<stop_x && !end_searching_){
          TrajectoryReplanningMasterForFivedrones::WaypointUpdate(x[0],  y[0], firefly1_gas, trajectory_1_);
          TrajectoryReplanningMasterForFivedrones::WaypointUpdate(x[1],  y[1], firefly2_gas, trajectory_2_);
          TrajectoryReplanningMasterForFivedrones::WaypointUpdate(x[2],  y[2], firefly3_gas, trajectory_3_);
          TrajectoryReplanningMasterForFivedrones::WaypointUpdate(x[3],  y[3], firefly4_gas, trajectory_4_);
          TrajectoryReplanningMasterForFivedrones::WaypointUpdate(x[4],  y[4], firefly5_gas, trajectory_5_);
          if (publish_whole_trajectory_) {
    // Publish the entire trajectory at once.
              mav_msgs::EigenTrajectoryPoint::Vector flat_states1, flat_states2, flat_states3, flat_states4, flat_states5;
              mav_trajectory_generation::sampleWholeTrajectory(trajectory_1_, dt_,
                                                     &flat_states1);
              mav_trajectory_generation::sampleWholeTrajectory(trajectory_2_, dt_,
                                                     &flat_states2);
              mav_trajectory_generation::sampleWholeTrajectory(trajectory_3_, dt_,
                                                     &flat_states3);
              mav_trajectory_generation::sampleWholeTrajectory(trajectory_4_, dt_,
                                                     &flat_states4);
              mav_trajectory_generation::sampleWholeTrajectory(trajectory_5_, dt_,
                                                     &flat_states5);
              trajectory_msgs::MultiDOFJointTrajectory msg_pub_1, msg_pub_2, msg_pub_3, msg_pub_4, msg_pub_5;
              msgMultiDofJointTrajectoryFromEigen(flat_states1, &msg_pub_1);
              msgMultiDofJointTrajectoryFromEigen(flat_states2, &msg_pub_2);
              msgMultiDofJointTrajectoryFromEigen(flat_states3, &msg_pub_3);
              msgMultiDofJointTrajectoryFromEigen(flat_states4, &msg_pub_4);
              msgMultiDofJointTrajectoryFromEigen(flat_states5, &msg_pub_5);
              command_pub_1_.publish(msg_pub_1);
              command_pub_2_.publish(msg_pub_2);
              command_pub_3_.publish(msg_pub_3);
              command_pub_4_.publish(msg_pub_4);
              command_pub_5_.publish(msg_pub_5);
          } else {
              publish_timer_.start();

              current_sample_time_ = 0;

              start_time_ = ros::Time::now();
         }
    }else{
         publish_timer_.stop();
         end_searching_=true;
         if(finding_){
         TrajectoryReplanningMasterForFivedrones::FindTarget(x[0],  y[0], 0, 9, trajectory_1_);
         TrajectoryReplanningMasterForFivedrones::FindTarget(x[1],  y[1], 0, 5, trajectory_2_);
         TrajectoryReplanningMasterForFivedrones::FindTarget(x[2],  y[2], -6.5, 5, trajectory_3_);
         TrajectoryReplanningMasterForFivedrones::FindTarget(x[3],  y[3], 6.5, 5, trajectory_4_);
         TrajectoryReplanningMasterForFivedrones::FindTarget(x[4],  y[4], 0, 1, trajectory_5_);
         mav_msgs::EigenTrajectoryPoint::Vector flat_states1, flat_states2, flat_states3, flat_states4, flat_states5;
         mav_trajectory_generation::sampleWholeTrajectory(trajectory_1_, dt_,
                                                     &flat_states1);
         mav_trajectory_generation::sampleWholeTrajectory(trajectory_2_, dt_,
                                                     &flat_states2);
         mav_trajectory_generation::sampleWholeTrajectory(trajectory_3_, dt_,
                                                     &flat_states3);
         mav_trajectory_generation::sampleWholeTrajectory(trajectory_4_, dt_,
                                                     &flat_states4);
         mav_trajectory_generation::sampleWholeTrajectory(trajectory_5_, dt_,
                                                     &flat_states5);
         trajectory_msgs::MultiDOFJointTrajectory msg_pub_1, msg_pub_2, msg_pub_3, msg_pub_4, msg_pub_5;
         msgMultiDofJointTrajectoryFromEigen(flat_states1, &msg_pub_1);
         msgMultiDofJointTrajectoryFromEigen(flat_states2, &msg_pub_2);
         msgMultiDofJointTrajectoryFromEigen(flat_states3, &msg_pub_3);
         msgMultiDofJointTrajectoryFromEigen(flat_states4, &msg_pub_4);
         msgMultiDofJointTrajectoryFromEigen(flat_states5, &msg_pub_5);
         command_pub_1_.publish(msg_pub_1);
         command_pub_2_.publish(msg_pub_2);
         command_pub_3_.publish(msg_pub_3);
         command_pub_4_.publish(msg_pub_4);
         command_pub_5_.publish(msg_pub_5);
         finding_=false;
         }
    }

}



int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_replanning_master_for_five_drones");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
    ros::Duration(30).sleep();
    TrajectoryReplanningMasterForFivedrones trajectory_replanning_master_for_five_drones(nh, pnh);
    ROS_INFO("Initialized trajectory replanning node.");


}
