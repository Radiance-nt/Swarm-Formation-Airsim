#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <deque>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
double ctrl_freq;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

Eigen::Vector3d last_vel(Eigen::Vector3d::Zero()), last_acc(Eigen::Vector3d::Zero()), last_jerk(Eigen::Vector3d::Zero());
bool flag = false;
double jerk2_inter = 0, acc2_inter = 0;
int cnt = 0;
ros::Time global_start_time;
int drone_id_;
std::string result_dir = "/home/zuzu/Documents/report/";
std::fstream result_file;
std::vector<Eigen::Vector3d> pos_vec_, vel_vec_, acc_vec_, jerk_vec_;
std::vector<double> time_vec_;
Eigen::Vector3d odom_pos_vec_, odom_vel_vec_, odom_acc_vec_;
ros::Time odom_timestamp, prev_odom_timestamp=ros::Time(0);
Eigen::Vector3d prev_odom_vel_vec_;
bool has_odom_ = false;
double kp_pos[3] = {0.0, 0.0, 0.0};
double kd_pos[3] = {0.0, 0.0, 0.0};
double ki_pos[3] = {0.0, 0.0, 0.0};

std::deque<Eigen::Vector3d> integral_pos_buffer;
int window_size = 10;
Eigen::Vector3d integral_pos_max(Eigen::Vector3d::Ones());

Eigen::Vector3d positionPIDControl(const Eigen::Vector3d& desired_pos, const Eigen::Vector3d& current_pos, const Eigen::Vector3d& current_vel, double dt) {
  Eigen::Vector3d error_pos = desired_pos - current_pos;
  integral_pos_buffer.push_back(error_pos);
  if (integral_pos_buffer.size() > window_size) {
    integral_pos_buffer.pop_front();
  }
  Eigen::Vector3d integral_pos = Eigen::Vector3d::Zero();
  for (const auto& error : integral_pos_buffer) {
    integral_pos += error;
  }
  for (int i = 0; i < 3; ++i) {
    integral_pos[i] = std::max(-integral_pos_max[i], std::min(integral_pos[i], integral_pos_max[i]));
  }
  Eigen::Vector3d control_signal_pos;
  for (int i = 0; i < 3; ++i) {
    control_signal_pos[i] = kp_pos[i] * error_pos[i] + ki_pos[i] * integral_pos[i] - kd_pos[i] * current_vel[i];
    // control_signal_pos[i] = kp_pos[i] * error_pos[i] - kd_pos[i] * current_vel[i];
  }
  // ROS_INFO_STREAM("error_pos: " << error_pos.transpose());
  // ROS_INFO_STREAM("integral_pos: " << integral_pos.transpose());
  // ROS_INFO_STREAM("current_vel: " << current_vel.transpose());
  return control_signal_pos;
}

const std::vector<std::string> explode(const std::string& s, const char& c)
{
  std::string buff{""};
  std::vector<std::string> v;
  
  for(auto n:s)
  {
    if(n != c) buff+=n; else
    if(n == c && buff != "") { v.push_back(buff); buff = ""; }
  }
  if(buff != "") v.push_back(buff);
  
  return v;
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

void finishCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {
    ROS_WARN("total_time, cnt, acc2_inter, jerk2_inter = %lf \t %d \t %lf \t %lf", (ros::Time::now() - global_start_time).toSec(), cnt, acc2_inter, jerk2_inter);
    // ROS_WARN("time.size = %d, %d, %d, %d", time_vec_.size(), vel_vec_.size(), acc_vec_.size(), jerk_vec_.size());
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //转为字符串
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
    result_file << str_time << "\n";
    double max_vel2 = 0;
    for (int i = 0; i < time_vec_.size(); i++) {
      double tmp_vel2 = (vel_vec_[i](0))*(vel_vec_[i](0)) + (vel_vec_[i](1))*(vel_vec_[i](1)) + (vel_vec_[i](2))*(vel_vec_[i](2));
      max_vel2 = (tmp_vel2 > max_vel2) ? tmp_vel2 : max_vel2; 
    }
    result_file << "max_vel = " << sqrt(max_vel2) << "\n";
    result_file << "\n"; 
  }
}

void startCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {
    ROS_WARN("START!!!!");
    global_start_time = ros::Time::now();
  }
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();


  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (flag == false) {
    flag = true;
  } else {
    cnt++;
    acc2_inter += last_acc.norm()*last_acc.norm()*(time_now-time_last).toSec();
    jerk2_inter += last_jerk.norm()*last_jerk.norm()*(time_now-time_last).toSec();
    
  }
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jerk = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_->getPos(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    // std::cout << "[Traj server]: invalid time." << std::endl;
  }
  Eigen::Vector3d vel_control_signal(Eigen::Vector3d::Zero());
  if (has_odom_){
    double get_pos_timestamp_ = std::min((odom_timestamp - start_time_).toSec(), traj_duration_);
    vel_control_signal = positionPIDControl(traj_->getPos(get_pos_timestamp_), odom_pos_vec_, odom_vel_vec_, ctrl_freq);
    vel += vel_control_signal;
  }
  time_last = time_now;
  time_vec_.push_back((ros::Time::now() - global_start_time).toSec());
  pos_vec_.push_back(pos);
  vel_vec_.push_back(vel);
  acc_vec_.push_back(acc);
  jerk_vec_.push_back(jerk);
  last_vel = vel;
  last_acc = acc;
  last_jerk = jerk;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
}

void odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  // std::cout<< "odom received" << std::endl;
  has_odom_ = true;
  odom_timestamp = odom->header.stamp;
  odom_pos_vec_(0) = odom->pose.pose.position.x;
  odom_pos_vec_(1) = odom->pose.pose.position.y;
  odom_pos_vec_(2) = odom->pose.pose.position.z;
  odom_vel_vec_(0) = odom->twist.twist.linear.x;
  odom_vel_vec_(1) = odom->twist.twist.linear.y;
  odom_vel_vec_(2) = odom->twist.twist.linear.z;

  odom_acc_vec_ = Eigen::Vector3d(0,0,0);
  if (prev_odom_timestamp.toSec() != 0) {
    double dt = (odom_timestamp - prev_odom_timestamp).toSec();
    if (dt > 0) {
      odom_acc_vec_ = (odom_vel_vec_ - prev_odom_vel_vec_) / dt;
    }
  }

  prev_odom_vel_vec_ = odom_vel_vec_;
  prev_odom_timestamp = odom_timestamp;

  if (receive_traj_){   
    traj_->odom_pos_vec_ = odom_pos_vec_;
    traj_->odom_vel_vec_ = odom_vel_vec_;
    traj_->odom_acc_vec_ = odom_acc_vec_;
  }
}


void updateParametersCallback(const ros::TimerEvent &e)
{
  for (int i = 0; i < 3; ++i) {
    double kp, kd, ki, integral_max;
    if (ros::param::get("/kp_pos_" + std::to_string(i), kp)) {
      kp_pos[i] = kp;
    }
    if (ros::param::get("/kd_pos_" + std::to_string(i), kd)) {
      kd_pos[i] = kd;
    }
    if (ros::param::get("/ki_pos_" + std::to_string(i), ki)) {
      ki_pos[i] = ki;
    }
    if (ros::param::get("/integral_pos_max_" + std::to_string(i), integral_max)) {
      integral_pos_max[i] = integral_max;
    }
    if (integral_pos_max[i] < 0) {
      integral_pos_max[i] = integral_pos_max[i] * -1;
    }
  }
  // ROS_INFO("kp_pos: [%f, %f, %f]", kp_pos[0], kp_pos[1], kp_pos[2]);
  // ROS_INFO("kd_pos: [%f, %f, %f]", kd_pos[0], kd_pos[1], kd_pos[2]);
  // ROS_INFO("ki_pos: [%f, %f, %f]", ki_pos[0], ki_pos[1], ki_pos[2]);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");
  // get drone num
  std::string name_drone = ros::this_node::getName();
  std::vector<std::string> v{explode(name_drone, '_')};
  // drone_id_ = std::stoi(v[1]);
  result_file.open(result_dir+v[1]+"_vaj.txt", std::ios::out);

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber reached_sub = nh.subscribe("planning/finish", 10, finishCallback);
  ros::Subscriber start_sub = nh.subscribe("planning/start", 10, startCallback);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  nh.param("traj_server/ctrl_freq", ctrl_freq, 0.05);
  std::string odom_topic_name = "/drone_" + v[1] + "_visual_slam/odom";
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_name, 10, odomCallback);
  ros::Timer cmd_timer = nh.createTimer(ros::Duration(ctrl_freq), cmdCallback);
  ros::Timer param_update_timer = nh.createTimer(ros::Duration(0.5), updateParametersCallback);
  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}