// Copyright 2023 amsl

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class PIDController
{
public:
  PIDController();
  void process();

private:
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);
  void twist_callback(const geometry_msgs::TwistConstPtr &msg);
  geometry_msgs::Twist controller();

  double hz_;
  double dt_;
  double Kp_of_angular_;
  double Ki_of_angular_;
  double Kd_of_angular_;
  double prev_error_of_angular_;
  double i_sum_of_angular_;

  geometry_msgs::Twist cmd_vel_;
  nav_msgs::Odometry odom_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber twist_sub_;
  ros::Publisher twist_pub_;
};

PIDController::PIDController() : private_nh_("~"), prev_error_of_angular_(0.0), i_sum_of_angular_(0.0)
{
  private_nh_.param<double>("hz", hz_, 20.0);
  private_nh_.param<double>("Kp_of_angular", Kp_of_angular_, 1.0);
  private_nh_.param<double>("Ki_of_angular", Ki_of_angular_, 1.0);
  private_nh_.param<double>("Kd_of_angular", Kd_of_angular_, 1.0);

  dt_ = 1.0 / hz_;

  ROS_INFO_STREAM("=== PID Controller ===");
  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("dt: " << dt_);
  ROS_INFO_STREAM("Kp_of_angular: " << Kp_of_angular_);
  ROS_INFO_STREAM("Ki_of_angular: " << Ki_of_angular_);
  ROS_INFO_STREAM("Kd_of_angular: " << Kd_of_angular_);

  odom_sub_ = nh_.subscribe("/odom", 1, &PIDController::odom_callback, this);
  twist_sub_ = nh_.subscribe("/cmd_vel_in", 1, &PIDController::twist_callback, this);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_out", 1);
}

void PIDController::odom_callback(const nav_msgs::OdometryConstPtr &msg) { odom_ = *msg; }

void PIDController::twist_callback(const geometry_msgs::TwistConstPtr &msg) { cmd_vel_ = *msg; }

geometry_msgs::Twist PIDController::controller()
{
  const double error_of_angular_ = cmd_vel_.angular.z - odom_.twist.twist.angular.z;
  i_sum_of_angular_ += (error_of_angular_ + prev_error_of_angular_) * dt_ / 2.0;

  const double P_of_angular_ = Kp_of_angular_ * error_of_angular_;
  const double I_of_angular_ = Ki_of_angular_ * i_sum_of_angular_;
  const double D_of_angular_ = Kd_of_angular_ * (error_of_angular_ - prev_error_of_angular_) / dt_;
  cmd_vel_.angular.z = P_of_angular_ + I_of_angular_ + D_of_angular_;

  prev_error_of_angular_ = error_of_angular_;

  return cmd_vel_;
}

void PIDController::process()
{
  ros::Rate loop_rate(hz_);

  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel = controller();
    twist_pub_.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_controller");
  PIDController pid_controller;
  pid_controller.process();

  return 0;
}
