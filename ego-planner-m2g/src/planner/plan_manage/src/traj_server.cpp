#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

#include "plan_manage/VelCmd.h"

ros::Publisher pos_cmd_pub;
ros::Publisher vel_cmd_pub;

quadrotor_msgs::PositionCommand cmd;

airsim_ros::VelCmd vel_cmd;


double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

double last_roll_, last_roll_dot_;
double last_pitch_, last_pitch_dot_;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;

    //    0  1  2  3  ... msg->post_pts.size
    //0:  x1 x2 x3 x4 ... x_size
    //1:  y1 y2 y3 y4 ... y_size
    //2:  z1 z2 z3 z4 ... z_size


  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  //recursive

  //vector[0]: pos_traj is a b-spline :position
  //vector[1]: is a b-spline : vel
  //vector[2]: is a b-spline : acc

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  //dir is Differential. Yes, it is the math Differential

  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_; //y and x angle is dir(1), dir(0)
  //This is I want!!!!!

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

std::pair<double, double> calculate_rotation(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last, double &last_rot_, double &last_rot_dot_, int rotation = 1)
{
  // rotation = 1 -> yaw
  // rotation = 2 -> roll
  // rotation = 3 -> pitch

  constexpr double PI = 3.1415926;
  //constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // std::pair<double, double> yaw_yawdot(0, 0);
  // double yaw = 0;
  // double yawdot = 0;
  // double yaw_temp = 0;

  constexpr double ROT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> rot_rotdot(0,0);
  double rot = 0;
  double rotdot = 0;
  double rot_temp = 0;


  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  //dir is Differential. Yes, it is the math Differential

  //dir(i) -> 0:x, 1:y, 2:z
  //
  if (rotation == 1)
    {
      rot_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_rot_; //y and x angle is dir(1), dir(0)
    }
  else if (rotation == 2)//roll
    {
      rot_temp = dir.norm() > 0.1 ? atan2(dir(2), dir(1)) : last_rot_;
    }
  else if (rotation == 3)//pitch
    {
      rot_temp = dir.norm() > 0.1 ? atan2(dir(0), dir(2)) : last_rot_;
    }
  //This is I want!!!!!

  //double max_yaw_change = ROT_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  double max_rot_change = ROT_DOT_MAX_PER_SEC * (time_now - time_last).toSec();

  if (rot_temp - last_rot_ > PI)
  {
    if (rot_temp - last_rot_ - 2 * PI < -max_rot_change)
    {
      rot = last_rot_ - max_rot_change;
      if (rot < -PI)
        rot += 2 * PI;

      rotdot = -ROT_DOT_MAX_PER_SEC;
    }
    else
    {
      rot = rot_temp;
      if (rot - last_rot_ > PI)
        rotdot = -ROT_DOT_MAX_PER_SEC;
      else
        rotdot = (rot_temp - last_rot_) / (time_now - time_last).toSec();
    }
  }
  else if (rot_temp - last_rot_ < -PI)
  {
    if (rot_temp - last_rot_ + 2 * PI > max_rot_change)
    {
      rot = last_rot_ + max_rot_change;
      if (rot > PI)
        rot -= 2 * PI;

      rotdot = ROT_DOT_MAX_PER_SEC;
    }
    else
    {
      rot = rot_temp;
      if (rot - last_rot_ < -PI)
        rotdot = ROT_DOT_MAX_PER_SEC;
      else
        rotdot = (rot_temp - last_rot_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (rot_temp - last_rot_ < -max_rot_change)
    {
      rot = last_rot_ - max_rot_change;
      if (rot < -PI)
        rot += 2 * PI;

      rotdot = -ROT_DOT_MAX_PER_SEC;
    }
    else if (rot_temp - last_rot_ > max_rot_change)
    {
      rot = last_rot_ + max_rot_change;
      if (rot > PI)
        rot -= 2 * PI;

      rotdot = ROT_DOT_MAX_PER_SEC;
    }
    else
    {
      rot = rot_temp;
      if (rot - last_rot_ > PI)
        rotdot = -ROT_DOT_MAX_PER_SEC;
      else if (rot - last_rot_ < -PI)
        rotdot = ROT_DOT_MAX_PER_SEC;
      else
        rotdot = (rot_temp - last_rot_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(rot - last_rot_) <= max_rot_change)
    rot = 0.5 * last_rot_ + 0.5 * rot; // nieve LPF
  rotdot = 0.5 * last_rot_dot_ + 0.5 * rotdot;
  last_rot_ = rot;
  last_rot_dot_ = rotdot;

  rot_rotdot.first = rot;
  rot_rotdot.second = rotdot;

  return rot_rotdot;
}




void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);
  std::pair<double, double> roll_rolldot(0, 0);//
  std::pair<double, double> pitch_pitchdot(0, 0);//

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    //B-spline Derivative

    pos = traj_[0].evaluateDeBoorT(t_cur); // function:traj_[0].evaluateDeBoorT(t_cur) is a function, which return a vector
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate rotation ***/
    //yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    yaw_yawdot = calculate_rotation(t_cur, pos, time_now, time_last, last_yaw_, last_yaw_dot_, 1);
    roll_rolldot = calculate_rotation(t_cur, pos, time_now, time_last, last_roll_, last_roll_dot_,2);
    pitch_pitchdot = calculate_rotation(t_cur, pos, time_now, time_last, last_pitch_, last_pitch_dot_, 3);
    /*** calculate rotation ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    roll_rolldot.first = last_roll_;
    roll_rolldot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

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

  //airsim_control
  
  vel_cmd.twist.angular.x = roll_rolldot.second;
  vel_cmd.twist.angular.y = pitch_pitchdot.second;
  vel_cmd.twist.angular.z = yaw_yawdot.second;

  vel_cmd.twist.linear.x = vel(0);
  vel_cmd.twist.linear.y = vel(1);
  vel_cmd.twist.linear.z = vel(2);

  vel_cmd_pub.publish(vel_cmd);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  vel_cmd_pub = node.advertise<airsim_ros::VelCmd>("/vel_cmd_body_frame", 50);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0); //origin is 1
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  last_roll_ = 0.0;
  last_roll_dot_ = 0.0;

  last_pitch_ = 0.0;
  last_pitch_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}