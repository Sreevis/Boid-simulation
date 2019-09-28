#ifndef BOID_H
#define BOID_H

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace boid
{

struct Boid{
  tf2::Vector3 pos;
  tf2::Quaternion heading;
  double forward_vel;
  double turning_vel;
};

struct EulerAngles
{
    double roll, pitch, yaw;
};

class BoidController
{
public:
  BoidController(ros::NodeHandle *nh);

private:
  void subscriberCB(const gazebo_msgs::ModelStates& msg);
  void inputGoalCB(const geometry_msgs::Point& msg);
  void getBoidStates(const gazebo_msgs::ModelStates& msg);
  
  void setVel(const tf2::Vector3& angular_vel);
  EulerAngles toEulerAngles(const tf2::Quaternion& q);
  void setHeading(const tf2::Quaternion& goal_heading);
  tf2::Quaternion lookAt(const tf2::Vector3& src, const tf2::Vector3& dest);
  tf2::Quaternion computeHeading();
  tf2::Quaternion computeAvoidanceHeading();
  bool checkCollision(const tf2::Vector3& obstacle, const tf2::Vector3& ahead, const tf2::Vector3& ahead2);
  tf2::Vector3 getFlockCenter();
  double computeVelocity();
  void update();
  
  ros::NodeHandle nh_;
  ros::Subscriber gz_states_sub_, input_goal_;
  ros::Publisher  vel_pub_;

  std::vector<Boid> flock_;
  Boid boid_;

  //Params
  double view_radius_;
  double view_angle_;
  double avoidance_radius_;

  double min_vel_;
  double max_vel_;

  double min_steer_vel_;
  double max_steer_vel_;

  double avoidance_weight_;
  double centering_weight_;
  double matching_weight_;
  double goal_weight_;

  double turning_vel_;
  tf2::Vector3 goal_;
};

}

#endif // BOID_H
