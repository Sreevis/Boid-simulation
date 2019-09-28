/*
 Copywrite 2019. All rights reserved.
 Author: Sreevishnu Sajikumar
 Contact: sreevishnus18@yahoo.com
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Boid node: for controlling each boids according to flocking algorithm
 * Refer : http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/ for more
 */

#include "boid_gazebo/boid.h"

using namespace boid;

BoidController::BoidController(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
  ROS_INFO("Initializing boid ...!");
  gz_states_sub_ = nh_.subscribe("/gazebo/model_states", 100, &BoidController::subscriberCB,this);
  input_goal_ = nh_.subscribe("/boid/set_goal", 100, &BoidController::inputGoalCB,this);
  vel_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1, true);

  min_vel_ = 0.5;
  max_vel_ = 1;
  min_steer_vel_ = 0.5;
  max_steer_vel_ = 1;

  view_radius_ = 0.5;
  view_angle_ = M_PI_2; // halfside
  avoidance_radius_ = 0.5;
  avoidance_weight_ = 1;
  centering_weight_ = 1;
  matching_weight_ = 10;
  turning_vel_ = 2;

  goal_.setValue(5,5,5);
}

// Subscriber callback for getting model states from gazebo
void BoidController::subscriberCB(const gazebo_msgs::ModelStates& msg)
{
  getBoidStates(msg);
  update();
}

// Subscriber callback for getting user input for goal
void BoidController::inputGoalCB(const geometry_msgs::Point& msg)
{
  goal_.setX(msg.x);
  goal_.setY(msg.y);
  goal_.setZ(msg.z);
}

// Set own parms and collect infos of neighbours
void BoidController::getBoidStates(const gazebo_msgs::ModelStates& msg)
{
  flock_.clear();
  for (unsigned long int i=0;i<msg.name.size();i++)
  {
    if(msg.name[i] != "ground_plane")
    {
      if(nh_.getNamespace() == "/"+msg.name[i])
      {
        boid_.pos.setX(msg.pose[i].position.x);
        boid_.pos.setY(msg.pose[i].position.y);
        boid_.pos.setZ(msg.pose[i].position.z);

        boid_.heading.setX(msg.pose[i].orientation.x);
        boid_.heading.setY(msg.pose[i].orientation.y);
        boid_.heading.setZ(msg.pose[i].orientation.z);
        boid_.heading.setW(msg.pose[i].orientation.w);

        boid_.forward_vel = msg.twist[i].linear.x;
        boid_.turning_vel = msg.twist[i].angular.x; //use uniform velocity about all axis

      }else
      {
	Boid neighbour;
        neighbour.pos.setX(msg.pose[i].position.x);
        neighbour.pos.setY(msg.pose[i].position.y);
        neighbour.pos.setZ(msg.pose[i].position.z);

        neighbour.heading.setX(msg.pose[i].orientation.x);
        neighbour.heading.setY(msg.pose[i].orientation.y);
        neighbour.heading.setZ(msg.pose[i].orientation.z);
        neighbour.heading.setW(msg.pose[i].orientation.w);

        neighbour.forward_vel = msg.twist[i].linear.x;
        neighbour.turning_vel = msg.twist[i].angular.x; //use uniform velocity about all axis

        // Collect information of neighbouring boids within view angle and view radius
        if((neighbour.pos.distance(boid_.pos) <= view_radius_) &&
           (2*fabs(lookAt(boid_.pos,neighbour.pos).normalize().angle(boid_.heading.normalize()))<= view_angle_)){
          //ROS_INFO("Test %s %f", nh_.getNamespace().c_str(),2* lookAt(boid_.pos,neighbour.pos).normalize().angle(boid_.heading.normalize()));
           flock_.push_back(neighbour);
        }

      }

    }

  }

}

/* Sets the velocity of the boid.
 */
void BoidController::setVel(const tf2::Vector3& angular_vel)
{
  gazebo_msgs::ModelState  boid_vel;

  std::string  boid_name = nh_.getNamespace();
  boid_vel.model_name = boid_name.erase(0,1);

  boid_vel.twist.linear.x = computeVelocity(); // Movement is always towards front
  boid_vel.twist.angular.x = angular_vel.x();
  boid_vel.twist.angular.y = angular_vel.y();
  boid_vel.twist.angular.z = angular_vel.z();

  boid_vel.reference_frame = boid_vel.model_name+"::base_link";

  vel_pub_.publish(boid_vel);

}

/* Convert Quaternion to RPY
 *  Ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
EulerAngles BoidController::toEulerAngles(const tf2::Quaternion& q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

// Set the orientation of boid w.r.t resultant heading of avoidance and centering
void BoidController::setHeading(const tf2::Quaternion& goal_heading)
{
  EulerAngles eu_goal, current_heading;
  eu_goal = toEulerAngles(goal_heading);
  current_heading = toEulerAngles(boid_.heading);

  double r =0, p =0, y =0;
  tf2::Vector3 angular_vel;

  (eu_goal.pitch > current_heading.pitch) ? p = turning_vel_ : p = -turning_vel_;
  if(eu_goal.yaw > current_heading.yaw)
  {
    if(fabs(eu_goal.yaw - current_heading.yaw) > M_PI){
      y = -turning_vel_ ;
    }
    else if (fabs(eu_goal.yaw - current_heading.yaw) < M_PI) {
      y = turning_vel_;
    }
  }
  else {
    if(fabs(eu_goal.yaw - current_heading.yaw) > M_PI){
      y = turning_vel_ ;
    }
    else if (fabs(eu_goal.yaw - current_heading.yaw) < M_PI) {
      y = -turning_vel_;
    }

  }
  angular_vel.setX(r);
  angular_vel.setY(p);
  angular_vel.setZ(y);

  setVel(angular_vel);
}

// Computes the quternion rotation required for a rotation from source point to destination point
tf2::Quaternion BoidController::lookAt(const tf2::Vector3& src, const tf2::Vector3& dest)
{
  tf2::Quaternion q;
  tf2::Vector3 up(0,0,1);
  tf2::Vector3 forward_axis(1,0,0);
  tf2::Vector3 forward_vector = (dest - src).normalize();
  double dot = forward_axis.dot(forward_vector);

  if (fabs(dot - (-1.0)) < 0.000001)
  {
      return tf2::Quaternion(up.x(), up.y(), up.z(), M_PI);
  }
  if (fabs(dot - (1.0)) < 0.000001)
  {
    return q.getIdentity();
  }
  double rotAngle = forward_axis.angle(forward_vector);
  tf2::Vector3 rotAxis = forward_axis.cross(forward_vector);
  rotAxis = rotAxis.normalize();
  q.setRotation(rotAxis, rotAngle);
  return q;
}

// Computes the heading using flocking algorithm
tf2::Quaternion BoidController::computeHeading(){
  //Based on http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/
  // Movement of each boid is based on avoidance_heading, velocity_matching and center_heading

  tf2::Quaternion goal_heading, center_heading, match_heading, avoidance_heading, random_heading;

  avoidance_heading = computeAvoidanceHeading();
  if(flock_.size() == 0)
  {
    center_heading = center_heading.getIdentity();
    random_heading = lookAt(boid_.pos, goal_);
    // Since no neighbours are detected, movement is only affected by input goal
    goal_heading =   random_heading.normalize();
   }else
  {
    // Movement is based on the movement of detected neighbours
     center_heading = lookAt(boid_.pos, getFlockCenter());
     double v = turning_vel_* centering_weight_;
     turning_vel_ = std::max<double>(min_steer_vel_, std::min<double>(v, max_steer_vel_));
     goal_heading =    avoidance_heading.normalize()* center_heading.normalize();
  }
  return goal_heading;
}

// Compute the avoidance heading using ahead vector
tf2::Quaternion BoidController::computeAvoidanceHeading()
{
  tf2::Quaternion avoidance_heading;
  avoidance_heading = avoidance_heading.getIdentity();
  double f_vel=computeVelocity();
  tf2::Vector3 ahead(boid_.pos);
  tf2::Vector3 ahead2(boid_.pos); // adds more safety
  ahead.setX(boid_.pos.x() + f_vel * view_radius_);
  ahead2.setX(boid_.pos.x() + f_vel * view_radius_*0.5);
  tf2::Quaternion q = boid_.heading * ahead;
  q *= boid_.heading.inverse();

  q.normalize();
  ahead.setX(boid_.pos.x() + q.getX());
  ahead.setY(boid_.pos.y() + q.getY());
  ahead.setZ(boid_.pos.z() + q.getZ());

  for (auto itr=flock_.begin();itr!=flock_.end();itr++)
  {
    if(checkCollision(itr->pos, ahead, ahead2)){
      if(boid_.pos.distance(ahead)> boid_.pos.distance(itr->pos)){
        avoidance_heading = lookAt(itr->pos, ahead2); 
      }
      else{
        avoidance_heading = lookAt(itr->pos, ahead);
      }
    }
    else {
      avoidance_heading = avoidance_heading.getIdentity();
    }
  }
  return avoidance_heading.normalize();
}

bool BoidController::checkCollision(const tf2::Vector3& obstacle, const tf2::Vector3& ahead, const tf2::Vector3& ahead2)
{
  if(obstacle.distance(ahead) <= avoidance_radius_ || obstacle.distance(ahead2) <= avoidance_radius_)
  {
    return true;
  }
  else {
    return false;
  }
}

// Computes the center of neighbours and own for calculating center_heading
tf2::Vector3 BoidController::getFlockCenter()
{
  tf2::Vector3 flock_center(0,0,0),  flock_pos_sum(0,0,0);
  for (auto i=flock_.begin();i!=flock_.end();i++) {
    flock_pos_sum+= i->pos;
  }
  flock_center = (flock_pos_sum + boid_.pos)/(flock_.size()+1);
  return flock_center;
}

// Computes forward velocity
double BoidController::computeVelocity()
{
  double flock_vel = 0;
  if(flock_.size()!=0)
  {
    for (auto i=flock_.begin();i!=flock_.end();i++) {
      flock_vel += i->forward_vel;
    }
    flock_vel = (flock_vel+boid_.forward_vel)/(flock_.size()+1);
    flock_vel = flock_vel* matching_weight_;
    flock_vel = std::max<double>(min_vel_, std::min<double>(flock_vel, max_vel_));
  }else {
      flock_vel= max_vel_;
  }
  return flock_vel;
}

// Updates with each iteration
void BoidController::update()
{
  tf2::Quaternion goal_heading;
  goal_heading = computeHeading();
  setHeading(goal_heading.normalized());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "boid");
    ros::NodeHandle nh;
    BoidController BoidController(&nh);
    ros::spin();
    return 0;
}
