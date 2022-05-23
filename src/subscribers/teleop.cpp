/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * LOCAL includes
 */
#include "teleop.hpp"


namespace naoqi
{
namespace subscriber
{

TeleopSubscriber::TeleopSubscriber( const std::string& name, const std::string& cmd_vel_topic, const std::string& joint_angles_topic, const qi::SessionPtr& session ):
  cmd_vel_topic_(cmd_vel_topic),
  joint_angles_topic_(joint_angles_topic),
  BaseSubscriber( name, cmd_vel_topic, session ),
  joint_angle_trajectory_topic_(joint_angles_topic+std::string("_trajectory")), //Anis
  p_motion_( session->service("ALMotion") )
{}

void TeleopSubscriber::reset( ros::NodeHandle& nh )
{
  sub_cmd_vel_ = nh.subscribe( cmd_vel_topic_, 10, &TeleopSubscriber::cmd_vel_callback, this );
  sub_joint_angles_ = nh.subscribe( joint_angles_topic_, 10, &TeleopSubscriber::joint_angles_callback, this );
  sub_joint_angle_trajectory_ = nh.subscribe(  joint_angle_trajectory_topic_, 10, &TeleopSubscriber::joint_angle_trajectory_callback, this ); //Anis

  is_initialized_ = true;
}

void TeleopSubscriber::cmd_vel_callback( const geometry_msgs::TwistConstPtr& twist_msg )
{
  // no need to check for max velocity since motion clamps the velocities internally
  const float& vel_x = twist_msg->linear.x;
  const float& vel_y = twist_msg->linear.y;
  const float& vel_th = twist_msg->angular.z;

  std::cout << "going to move x: " << vel_x << " y: " << vel_y << " th: " << vel_th << std::endl;
  p_motion_.async<void>("move", vel_x, vel_y, vel_th );
}

void TeleopSubscriber::joint_angles_callback( const naoqi_bridge_msgs::JointAnglesWithSpeedConstPtr& js_msg )
{
  if ( js_msg->relative==0 )
  {
    p_motion_.async<void>("setAngles", js_msg->joint_names, js_msg->joint_angles, js_msg->speed);
  }
  else
  {
    p_motion_.async<void>("changeAngles", js_msg->joint_names, js_msg->joint_angles, js_msg->speed);
  }
}

void TeleopSubscriber::joint_angle_trajectory_callback( const naoqi_bridge_msgs::JointAngleTrajectoryConstPtr& jt_msg ) //Anis
{
  std::vector< std::vector<float> > angles(jt_msg->joint_names.size());

  int n_angles = jt_msg->joint_angles.size() / jt_msg->joint_names.size();
  for(int i = 0; i < jt_msg->joint_names.size(); i++)
    angles[i] = std::vector<float>(&jt_msg->joint_angles[n_angles*i], &jt_msg->joint_angles[n_angles*(i+1)] );


  std::vector< std::vector<float> > times(jt_msg->joint_names.size());

  int n_times = jt_msg->times.size() / jt_msg->joint_names.size();
  for(int i = 0; i < jt_msg->joint_names.size(); i++)
    times[i] = std::vector<float>(&jt_msg->times[n_times*i], &jt_msg->times[n_times*(i+1)] );

  p_motion_.async<void>("angleInterpolation", jt_msg->joint_names, angles, times, jt_msg->relative != 1);
}


} //publisher
} // naoqi
