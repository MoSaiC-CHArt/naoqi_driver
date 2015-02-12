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

#ifndef LASER_PUBLISHER_HPP
#define LASER_PUBLISHER_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <qi/anyobject.hpp>

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class LaserPublisher : public BasePublisher<LaserPublisher>
{

public:
  LaserPublisher( const std::string& name, const std::string& topic, float frequency, const qi::AnyObject& p_memory );

  void publish();

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
    return pub_.getNumSubscribers() > 0;
  }

private:
  qi::AnyObject p_memory_;
  ros::Publisher pub_;
  sensor_msgs::LaserScan msg_;
}; // class

} //publisher
} // alros

#endif