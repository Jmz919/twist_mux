/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  or send a letter to
 *  Creative Commons, 444 Castro Street, Suite 900,
 *  Mountain View, California, 94041, USA.
 *********************************************************************/

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 */

#include <ros/ros.h>
#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include <twist_mux/twist_mux.h>
#include <twist_mux/topic_handle.h>
#include <twist_mux/twist_mux_diagnostics.h>
#include <twist_mux/twist_mux_diagnostics_status.h>
#include <twist_mux/utils.h>
#include <twist_mux/xmlrpc_helpers.h>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: linear (abs(x)) or angular (abs(yaw))
 * @param old_twist Old velocity
 * @param new_twist New velocity
 * @return true is any of the absolute velocity components has increased
 */
bool hasIncreasedAbsVelocity(const geometry_msgs::Twist& old_twist, const geometry_msgs::Twist& new_twist)
{
  const auto old_linear_x = std::abs(old_twist.linear.x);
  const auto new_linear_x = std::abs(new_twist.linear.x);

  const auto old_angular_z = std::abs(old_twist.angular.z);
  const auto new_angular_z = std::abs(new_twist.angular.z);

  return (old_linear_x  < new_linear_x ) or
         (old_angular_z < new_angular_z);
}

namespace twist_mux
{

TwistMux::TwistMux(int window_size)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  velocity_hs_ = boost::make_shared<velocity_topic_container>();
  lock_hs_     = boost::make_shared<lock_topic_container>();
  //Containers for the topics and locks

  getTopicHandles(nh, nh_priv);//, "topics", *velocity_hs_);
  //getTopicHandles(nh, nh_priv, "locks" , *lock_hs_ );
  //Gets the topics and locks

  // Publisher for output topic:
  std::string pub_msg = getPublisher(nh, nh_priv, "publisher");

  if(pub_msg == "twist") {
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_out", 1);
  }
  else {
    cmd_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_out", 1);
  }

  /// Diagnostics:
  diagnostics_ = boost::make_shared<diagnostics_type>();
  status_      = boost::make_shared<status_type>();
  status_->velocity_hs = velocity_hs_;
  status_->lock_hs     = lock_hs_;

  diagnostics_timer_ = nh.createTimer(ros::Duration(DIAGNOSTICS_PERIOD), &TwistMux::updateDiagnostics, this);
}

TwistMux::~TwistMux()
{}

void TwistMux::updateDiagnostics(const ros::TimerEvent& event)
{
  status_->priority = getLockPriority();
  diagnostics_->updateStatus(status_);
}

void TwistMux::publishTwist(const geometry_msgs::TwistConstPtr& msg)
{
  cmd_pub_.publish(*msg);
}

void TwistMux::publishTwistStamped(const geometry_msgs::TwistStampedConstPtr& msg)
{
  cmd_pub_.publish(*msg);
}

void TwistMux::getTopicHandles(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)//, const std::string& param_name)//, velocity_topic_container& topic_hs)
{
  try
  {
    xh::Array output;

    std::string param_name = "topics";
    xh::fetchParam(nh_priv, param_name, output);

    xh::Struct output_i;
    std::string name, topic;
    double timeout;
    int msg_type;
    int priority;

    //if(param_name=="topics") {
      for (int i = 0; i < output.size(); i++)
      {
        xh::getArrayItem(output, i, output_i);

        xh::getStructMember(output_i, "name"    , name    );
        xh::getStructMember(output_i, "topic"   , topic   );
        xh::getStructMember(output_i, "timeout" , timeout );
        xh::getStructMember(output_i, "priority", priority);
        xh::getStructMember(output_i, "msg_type", msg_type);
        //reads the data from the yaml file to get the topics
        //problem without having a msg_type labeled in the yaml

        if(msg_type == 1) {
          //create a VelocityTopicStampHandle
          //topic_hs.push_back the element to not construct the element again while having an element of VelocityTopicStampHandle

          boost::shared_ptr<VelocityTopicStampHandle> twistStamped = boost::make_shared<VelocityTopicStampHandle>(nh, name, topic, timeout, priority, this, msg_type);
          velocity_hs_->push_back(twistStamped);
        }
        else {
          //create a VelocityTopicHandle
          //topic_hs.push_back the element to not construct the element again while having an element of VelocityTopicHandle

          boost::shared_ptr<VelocityTopicHandle> twist = boost::make_shared<VelocityTopicHandle>(nh, name, topic, timeout, priority, this, msg_type);
          velocity_hs_->push_back(twist);
        }
      }
    }
    catch (const xh::XmlrpcHelperException& e)
    {
      ROS_FATAL_STREAM("Error parsing topics params: " << e.what());
    }

    try {
      xh::Array output;

      xh::Struct output_i;
      std::string name, topic;
      double timeout;
      int msg_type;
      int priority;

      std::string param_name = "locks";
      xh::fetchParam(nh_priv, param_name, output);

      for (int i = 0; i < output.size(); i++)
      {
        xh::getArrayItem(output, i, output_i);

        xh::getStructMember(output_i, "name"    , name    );
        xh::getStructMember(output_i, "topic"   , topic   );
        xh::getStructMember(output_i, "timeout" , timeout );
        xh::getStructMember(output_i, "priority", priority);
        //reads the data from the yaml file to get the topics
        //problem without having a msg_type labeled in the yaml
        boost::shared_ptr<LockTopicHandle> lock = boost::make_shared<LockTopicHandle>(nh, name, topic, timeout, priority, this);
        lock_hs_->push_back(lock);
        // adds the topics onto the vector or list
      }
    //}
  }
  catch (const xh::XmlrpcHelperException& e)
  {
    ROS_FATAL_STREAM("Error parsing locks params: " << e.what());
  }
}

void TwistMux::getPublisher(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, const std::string& param_name)
{
  try
  {

  }
  catch (const xh::XmlrpcHelperException& e)
  {
    ROS_FATAL_STREAM("Error parsing params: " << e.what());
  }
}

int TwistMux::getLockPriority()
{
  LockTopicHandle::priority_type priority = 0;

  /// max_element on the priority of lock topic handles satisfying
  /// that is locked:
  for (const auto& lock_h : *lock_hs_)
  {
    if (lock_h->isLocked())
    {
      auto tmp = lock_h->getPriority();
      if (priority < tmp)
      {
        priority = tmp;
      }
    }
  }

  ROS_DEBUG_STREAM("Priority = " << static_cast<int>(priority));

  return priority;
}

bool TwistMux::hasPriority(const TopicsHandleBase& twist)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string velocity_name = "NULL";

  /// max_element on the priority of velocity topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto& velocity_h : *velocity_hs_)
  {
    if (not velocity_h->isMasked(lock_priority))
    {
      const auto velocity_priority = velocity_h->getPriority();
      if (priority < velocity_priority)
      {
        priority = velocity_priority;
        velocity_name = velocity_h->getName();
      }
    }
  }

  return twist.getName() == velocity_name;
}
} // namespace twist_mux
