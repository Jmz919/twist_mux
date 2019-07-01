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

  //Containers for the topics and locks
  velocity_hs_ = boost::make_shared<velocity_topic_container>();
  lock_hs_     = boost::make_shared<lock_topic_container>();

  //Gets the topics and locks
  getTopicHandles(nh, nh_priv);

  //Gets the Publishers for output topics:
  getPublishers(nh, nh_priv);

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

void TwistMux::publishTwist(const geometry_msgs::Twist& msg)
{
  if (cmd_pub_) {
    cmd_pub_.publish(msg);
  }
  // cmd_pub_twist_.publish(msg);
}

void TwistMux::publishTwistStamped(const geometry_msgs::TwistStamped& msg)
{
  if (cmd_pub_stamped_) {
    cmd_pub_stamped_.publish(msg);
  }
  // cmd_pub_twist_stamped_.publish(msg);
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
    nh.param<int>("msg_type", msg_type, 0);

    //if(param_name=="topics") {
      for (int i = 0; i < output.size(); i++)
      {
        xh::getArrayItem(output, i, output_i);

        xh::getStructMember(output_i, "name"    , name    );
        xh::getStructMember(output_i, "topic"   , topic   );
        xh::getStructMember(output_i, "timeout" , timeout );
        xh::getStructMember(output_i, "priority", priority);
        if(msg_type) {
          xh::getStructMember(output_i, "msg_type", msg_type);
        }
        else {
          msg_type = 0;
        }

        if(msg_type == 1) {
          boost::shared_ptr<VelocityTopicStampHandle> twistStamped = boost::make_shared<VelocityTopicStampHandle>(nh, name, topic, timeout, priority, this, msg_type);
          velocity_hs_->push_back(twistStamped);
        }
        else {
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

        boost::shared_ptr<LockTopicHandle> lock = boost::make_shared<LockTopicHandle>(nh, name, topic, timeout, priority, this);
        lock_hs_->push_back(lock);
      }
  }
  catch (const xh::XmlrpcHelperException& e)
  {
    ROS_FATAL_STREAM("Error parsing locks params: " << e.what());
  }
}

void TwistMux::getPublishers(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
{
  try
  {
    xh::Array output;
    xh::Struct output_i;
    std::string topic;
    int msg_type;
    std::string param_name = "publishers";

    xh::fetchParam(nh_priv, param_name, output);

    for (int i = 0; i < 2; i++)
    {
      xh::getArrayItem(output, i, output_i);

      xh::getStructMember(output_i, "topic", topic);
      xh::getStructMember(output_i, "msg_type", msg_type);

      if(topic.length() != 0) {
        if(msg_type == 1) {
          cmd_pub_stamped_ = nh.advertise<geometry_msgs::TwistStamped>(topic, 1);
        }
        else {
          cmd_pub_ = nh.advertise<geometry_msgs::Twist>(topic, 1);
        }
      }
    }
  }
  catch (const xh::XmlrpcHelperException& e)
  {
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_out", 1);
    // cmd_pub_twist_stamped_ = nh.advertise<geometry_msgs::TwistStamped>("twistStamped", 1);
    //In case there is a problem parsing or old file
    //There are default publishers

    ROS_WARN_STREAM("Error parsing publishers defaulting to Twist: " << e.what());
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
