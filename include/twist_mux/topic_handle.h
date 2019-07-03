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

#ifndef TOPIC_HANDLE_H
#define TOPIC_HANDLE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <twist_mux/utils.h>
#include <twist_mux/twist_mux.h>

#include <boost/utility.hpp>
#include <boost/scoped_ptr.hpp>

#include <string>
#include <vector>

namespace twist_mux
{

  class LockTopicHandle {
    public:
      typedef int priority_type;
      LockTopicHandle(ros::NodeHandle& nh, const std::string& name, const std::string& topic, double timeout, priority_type priority, TwistMux* mux, int msg_type=0)
        : nh_(nh)
        , name_(name)
        , topic_(topic)
        , timeout_(timeout)
        , priority_(clamp(priority, priority_type(0), priority_type(255)))
        , mux_(mux)
        , msg_type_(msg_type)
        , stamp_(0.0)
        //holds the names of the topics and other data
      {
        subscriber_ = nh_.subscribe(topic_, 1, &LockTopicHandle::callback, this);

        ROS_INFO_STREAM
        (
          "Topic handler '" << name_ << "' subscribed to topic '" << topic_ <<
          "': timeout = " << ((timeout_) ? std::to_string(timeout_) + "s" : "None") <<
          ", priority = " << static_cast<int>(priority_)
          //if successful reports that the node subscribe to the topic
        );
      }

      /**
       * @brief isLocked
       * @return true if has expired or locked (i.e. bool message data is true)
       */
      bool isLocked() const
      {
        return hasExpired() or getMessage().data;
      }

      void callback(const std_msgs::BoolConstPtr& msg)
      {
        stamp_ = ros::Time::now();
        msg_   = *msg;
      }

      bool hasExpired() const
      {
        return (timeout_ > 0.0) and
               ((ros::Time::now() - stamp_).toSec() > timeout_);
      }

      const std::string& getName() const
      {
        return name_;
      }

      const std::string& getTopic() const
      {
        return topic_;
      }

      const double& getTimeout() const
      {
        return timeout_;
      }

      const int& getMsgType() const
      {
        return msg_type_;
      }


      /**
       * @brief getPriority Priority getter
       * @return Priority
       */
      const priority_type& getPriority() const
      {
        return priority_;
      }

      const ros::Time& getStamp() const
      {
        return stamp_;
      }

      const std_msgs::Bool& getMessage() const
      {
        return msg_;
      }

    protected:
      ros::NodeHandle nh_;
      std::string name_;
      std::string topic_;
      ros::Subscriber subscriber_;
      double timeout_;
      int msg_type_;
      priority_type priority_;

    protected:
      TwistMux* mux_;
      ros::Time stamp_;
      std_msgs::Bool msg_;
      geometry_msgs::TwistStamped ts;
    };


  class TopicsHandleBase {
    public:

      typedef int priority_type;

      TopicsHandleBase(ros::NodeHandle& nh, const std::string& name, const std::string& topic, double timeout, priority_type priority, TwistMux* mux, int msg_type=0)
        : nh_(nh)
        , name_(name)
        , topic_(topic)
        , timeout_(timeout)
        , priority_(clamp(priority, priority_type(0), priority_type(255)))
        , mux_(mux)
        , msg_type_(msg_type)
        {
        }

        virtual ~TopicsHandleBase()
        {
          subscriber_.shutdown();
        }

        /**
         * @brief hasExpired
         * @return true if the message has expired; false otherwise.
         *         If the timeout is set to 0.0, this function always returns
         *         false
         */
        bool hasExpired() const
        {
          return (timeout_ > 0.0) and
                 ((ros::Time::now() - ts_.header.stamp).toSec() > timeout_);
        }


        const std::string& getName() const
        {
          return name_;
        }

        const std::string& getTopic() const
        {
          return topic_;
        }

        const double& getTimeout() const
        {
          return timeout_;
        }

        const int& getMsgType() const
        {
          return msg_type_;
        }

        const priority_type& getPriority() const {
          return priority_;
        }

        const ros::Time& getTime() const {
          return ts_.header.stamp;
        }

        const geometry_msgs::Twist getTwist() const {
          return ts_.twist;
        }


        bool isMasked(priority_type lock_priority) const
        {
          return hasExpired() or (getPriority() < lock_priority);
        }


      protected:
        ros::NodeHandle nh_;
        std::string name_;
        std::string topic_;
        ros::Subscriber subscriber_;
        double timeout_;
        int msg_type_;
        priority_type priority_;

        TwistMux* mux_;
        geometry_msgs::TwistStamped ts_;
      };


template<typename T>
class TopicHandle_ : public TopicsHandleBase, boost::noncopyable
{
public:

  typedef int priority_type;

  /**
   * @brief TopicHandle_
   * @param nh Node handle
   * @param name Name identifier
   * @param topic Topic name
   * @param timeout Timeout to consider that the messages are old; note
   * that initially the message stamp is set to 0.0, so the message has
   * expired
   * @param priority Priority of the topic
   */
  TopicHandle_(ros::NodeHandle& nh, const std::string& name, const std::string& topic, double timeout, priority_type priority, TwistMux* mux, int msg_type=0)
    : TopicsHandleBase(nh, name, topic, timeout, priority, mux, msg_type)
    //holds the names of the topics and other data
  {
    ROS_INFO_STREAM
    (
      "Topic handler '" << name_ << "' subscribed to topic '" << topic_ <<
      "': timeout = " << ((timeout_) ? std::to_string(timeout_) + "s" : "None") <<
      ", priority = " << static_cast<int>(priority_) << " msg_type = " << static_cast<int>(msg_type_)
      //if successful reports that the node subscribe to the topic
    );
  }

  virtual ~TopicHandle_()
  {
    subscriber_.shutdown();
  }


};

class TwistTopicHandle : public TopicHandle_<geometry_msgs::Twist>
{

public:

  TwistTopicHandle(ros::NodeHandle& nh, const std::string& name, const std::string& topic, double timeout, priority_type priority, TwistMux* mux, int msg_type=0)
    : TopicHandle_<geometry_msgs::Twist>(nh, name, topic, timeout, priority, mux, msg_type)
  {
    subscriber_ = nh_.subscribe(topic_, 1, &TwistTopicHandle::callback, this);
  }

  void callback(const geometry_msgs::TwistConstPtr& msg)
  {

    ts_.header.stamp = ros::Time::now();
    ts_.twist = *msg;

    // Check if this twist has priority.
    // Note that we have to check all the locks because they might time out
    // and since we have several topics we must look for the highest one in
    // all the topic list; so far there's no O(1) solution.

    //check the publisher type to determine how to publish

    if (mux_->hasPriority(*this))
    {
      mux_->publishTwist(ts_.twist);
      mux_->publishTwistStamped(ts_);
    }
  }
};

//may want to remove the publish msg to make sure each msg is evaulated for priority

class TwistStampedTopicHandle : public TopicHandle_<geometry_msgs::TwistStamped>
{

public:

  TwistStampedTopicHandle(ros::NodeHandle& nh, const std::string& name, const std::string& topic, double timeout, priority_type priority, TwistMux* mux, int msg_type=0)
    : TopicHandle_<geometry_msgs::TwistStamped>(nh, name, topic, timeout, priority, mux, msg_type)
  {
    subscriber_ = nh_.subscribe(topic_, 1, &TwistStampedTopicHandle::callback, this);
  }

  void callback(const geometry_msgs::TwistStampedConstPtr& msg)
  {

    ts_  = *msg;

    // Check if this twist has priority.
    // Note that we have to check all the locks because they might time out
    // and since we have several topics we must look for the highest one in
    // all the topic list; so far there's no O(1) solution.

    //check the publisher type and to determine how to publish

    if (mux_->hasPriority(*this))
    {
      mux_->publishTwist(ts_.twist);
      mux_->publishTwistStamped(ts_);
    }
  }
};

} // namespace twist_mux

#endif // TOPIC_HANDLE_H
