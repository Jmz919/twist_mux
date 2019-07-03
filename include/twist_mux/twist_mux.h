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

#ifndef TWIST_MUX_H
#define TWIST_MUX_H

#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <list>

namespace twist_mux
{

// Forwarding declarations:
class TwistMuxDiagnostics;
struct TwistMuxDiagnosticsStatus;
class TopicsHandleBase;
class TopicsHandle;
class VelocityTopicHandle;
class VelocityTopicStampHandle;
class LockTopicHandle;

/**
 * @brief The TwistMux class implements a top-level twist multiplexer module
 * that priorize different velocity command topic inputs according to locks.
 */
class TwistMux
{
public:

  // @todo use this type alias when the compiler supports this C++11 feat.
  //template<typename T>
  //using handle_container = std::list<T>;
  // @todo alternatively we do the following:

  typedef std::list<boost::shared_ptr<TopicsHandleBase> >   velocity_topic_container;
  typedef std::list<boost::shared_ptr<LockTopicHandle>  >   lock_topic_container;

  TwistMux(int window_size = 10);
  ~TwistMux();

  bool hasPriority(const TopicsHandleBase& twist);

  inline void publishTwist(const geometry_msgs::Twist& msg)
  {
    if (cmd_pub_) {
      cmd_pub_.publish(msg);
    }
  }

  inline void publishTwistStamped(const geometry_msgs::TwistStamped& msg)
  {
    if (cmd_pub_stamped_) {
      cmd_pub_stamped_.publish(msg);
    }
  }

  void updateDiagnostics(const ros::TimerEvent& event);

  void getPublishers(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);


protected:

  typedef TwistMuxDiagnostics       diagnostics_type;
  typedef TwistMuxDiagnosticsStatus status_type;

  ros::Timer diagnostics_timer_;

  static constexpr double DIAGNOSTICS_PERIOD = 1.0;

  int pub_msg;

  /**
   * @brief velocity_hs_ Velocity topics' handles.
   * Note that if we use a vector, as a consequence of the re-allocation and
   * the fact that we have a subscriber inside with a pointer to 'this', we
   * must reserve the number of handles initially.
   */
  // @todo use handle_container (see above)

  boost::shared_ptr<velocity_topic_container> velocity_hs_;
  boost::shared_ptr<lock_topic_container>     lock_hs_;

  ros::Publisher cmd_pub_;
  ros::Publisher cmd_pub_stamped_;

  geometry_msgs::Twist last_cmd_;
  geometry_msgs::TwistStamped last_stamp_cmd_;

  void getTopicHandles(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  int getLockPriority();

  boost::shared_ptr<diagnostics_type> diagnostics_;
  boost::shared_ptr<status_type>      status_;
};

} // namespace twist_mux

#endif // TWIST_MUX_H
