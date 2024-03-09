#ifndef BACK_UP_FOLLOW_PATH_HPP
#define BACK_UP_FOLLOW_PATH_HPP

#include <memory>

#include "nav2_msgs/action/back_up_follow_path.hpp"
#include "nav2_behaviors/timed_behavior.hpp"

namespace nav2_behaviors
{
  class BackUpFollowPath : public TimedBehavior<nav2_msgs::action::BackUpFollowPath>
  {
  public:
    Status onRun(const std::shared_ptr<const nav2_msgs::action::BackUpFollowPath::Goal> command) override;
    Status onCycleUpdate() override;

  private:
    int count_ = 0;
    int max_count_ = 10;
    geometry_msgs::msg::Twist cmd_vel_;
    double get_distance(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2);
  };
}

#endif // BACK_UP_FOLLOW_PATH_HPP