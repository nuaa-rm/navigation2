#include "nav2_behaviors/plugins/back_up_follow_path.hpp"

namespace nav2_behaviors
{
  Status BackUpFollowPath::onRun(const std::shared_ptr<const nav2_msgs::action::BackUpFollowPath::Goal> command)
  {
    RCLCPP_INFO(logger_, "后退节点成功启动");
    nav2_msgs::action::BackUpFollowPath_Goal goal_;
    goal_ = *command;
    max_count_ = goal_.time / cycle_frequency_;
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0;
    // geometry_msgs::msg::PoseStamped current_pose;
    // if (!nav2_util::getCurrentPose(
    //         current_pose, *tf_, global_frame_, robot_base_frame_,
    //         transform_tolerance_))
    // {
    //   RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    //   return Status::FAILED;
    // }
    // if (goal_.path.poses.size() == 0)
    // {
    //   RCLCPP_INFO(logger_, "没有路径点");
    //   cmd_vel_.linear.x = -goal_.speed;
    //   return Status::SUCCEEDED;
    // }
    // std::size_t point_position = 0;
    // if (get_distance(current_pose.pose, goal_.path.poses[0].pose) < get_distance(current_pose.pose, goal_.path.poses[goal_.path.poses.size() - 1].pose))
    // {
    //   while (get_distance(current_pose.pose, goal_.path.poses[0].pose) < goal_.distance)
    //   {
    //     point_position++;
    //   }
    //   if (point_position > goal_.path.poses.size() - 1)
    //   {
    //     point_position = goal_.path.poses.size() - 1;
    //   }
    // }
    // else
    // {
    //   point_position = goal_.path.poses.size() - 1;
    //   while (get_distance(current_pose.pose, goal_.path.poses[point_position].pose) < goal_.distance)
    //   {
    //     point_position--;
    //   }
    //   if (point_position > goal_.path.poses.size() - 1)
    //   {
    //     point_position = 0;
    //   }
    // }
    // double target_x = goal_.path.poses[point_position].pose.position.x - current_pose.pose.position.x;
    // double target_y = goal_.path.poses[point_position].pose.position.y - current_pose.pose.position.y;
    // cmd_vel_.linear.x = goal_.speed * target_x / sqrt(target_x * target_x + target_y * target_y);
    // cmd_vel_.linear.y = goal_.speed * target_y / sqrt(target_x * target_x + target_y * target_y);
    // return Status::SUCCEEDED;
    cmd_vel_.linear.x = -goal_.speed;
    return Status::SUCCEEDED;
  }

  Status BackUpFollowPath::onCycleUpdate()
  {
    vel_pub_->publish(cmd_vel_);
    if (count_ >= max_count_)
    {
      count_ = 0;
      return Status::SUCCEEDED;
    }
    else
    {
      count_++;
      return Status::RUNNING;

    }
  }
  double BackUpFollowPath::get_distance(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2)
  {
    return sqrt((pose1.position.x - pose2.position.x) * (pose1.position.x - pose2.position.x) +
                (pose1.position.y - pose2.position.y) * (pose1.position.y - pose2.position.y));
  }
} // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUpFollowPath, nav2_core::Behavior)
