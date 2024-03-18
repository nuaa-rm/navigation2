#include "nav2_behavior_tree/plugins/action/keep_away_from_obstacles_action.hpp"

#include "nav2_behavior_tree/plugins/action/back_up_follow_path_action.hpp"

namespace nav2_behavior_tree
{

    KeepAwayFromObstaclesAction::KeepAwayFromObstaclesAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf) : BtActionNode<nav2_msgs::action::KeepAwayFromObstacles>(xml_tag_name, action_name, conf)
    {
        getInput("speed", goal_.speed);
        getInput("distance", goal_.distance);
        getInput("time", goal_.time);
    }

    void KeepAwayFromObstaclesAction::on_tick()
    {
        increment_recovery_count();
        std::cout << "行为树成功调用 speed: " << goal_.speed << "distance: " << goal_.distance << "time: " << goal_.time << std::endl;
    }

} // namespace robot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::KeepAwayFromObstaclesAction>(
            name, "keep_away_from_obstacles", config);
    };

    factory.registerBuilder<nav2_behavior_tree::KeepAwayFromObstaclesAction>(
        "KeepAwayFromObstacles", builder);
}
