#ifndef BACK_UP_FOLLOW_PATH_ACTION_HPP
#define BACK_UP_FOLLOW_PATH_ACTION_HPP
#include <string>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include "nav2_msgs/action/back_up_follow_path.hpp"
#include <nav_msgs/msg/path.hpp>


// Include any necessary headers here

namespace nav2_behavior_tree
{

    class BackUpFollowPathAction : public BtActionNode<nav2_msgs::action::BackUpFollowPath>
    {
    public:
        // Constructor and destructor
        BackUpFollowPathAction(
            const std::string &xml_tag_name,
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        // Public member functions
        void on_tick() override;
        static BT::PortsList providedPorts()
        {
            return providedBasicPorts(
                {
                    BT::InputPort<double>("speed", 1.0, "Target speed of the robot"),
                    BT::InputPort<double>("distance", 0.5, "Back up distance for the direction"),
                    BT::InputPort<double>("time", 1.0, "Back up time"),
                    BT::InputPort<nav_msgs::msg::Path>("path", "Path to be followed"),
                });
        }

    private:
        // Private member variables and functions
    };

} // namespace robot_behavior_tree

#endif // BACK_UP_FOLLOW_PATH_ACTION_HPP
