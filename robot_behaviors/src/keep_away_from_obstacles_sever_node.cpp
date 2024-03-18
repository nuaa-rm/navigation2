#include "nav2_msgs/action/keep_away_from_obstacles.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
// 创建一个ActionServer类
class KeepAwayFromObstacles : public rclcpp::Node
{
public:
    explicit KeepAwayFromObstacles(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
        this->action_server_ = rclcpp_action::create_server<nav2_msgs::action::KeepAwayFromObstacles>(
            this,
            "keep_away_from_obstacles",
            std::bind(&KeepAwayFromObstacles::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&KeepAwayFromObstacles::handle_cancel, this, std::placeholders::_1),
            std::bind(&KeepAwayFromObstacles::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<nav2_msgs::action::KeepAwayFromObstacles>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const nav2_msgs::action::KeepAwayFromObstacles::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "收到目标请求");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::KeepAwayFromObstacles>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::KeepAwayFromObstacles>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "目标已接受");
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&KeepAwayFromObstacles::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::KeepAwayFromObstacles>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "开始执行目标");
        // 等待5秒，模拟执行过程
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "目标已完成");
        auto result = std::make_shared<nav2_msgs::action::KeepAwayFromObstacles::Result>();
        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<KeepAwayFromObstacles>("keep_away_from_obstacles");
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}