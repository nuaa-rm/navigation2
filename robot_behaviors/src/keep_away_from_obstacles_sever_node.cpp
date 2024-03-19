// 接收代价地图和footprint，将机器人朝着代价地图中的最低代价区域移动，直到机器人离障碍物足够远。

#include "nav2_msgs/action/keep_away_from_obstacles.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// 创建一个ActionServer类
class KeepAwayFromObstacles : public rclcpp::Node
{
public:
    explicit KeepAwayFromObstacles(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
        this->declare_parameter("occupied_threshold", 50);
        this->get_parameter("occupied_threshold", occupied_threshold_);
        this->declare_parameter("free_threshold", 0);
        this->get_parameter("free_threshold", free_threshold_);
        this->declare_parameter("costmap_topic", "costmap");
        this->get_parameter("costmap_topic", costmap_topic_);
        this->declare_parameter("map_frame", "map");
        this->get_parameter("map_frame", map_frame_);
        this->declare_parameter("robot_frame", "base_footprint");
        this->get_parameter("robot_frame", robot_frame_);
        this->declare_parameter("sleep_time", 100);
        this->get_parameter("sleep_time", sleep_time_);
        if_cancel_ = false;
        costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(costmap_topic_, 10, std::bind(&KeepAwayFromObstacles::cost_map_callback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // tfbuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // tflistener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
        this->action_server_ = rclcpp_action::create_server<nav2_msgs::action::KeepAwayFromObstacles>(
            this,
            "keep_away_from_obstacles",
            std::bind(&KeepAwayFromObstacles::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&KeepAwayFromObstacles::handle_cancel, this, std::placeholders::_1),
            std::bind(&KeepAwayFromObstacles::handle_accepted, this, std::placeholders::_1));
    }

private:
    std::string costmap_topic_;
    std::string map_frame_;
    std::string robot_frame_;
    int occupied_threshold_;
    int free_threshold_;
    int sleep_time_;
    rclcpp_action::Server<nav2_msgs::action::KeepAwayFromObstacles>::SharedPtr action_server_;
    nav2_msgs::msg::Costmap costmap_;
    double distance_;
    double speed_;
    bool if_cancel_;
    // std::unique_ptr<tf2_ros::Buffer> tfbuffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tflistener_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const nav2_msgs::action::KeepAwayFromObstacles::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "收到目标请求");
        distance_ = goal->distance;
        speed_ = goal->speed;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::KeepAwayFromObstacles>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        if_cancel_ = true;
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
        // 开始的时间
        auto start_time = this->now();
        geometry_msgs::msg::Twist cmd_vel;
        if (costmap_.data.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "代价地图为空");
            auto result = std::make_shared<nav2_msgs::action::KeepAwayFromObstacles::Result>();
            auto end_time = this->now();
            result->total_elapsed_time = (end_time - start_time);
            goal_handle->abort(result);
            return;
        }
        bool if_stuck = true;
        while (if_stuck)
        {
            if (if_cancel_)
            {
                auto result = std::make_shared<nav2_msgs::action::KeepAwayFromObstacles::Result>();
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel_pub_->publish(cmd_vel);
                auto end_time = this->now();
                result->total_elapsed_time = (end_time - start_time);
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "目标已取消");
                return;
            }
            // // 获取当前机器人位置
            // geometry_msgs::msg::TransformStamped transformStamped;
            // try
            // {
            //     transformStamped = tfbuffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePoint());
            // }
            // catch (tf2::TransformException &ex)
            // {
            //     RCLCPP_ERROR(this->get_logger(), "获取机器人位置失败: %s", ex.what());
            //     auto result = std::make_shared<nav2_msgs::action::KeepAwayFromObstacles::Result>();
            //     auto end_time = this->now();
            //     result->total_elapsed_time = (end_time - start_time);
            //     goal_handle->abort(result);
            //     return;
            // }

            // 查询机器人中心点有没有被占据
            int index = costmap_.metadata.size_x / 2 + costmap_.metadata.size_y / 2 * costmap_.metadata.size_y;
            if (costmap_.data[index] <= occupied_threshold_)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                if_stuck = false;
                cmd_vel_pub_->publish(cmd_vel);
                RCLCPP_INFO(this->get_logger(), "机器人没有被卡住");
            }
            else
            {
                // 获取代价地图中，距离机器人最近的没有被占据的点的坐标
                double min_distance = std::numeric_limits<double>::max();
                int min_index_x = 0;
                int min_index_y = 0;
                for (int i = 0; i < costmap_.data.size(); i++)
                {
                    if (costmap_.data[i] <= free_threshold_)
                    {
                        double x = costmap_.metadata.origin.position.x / costmap_.metadata.resolution + (i % costmap_.metadata.size_y);
                        double y = costmap_.metadata.origin.position.y / costmap_.metadata.resolution + (i / costmap_.metadata.size_y);
                        double distance = std::sqrt(std::pow(x - costmap_.metadata.size_x / 2, 2) + std::pow(y - costmap_.metadata.size_y / 2, 2));
                        if (distance < min_distance)
                        {
                            min_distance = distance;
                            min_index_x = x;
                            min_index_y = y;
                        }
                    }
                }
                RCLCPP_INFO(this->get_logger(), "机器人中心点的坐标：(%d, %d), 机器人中心点的代价值：%d, 最近的没有被占据的点的坐标：(%d, %d)", costmap_.metadata.size_x / 2, costmap_.metadata.size_y / 2, costmap_.data[index], min_index_x, min_index_y);
                cmd_vel.linear.x = speed_ * (min_index_x - costmap_.metadata.size_x / 2) / sqrt(pow(min_index_y - costmap_.metadata.size_y / 2, 2) + pow(min_index_x - costmap_.metadata.size_x / 2, 2));
                cmd_vel.linear.y = speed_ * (min_index_y - costmap_.metadata.size_y / 2) / sqrt(pow(min_index_y - costmap_.metadata.size_y / 2, 2) + pow(min_index_x - costmap_.metadata.size_x / 2, 2));
                cmd_vel.angular.z = 0;
                cmd_vel_pub_->publish(cmd_vel);
                // 休眠一段时间
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_));
            }
        }
        auto result = std::make_shared<nav2_msgs::action::KeepAwayFromObstacles::Result>();
        auto end_time = this->now();
        result->total_elapsed_time = (end_time - start_time);
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "目标已完成");
    }

    void cost_map_callback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        costmap_ = *msg;
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