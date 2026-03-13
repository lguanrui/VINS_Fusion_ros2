#pragma once

#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

namespace loop_fusion
{

class LoopFusionRunner
{
public:
    explicit LoopFusionRunner(const rclcpp::NodeOptions & options);
    ~LoopFusionRunner();

    rclcpp::Node::SharedPtr node() const
    {
        return node_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool enable_keyboard_commands_{false};
    std::thread measurement_thread_;
    std::thread keyboard_thread_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_extrinsic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_point_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_margin_point_;
};

}  // namespace loop_fusion
