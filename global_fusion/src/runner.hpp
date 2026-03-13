#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace global_fusion
{

class GlobalFusionRunner
{
public:
    explicit GlobalFusionRunner(const rclcpp::NodeOptions & options);
    ~GlobalFusionRunner() = default;

    rclcpp::Node::SharedPtr node() const
    {
        return node_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_;
};

}  // namespace global_fusion
