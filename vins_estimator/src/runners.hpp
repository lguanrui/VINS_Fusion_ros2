#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/bool.hpp>

namespace vins
{

class VinsEstimatorRunner
{
public:
    explicit VinsEstimatorRunner(const rclcpp::NodeOptions & options);
    ~VinsEstimatorRunner();

    rclcpp::Node::SharedPtr node() const
    {
        return node_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::thread sync_thread_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_feature_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img0_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img1_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_restart_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_imu_switch_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cam_switch_;
};

class KittiOdomTestRunner
{
public:
    explicit KittiOdomTestRunner(const rclcpp::NodeOptions & options);
    ~KittiOdomTestRunner();

    rclcpp::Node::SharedPtr node() const
    {
        return node_;
    }

    void wait();

private:
    void run();

    rclcpp::Node::SharedPtr node_;
    std::thread worker_thread_;
};

class KittiGpsTestRunner
{
public:
    explicit KittiGpsTestRunner(const rclcpp::NodeOptions & options);
    ~KittiGpsTestRunner();

    rclcpp::Node::SharedPtr node() const
    {
        return node_;
    }

    void wait();

private:
    void run();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
    std::thread worker_thread_;
};

}  // namespace vins
