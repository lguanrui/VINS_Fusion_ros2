/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <atomic>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "runners.hpp"
#include "utility/visualization.h"

Estimator estimator;

queue<sensor_msgs::msg::Imu::ConstPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::ConstPtr> feature_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img0_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img1_buf;
std::mutex m_buf;
std::atomic<bool> vins_estimator_running{false};

// header: 1403715278
void img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_buf.lock();
    // std::cout << "Left : " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << endl;
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_buf.lock();
    // std::cout << "Right: " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << endl;
    img1_buf.push(img_msg);
    m_buf.unlock();
}


// cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::SharedPtr img_msg)
cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(vins_estimator_running.load() && rclcpp::ok())
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                double time1 = img1_buf.front()->header.stamp.sec + img1_buf.front()->header.stamp.nanosec * (1e-9);

                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");

                    // std::cout << std::fixed << img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9) << std::endl;
                    // assert(0);
                    
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    // std::cout << "IMU cb" << std::endl;

    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * (1e-9);
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);

    // std::cout << "got t_imu: " << std::fixed << t << endl;
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 8)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        assert(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.sec + feature_msg->header.stamp.nanosec * (1e-9);
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

namespace
{

void clear_vins_estimator_buffers()
{
    std::lock_guard<std::mutex> lock(m_buf);
    while (!imu_buf.empty())
        imu_buf.pop();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!img0_buf.empty())
        img0_buf.pop();
    while (!img1_buf.empty())
        img1_buf.pop();
}

rclcpp::NodeOptions make_host_options(const rclcpp::NodeOptions & options)
{
    auto host_options = options;
    host_options.arguments({});
    return host_options;
}

rclcpp::NodeOptions make_worker_options(const rclcpp::NodeOptions & options)
{
    auto worker_options = options;
    worker_options.use_intra_process_comms(true);
    return worker_options;
}

rclcpp::ExecutorOptions make_executor_options(const rclcpp::NodeOptions & options)
{
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = options.context();
    return executor_options;
}

}  // namespace

namespace vins
{

VinsEstimatorRunner::VinsEstimatorRunner(const rclcpp::NodeOptions & options)
{
    clear_vins_estimator_buffers();
    estimator.clearState();

    node_ = rclcpp::Node::make_shared("vins_estimator", options);
    const auto config_file = node_->declare_parameter<std::string>("config_file", "");
    if (config_file.empty())
    {
        throw std::invalid_argument("Parameter 'config_file' is required for vins_estimator.");
    }

    printf("config_file: %s\n", config_file.c_str());
    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(node_);

    if (USE_IMU)
    {
        sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)), imu_callback);
    }
    sub_feature_ = node_->create_subscription<sensor_msgs::msg::PointCloud>(
        "/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)), feature_callback);
    sub_img0_ = node_->create_subscription<sensor_msgs::msg::Image>(
        IMAGE0_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img0_callback);

    if (STEREO)
    {
        sub_img1_ = node_->create_subscription<sensor_msgs::msg::Image>(
            IMAGE1_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img1_callback);
    }

    sub_restart_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/vins_restart", rclcpp::QoS(rclcpp::KeepLast(100)), restart_callback);
    sub_imu_switch_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/vins_imu_switch", rclcpp::QoS(rclcpp::KeepLast(100)), imu_switch_callback);
    sub_cam_switch_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/vins_cam_switch", rclcpp::QoS(rclcpp::KeepLast(100)), cam_switch_callback);

    vins_estimator_running.store(true);
    sync_thread_ = std::thread(sync_process);
}

VinsEstimatorRunner::~VinsEstimatorRunner()
{
    vins_estimator_running.store(false);
    if (sync_thread_.joinable())
        sync_thread_.join();
}

class VinsEstimatorComponent : public rclcpp::Node
{
public:
    explicit VinsEstimatorComponent(const rclcpp::NodeOptions & options)
        : rclcpp::Node("vins_estimator_component_host", make_host_options(options)),
          runner_(std::make_unique<VinsEstimatorRunner>(make_worker_options(options))),
          executor_(make_executor_options(options))
    {
        executor_.add_node(runner_->node());
        spin_thread_ = std::thread([this]() { executor_.spin(); });
    }

    ~VinsEstimatorComponent() override
    {
        executor_.cancel();
        if (spin_thread_.joinable())
            spin_thread_.join();
        runner_.reset();
    }

private:
    std::unique_ptr<VinsEstimatorRunner> runner_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
};

}  // namespace vins

RCLCPP_COMPONENTS_REGISTER_NODE(vins::VinsEstimatorComponent)
