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

#include <iostream>
#include <stdio.h>
#include <stdexcept>
#include <thread>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "runners.hpp"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;

namespace
{
void ensure_parameter_set(const std::string & value, const char * name)
{
	if (value.empty())
		throw std::invalid_argument(std::string("Parameter '") + name + "' is required.");
}

rclcpp::NodeOptions make_host_options(const rclcpp::NodeOptions & options)
{
	auto host_options = options;
	host_options.arguments({});
	return host_options;
}

}  // namespace

namespace vins
{

KittiOdomTestRunner::KittiOdomTestRunner(const rclcpp::NodeOptions & options)
{
	node_ = rclcpp::Node::make_shared("kitti_odom_test", options);
	worker_thread_ = std::thread(&KittiOdomTestRunner::run, this);
}

KittiOdomTestRunner::~KittiOdomTestRunner()
{
	wait();
}

void KittiOdomTestRunner::wait()
{
	if (worker_thread_.joinable())
		worker_thread_.join();
}

void KittiOdomTestRunner::run()
{
	auto pubLeftImage = node_->create_publisher<sensor_msgs::msg::Image>("/leftImage", 1000);
	auto pubRightImage = node_->create_publisher<sensor_msgs::msg::Image>("/rightImage", 1000);

	const auto config_file = node_->declare_parameter<std::string>("config_file", "");
	const auto sequence = node_->declare_parameter<std::string>("sequence_path", "");
	ensure_parameter_set(config_file, "config_file");
	ensure_parameter_set(sequence, "sequence_path");

	printf("config_file: %s\n", config_file.c_str());
	printf("read sequence: %s\n", sequence.c_str());
	string dataPath = sequence + "/";

	readParameters(config_file);
	estimator.setParameter();
	registerPub(node_);

	FILE * file = std::fopen((dataPath + "times.txt").c_str(), "r");
	if (file == NULL)
	{
		printf("cannot find file: %stimes.txt\n", dataPath.c_str());
		return;
	}

	double imageTime;
	vector<double> imageTimeList;
	while (fscanf(file, "%lf", &imageTime) != EOF)
		imageTimeList.push_back(imageTime);
	std::fclose(file);

	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	FILE * outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(), "w");
	if (outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

	for (size_t i = 0; i < imageTimeList.size() && rclcpp::ok(); i++)
	{
		printf("\nprocess image %d\n", static_cast<int>(i));
		stringstream ss;
		ss << setfill('0') << setw(6) << i;
		leftImagePath = dataPath + "image_0/" + ss.str() + ".png";
		rightImagePath = dataPath + "image_1/" + ss.str() + ".png";

		imLeft = cv::imread(leftImagePath, cv::IMREAD_GRAYSCALE);
		auto imLeftMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imLeft).toImageMsg();
		imLeftMsg->header.stamp = rclcpp::Time(imageTimeList[i]);
		pubLeftImage->publish(*imLeftMsg);

		imRight = cv::imread(rightImagePath, cv::IMREAD_GRAYSCALE);
		auto imRightMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imRight).toImageMsg();
		imRightMsg->header.stamp = rclcpp::Time(imageTimeList[i]);
		pubRightImage->publish(*imRightMsg);

		estimator.inputImage(imageTimeList[i], imLeft, imRight);

		Eigen::Matrix<double, 4, 4> pose;
		estimator.getPoseInWorldFrame(pose);
		if (outFile != NULL)
		{
			fprintf(
				outFile,
				"%f %f %f %f %f %f %f %f %f %f %f %f \n",
				pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
				pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
				pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3));
		}
	}

	if (outFile != NULL)
		fclose(outFile);
}

class KittiOdomTestComponent : public rclcpp::Node
{
public:
	explicit KittiOdomTestComponent(const rclcpp::NodeOptions & options)
		: rclcpp::Node("kitti_odom_test_component_host", make_host_options(options)),
		  runner_(std::make_unique<KittiOdomTestRunner>(options))
	{
	}

private:
	std::unique_ptr<KittiOdomTestRunner> runner_;
};

}  // namespace vins

RCLCPP_COMPONENTS_REGISTER_NODE(vins::KittiOdomTestComponent)
