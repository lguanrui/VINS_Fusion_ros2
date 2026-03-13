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
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "estimator/estimator.h"
#include "runners.hpp"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

Estimator estimator;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGPS;


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

KittiGpsTestRunner::KittiGpsTestRunner(const rclcpp::NodeOptions & options)
{
	node_ = rclcpp::Node::make_shared("kitti_gps_test", options);
	worker_thread_ = std::thread(&KittiGpsTestRunner::run, this);
}

KittiGpsTestRunner::~KittiGpsTestRunner()
{
	wait();
}

void KittiGpsTestRunner::wait()
{
	if (worker_thread_.joinable())
		worker_thread_.join();
}

void KittiGpsTestRunner::run()
{
	pub_gps_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("/gps", 1000);
	pubGPS = pub_gps_;

	const auto config_file = node_->declare_parameter<std::string>("config_file", "");
	const auto sequence = node_->declare_parameter<std::string>("sequence_path", "");
	ensure_parameter_set(config_file, "config_file");
	ensure_parameter_set(sequence, "sequence_path");

	printf("config_file: %s\n", config_file.c_str());
	printf("read sequence: %s\n", sequence.c_str());
	string dataPath = sequence + "/";

	FILE * file = std::fopen((dataPath + "image_00/timestamps.txt").c_str(), "r");
	if (file == NULL)
	{
		printf("cannot find file: %simage_00/timestamps.txt \n", dataPath.c_str());
		return;
	}

	vector<double> imageTimeList;
	int year, month, day;
	int hour, minute;
	double second;
	while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
		imageTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
	std::fclose(file);

	vector<double> GPSTimeList;
	{
		FILE * gps_time_file = std::fopen((dataPath + "oxts/timestamps.txt").c_str(), "r");
		if (gps_time_file == NULL)
		{
			printf("cannot find file: %soxts/timestamps.txt \n", dataPath.c_str());
			return;
		}
		while (fscanf(gps_time_file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
			GPSTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
		std::fclose(gps_time_file);
	}

	readParameters(config_file);
	estimator.setParameter();
	registerPub(node_);

	FILE * outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(), "w");
	if (outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	double baseTime = imageTimeList[0] < GPSTimeList[0] ? imageTimeList[0] : GPSTimeList[0];

	for (size_t i = 0; i < imageTimeList.size() && rclcpp::ok(); i++)
	{
		printf("process image %d\n", static_cast<int>(i));
		stringstream ss;
		ss << setfill('0') << setw(10) << i;
		leftImagePath = dataPath + "image_00/data/" + ss.str() + ".png";
		rightImagePath = dataPath + "image_01/data/" + ss.str() + ".png";
		printf("%s\n", leftImagePath.c_str());
		printf("%s\n", rightImagePath.c_str());

		imLeft = cv::imread(leftImagePath, cv::IMREAD_GRAYSCALE);
		imRight = cv::imread(rightImagePath, cv::IMREAD_GRAYSCALE);
		double imgTime = imageTimeList[i] - baseTime;

		FILE * GPSFile = std::fopen((dataPath + "oxts/data/" + ss.str() + ".txt").c_str(), "r");
		if (GPSFile == NULL)
		{
			printf("cannot find file: %s\n", (dataPath + "oxts/data/" + ss.str() + ".txt").c_str());
			if (outFile != NULL)
				fclose(outFile);
			return;
		}

		double lat, lon, alt, roll, pitch, yaw;
		double vn, ve, vf, vl, vu;
		double ax, ay, az, af, al, au;
		double wx, wy, wz, wf, wl, wu;
		double pos_accuracy, vel_accuracy;
		double navstat, numsats;
		double velmode, orimode;

		fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll, &pitch, &yaw);
		fscanf(GPSFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
		fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al, &au);
		fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl, &wu);
		fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy, &vel_accuracy, &navstat, &numsats, &velmode, &orimode);
		std::fclose(GPSFile);

		sensor_msgs::msg::NavSatFix gps_position;
		gps_position.header.frame_id = "NED";
		int sec_ts = static_cast<int>(imgTime);
		uint nsec_ts = static_cast<uint>((imgTime - sec_ts) * 1e9);
		gps_position.header.stamp.sec = sec_ts;
		gps_position.header.stamp.nanosec = nsec_ts;
		gps_position.status.status = navstat;
		gps_position.status.service = numsats;
		gps_position.latitude = lat;
		gps_position.longitude = lon;
		gps_position.altitude = alt;
		gps_position.position_covariance[0] = pos_accuracy;
		pubGPS->publish(gps_position);

		estimator.inputImage(imgTime, imLeft, imRight);

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

class KittiGpsTestComponent : public rclcpp::Node
{
public:
	explicit KittiGpsTestComponent(const rclcpp::NodeOptions & options)
		: rclcpp::Node("kitti_gps_test_component_host", make_host_options(options)),
		  runner_(std::make_unique<KittiGpsTestRunner>(options))
	{
	}

private:
	std::unique_ptr<KittiGpsTestRunner> runner_;
};

}  // namespace vins

RCLCPP_COMPONENTS_REGISTER_NODE(vins::KittiGpsTestComponent)
