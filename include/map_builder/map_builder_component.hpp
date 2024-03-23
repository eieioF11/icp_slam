#pragma once
#include <execution>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
// common_lib
#include "common_lib/common_lib.hpp"
#include "common_lib/pcl_utility/pcl_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/ros_pcl_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "extension_node/extension_node.hpp"
// OpenMP
#include <omp.h>
#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
// デバック関連設定
#define DEBUG_OUTPUT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
namespace icp_slam {
  class MapBuilder : public ExtensionNode {
  public:
    MapBuilder(const rclcpp::NodeOptions& options) : MapBuilder("", options) {}
    MapBuilder(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : ExtensionNode("map_builder_node", name_space, options), broadcaster_(this), tf_buffer_(this->get_clock()), listener_(tf_buffer_) {
      RCLCPP_INFO(this->get_logger(), "start map_builder_node");
      // get param
      std::string CLOUD_TOPIC = param<std::string>("map_builder.topic_name.cloud", "icp_scan_matcher/icp_final_points");
      // frame
      MAP_FRAME   = param<std::string>("map_builder.tf_frame.map_frame", "map");
      ROBOT_FRAME = param<std::string>("map_builder.tf_frame.robot_frame", "base_link");
      // setup
      BROADCAST_PERIOD = param<double>("map_builder.broadcast_period", 0.001);
      // 点群パラメータ
      MIN_CLOUD_SIZE = param<int>("map_builder.min_point_cloud_size", 100);
      VOXELGRID_SIZE = param<double>("map_builder.filter.voxelgrid_size", 0.04);
      // publisher
      ref_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_slam/ref_points", rclcpp::QoS(10));
      // subscriber
      cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          CLOUD_TOPIC, rclcpp::QoS(10).best_effort(), [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { input_cloud_ = *msg; });
      // timer
      main_timer_ = this->create_wall_timer(BROADCAST_PERIOD * 1s, [&]() {
        if (!input_cloud_) return;
        pcl::PointCloud<pcl::PointNormal> cloud;
        pcl::fromROSMsg(input_cloud_.value(), cloud);
        if (cloud.empty()) {
          RCLCPP_ERROR(this->get_logger(), "cloud empty");
          return;
        }
        // nan値除去
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(cloud, cloud, mapping);
        // map 追加
        ref_clouds_.push_back(cloud);
        pcl::PointCloud<pcl::PointNormal> ref_cloud;
        ref_cloud = pcl::PointCloud<pcl::PointNormal>();
        for (auto& cloud : ref_clouds_) {
          ref_cloud += cloud;
        }
        //  Down sampling
        ref_cloud = voxelgrid_filter<pcl::PointNormal>(ref_cloud, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
        ref_cloud_pub_->publish(make_ros_pointcloud2<pcl::PointNormal>(make_header(MAP_FRAME, input_cloud_.value().header.stamp), ref_cloud));
        input_cloud_ = std::nullopt;
      });
    }

  private:
    bool update_cloud_;
    // param
    std::string MAP_FRAME;
    std::string ROBOT_FRAME;
    double BROADCAST_PERIOD;
    int MIN_CLOUD_SIZE;
    double VOXELGRID_SIZE;
    // tf
    tf2_ros::TransformBroadcaster broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
    // timer
    rclcpp::TimerBase::SharedPtr main_timer_;
    // subscriber
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ref_cloud_pub_;
    // cloud
    std::optional<sensor_msgs::msg::PointCloud2> input_cloud_;
    std::vector<pcl::PointCloud<pcl::PointNormal>> ref_clouds_;
  };
} // namespace icp_slam
