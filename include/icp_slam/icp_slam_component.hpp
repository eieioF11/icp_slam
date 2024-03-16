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
// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
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
class ICPScanMatcher : public ExtensionNode {
public:
  ICPScanMatcher(const rclcpp::NodeOptions& options) : ICPScanMatcher("", options) {}
  ICPScanMatcher(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : ExtensionNode("icp_scan_matcher_node", name_space, options), broadcaster_(this), tf_buffer_(this->get_clock()), listener_(tf_buffer_) {
    RCLCPP_INFO(this->get_logger(), "start icp_scan_matcher_node");
    // get param
    std::string CLOUD_TOPIC = param<std::string>("icp_scan_matcher.topic_name.cloud", "/camera/depth_registered/points");
    std::string LASER_TOPIC = param<std::string>("icp_scan_matcher.topic_name.scan", "/scan");
    std::string IMU_TOPIC   = param<std::string>("icp_scan_matcher.topic_name.imu", "/wit_ros/imu_pose");
    // frame
    MAP_FRAME = param<std::string>("icp_scan_matcher.tf_frame.map_frame", "map");
    // ODOM_FRAME  = param<std::string>("icp_scan_matcher.tf_frame.odom_frame", "odom");
    // ROBOT_FRAME = param<std::string>("icp_scan_matcher.tf_frame.robot_frame", "base_link");
    // setup
    bool mode_2d     = param<bool>("icp_scan_matcher.2d", false);
    BROADCAST_PERIOD = param<double>("icp_scan_matcher.broadcast_period", 0.001);
    // 点群パラメータ
    MIN_CLOUD_SIZE = param<int>("icp_scan_matcher.min_point_cloud_size", 100);
    VOXELGRID_SIZE = param<double>("icp_scan_matcher.filter.voxelgrid_size", 0.04);
    // scan matchingパラメータ
    TARGET_VOXELGRID_SIZE   = param<double>("icp_scan_matcher.filter.target_voxelgrid_size", 0.04);
    TARGET_UPDATE_MIN_SCORE = param<double>("icp_scan_matcher.filter.target_update_min_score", 0.0005);
    MIN_SCORE_LIMIT         = param<double>("icp_scan_matcher.min_score_limit", 0.01);
    // init
    // publisher
    laser_pose_pub_      = this->create_publisher<geometry_msgs::msg::PoseStamped>("icp_scan_matcher/laser_pose", rclcpp::QoS(10).best_effort());
    debug_cloud_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/debug_points", rclcpp::QoS(10));
    now_cloud_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/now_points", rclcpp::QoS(10));
    old_cloud_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/old_points", rclcpp::QoS(10));
    icp_final_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/icp_final_points", rclcpp::QoS(10));
    // subscriber
    if (mode_2d) {
      laser_sub_ =
          this->create_subscription<sensor_msgs::msg::LaserScan>(LASER_TOPIC, rclcpp::QoS(10), [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            sensor_msgs::msg::PointCloud2 get_cloud;
            projector_.projectLaser(*msg, get_cloud);
            get_cloud.header = msg->header;
            icp_scan_matcher(get_cloud);
          });
    } else {
      cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          CLOUD_TOPIC, rclcpp::QoS(10).best_effort(), [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { icp_scan_matcher(*msg); });
    }
    // timer
    // main_timer_ = this->create_wall_timer(1ms, [&]() {

    // });
  }

private:
  bool update_cloud_;
  // param
  std::string MAP_FRAME;
  // std::string ROBOT_FRAME;
  // std::string ODOM_FRAME;
  double BROADCAST_PERIOD;
  int MIN_CLOUD_SIZE;
  double VOXELGRID_SIZE;
  double TARGET_VOXELGRID_SIZE;
  double MIN_SCORE_LIMIT;
  double TARGET_UPDATE_MIN_SCORE;
  // tf
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  // rclcpp::TimerBase::SharedPtr main_timer_;
  // subscriber
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  // publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr laser_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr now_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr old_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icp_final_cloud_pub_;
  // cloud
  pcl::PointCloud<pcl::PointXYZ> old_cloud_;
  // laser
  laser_geometry::LaserProjection projector_;
  // pose
  Pose3d laser_pose_;
  geometry_msgs::msg::PoseStamped laser_pose_msg_;

  // function
  void icp_scan_matcher(sensor_msgs::msg::PointCloud2& get_cloud) {
#if defined(DEBUG_OUTPUT)
    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "get cloud" << std::endl;
#endif
    std::optional<sensor_msgs::msg::PointCloud2> trans_cloud = transform_pointcloud2(tf_buffer_, MAP_FRAME, get_cloud);
    if (trans_cloud) {
      // msg convert
      pcl::PointCloud<pcl::PointXYZ> now_cloud;
      pcl::fromROSMsg(trans_cloud.value(), now_cloud);
      if (!now_cloud.empty()) {
        // nan値除去
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(now_cloud, now_cloud, mapping);
        //  Down sampling
        now_cloud = voxelgrid_filter(now_cloud, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
        now_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), now_cloud));
        // 外れ値除去
        debug_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), now_cloud));
#if defined(DEBUG_OUTPUT)
        std::cout << "now_cloud size:" << now_cloud.points.size() << "|old_cloud size:" << old_cloud_.points.size() << std::endl;
#endif
        if (now_cloud.points.size() > MIN_CLOUD_SIZE) {
          if (!old_cloud_.empty()) {
            old_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), old_cloud_));
            // ICP
            const auto& result = iterative_closest_point(now_cloud, old_cloud_);
            if (result) {
              auto& [score, tmat, final_cloud] = result.value();
              icp_final_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), final_cloud));
#if defined(DEBUG_OUTPUT)
              std::cout << "ICP has converged, score is " << score << std::endl;
#endif
              if (score < MIN_SCORE_LIMIT) {
#if defined(ICP_RESULT)
                std::cout << "rotation matrix" << std::endl;
                for (int i = 0; i < 3; ++i) {
                  std::cout << tmat(i, 0) << ", " << tmat(i, 1) << ", " << tmat(i, 2) << std::endl;
                }
                std::cout << std::endl;
                std::cout << "translation vector" << std::endl;
                std::cout << tmat(0, 3) << ", " << tmat(1, 3) << ", " << tmat(2, 3) << std::endl;
#endif
                // calc laser position
                // laser_pose_       = estimate_pose_;
                Vector3d rpy      = laser_pose_.orientation.get_rpy();
                Vector3d diff_rpy = {std::atan2(tmat(2, 1), tmat(2, 2)), -std::asin(tmat(2, 0)), std::atan2(tmat(1, 0), tmat(0, 0))};
                Vector3d diff_pos = {tmat(0, 3), tmat(1, 3), tmat(2, 3)};
                rpy += diff_rpy;
                laser_pose_.position += diff_pos;
#if defined(DEBUG_OUTPUT)
                std::cout << "diff_pos:" << diff_pos << "|diff_rpy:" << diff_rpy << std::endl;
                std::cout << "pos:" << laser_pose_.position << "|rpy" << rpy << std::endl;
                std::cout << "norm pos:" << diff_pos.norm() << "|rpy:" << diff_rpy.norm() << std::endl;
#endif
                update_target_cloud(score, final_cloud);
                update_cloud_ = false;
              }
            } else
              RCLCPP_ERROR(this->get_logger(), "ICP has not converged.");
          } else
            old_cloud_ = now_cloud;
        }
      } else
        RCLCPP_ERROR(this->get_logger(), "now_cloud empty");
    } else
      RCLCPP_ERROR(this->get_logger(), "transform error");
    // debug output
    laser_pose_msg_.header = make_header(MAP_FRAME, get_cloud.header.stamp);
    laser_pose_msg_.pose   = make_geometry_pose(laser_pose_);
    laser_pose_pub_->publish(laser_pose_msg_);
  }
  void update_target_cloud(double score, const pcl::PointCloud<pcl::PointXYZ>& input_cloud) {
    if (!update_cloud_) {
      if (score < TARGET_UPDATE_MIN_SCORE) {
        old_cloud_ += input_cloud;
        //  Down sampling
        old_cloud_    = voxelgrid_filter(old_cloud_, TARGET_VOXELGRID_SIZE, TARGET_VOXELGRID_SIZE, TARGET_VOXELGRID_SIZE);
        update_cloud_ = true;
      }
    }
  }
};