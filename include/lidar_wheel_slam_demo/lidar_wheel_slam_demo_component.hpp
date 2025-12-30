#pragma once

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/integrated_icp_factor.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace c107 {

struct Frame {
  using Ptr = std::shared_ptr<Frame>;
  gtsam_points::PointCloudCPU::Ptr cloud;
  Eigen::Isometry3d T_world_robot = Eigen::Isometry3d::Identity();
};

using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseStamped>;

using Sync = message_filters::Synchronizer<SyncPolicy>;

using gtsam::symbol_shorthand::X;  // Pose3

class LidarWheelSLAMDemo : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;

  std::shared_ptr<Sync> sync_ptr_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> wheel_odom_sub_;

  Eigen::Isometry3d cr_wheel_T_world_robot_;
  Eigen::Isometry3d pr_wheel_T_world_robot_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values estimate_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;

  std::vector<Frame::Ptr> frames_;

  bool approx_zero(const double val, const double epsilon = 1e-6) const { return std::fabs(val) < epsilon; }

  gtsam_points::PointCloudCPU::Ptr ros_cloud_convert(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*pc_msg, *pcl_cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl_cloud, *pcl_cloud, indices);

    std::vector<Eigen::Vector4d> points;
    points.resize(pcl_cloud->points.size());
    for (size_t i = 0; i < pcl_cloud->points.size(); i++) {
      const auto& p = pcl_cloud->points[i];
      points[i] = Eigen::Vector4d(p.x, p.y, p.z, 1.0);
    }
    return std::make_shared<gtsam_points::PointCloudCPU>(points);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_cpu_to_pcl(const gtsam_points::PointCloudCPU::Ptr& cloud_cpu) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl_cloud->points.resize(cloud_cpu->size());
    for (size_t i = 0; i < cloud_cpu->size(); i++) {
      const Eigen::Vector4d& p = cloud_cpu->points_storage[i];
      pcl_cloud->points[i].x = static_cast<float>(p.x());
      pcl_cloud->points[i].y = static_cast<float>(p.y());
      pcl_cloud->points[i].z = static_cast<float>(p.z());
    }
    return pcl_cloud;
  }

  static Eigen::Isometry3d ros_pose_convert(const geometry_msgs::msg::Pose& p) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    T.linear() = q.normalized().toRotationMatrix();
    return T;
  }

public:
  LidarWheelSLAMDemo(const rclcpp::NodeOptions& options) : LidarWheelSLAMDemo("", options) {}

  LidarWheelSLAMDemo(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("lidar_wheel_odom_demo", name_space, options) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("lidar_wheel_odom_demo/path", 10);
    map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_points", 10);

    point_cloud_sub_.subscribe(this, "/velodyne_points", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    wheel_odom_sub_.subscribe(this, "/pose", rclcpp::SensorDataQoS().get_rmw_qos_profile());

    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(10), point_cloud_sub_, wheel_odom_sub_);
    sync_ptr_->registerCallback(&LidarWheelSLAMDemo::sensor_data_cb, this);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LidarWheelSLAMDemo::timer_cb, this));
  }

  void timer_cb() {
    if (frames_.empty()) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = this->now();

    for (size_t i = 0; i < frames_.size(); i++) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "world";
      ps.header.stamp = this->now();

      const Eigen::Isometry3d& T = frames_[i]->T_world_robot;
      Eigen::Quaterniond q(T.linear());

      ps.pose.position.x = T.translation().x();
      ps.pose.position.y = T.translation().y();
      ps.pose.position.z = T.translation().z();
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();

      path_msg.poses.emplace_back(ps);
    }
    path_pub_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "map publish start");
    if (map_cloud_->points.size() > 0) {
      sensor_msgs::msg::PointCloud2 map_cloud_msg;
      pcl::toROSMsg(*map_cloud_, map_cloud_msg);
      map_cloud_msg.header.frame_id = "world";
      map_cloud_msg.header.stamp = this->get_clock()->now();
      map_cloud_pub_->publish(map_cloud_msg);
    }
  }

  void sensor_data_cb(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg, const geometry_msgs::msg::PoseStamped::SharedPtr odom_msg) {
    if (frames_.empty()) {
      Frame::Ptr init_frame = std::make_shared<Frame>();
      init_frame->cloud = ros_cloud_convert(pc_msg);
      init_frame->cloud = gtsam_points::voxelgrid_sampling(init_frame->cloud, 0.1, 4);
      init_frame->T_world_robot = Eigen::Isometry3d::Identity();
      frames_.emplace_back(init_frame);

      pr_wheel_T_world_robot_ = ros_pose_convert(odom_msg->pose);
      estimate_.insert(X(0), gtsam::Pose3());  // Identity

      // add prior
      auto prior_noise = gtsam::noiseModel::Isotropic::Precision(6, 1e6);
      graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(), prior_noise);
      optimize();
      return;
    }

    // update wheel odom
    cr_wheel_T_world_robot_ = ros_pose_convert(odom_msg->pose);
    Eigen::Isometry3d wheel_T_pr_cr = pr_wheel_T_world_robot_.inverse() * cr_wheel_T_world_robot_;

    if (approx_zero(wheel_T_pr_cr.translation().norm(), 0.1) && approx_zero(Eigen::AngleAxisd(wheel_T_pr_cr.linear()).angle(), 10.0 * M_PI / 180.0)) {
      return;
    }

    pr_wheel_T_world_robot_ = cr_wheel_T_world_robot_;

    // append new frame
    Frame::Ptr new_frame = std::make_shared<Frame>();
    new_frame->cloud = ros_cloud_convert(pc_msg);
    new_frame->cloud = gtsam_points::voxelgrid_sampling(new_frame->cloud, 0.1, 4);
    frames_.emplace_back(new_frame);
    const size_t k = frames_.size() - 1;
    const Eigen::Isometry3d init_T_world_robot = frames_[k - 1]->T_world_robot * wheel_T_pr_cr;

    // Add initial value for new variable to Values
    // estimate_.insert(X(k), gtsam::Pose3());
    estimate_.insert(X(k), gtsam::Pose3(init_T_world_robot.matrix()));

    auto between_noise = gtsam::noiseModel::Isotropic::Precision(6, 1.0);
    graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(k - 1), X(k), gtsam::Pose3(wheel_T_pr_cr.inverse().matrix()), between_noise);

    auto icp_factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(X(k - 1), X(k), frames_[k - 1]->cloud, new_frame->cloud);
    icp_factor->set_max_correspondence_distance(0.3);
    graph_.add(icp_factor);

    optimize();
    timer_cb();
  }

  void optimize() {
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(10);
    params.setVerbosityLM("SUMMARY");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, estimate_, params);
    gtsam::Values result = optimizer.optimize();
    estimate_ = result;

    // update frames
    for (const auto& key : estimate_.keys()) {
      gtsam::Symbol s(key);
      if (s.chr() != 'x') continue;
      const size_t idx = s.index();
      if (idx >= frames_.size()) continue;

      const gtsam::Pose3 est_pose = estimate_.at<gtsam::Pose3>(key);
      frames_[idx]->T_world_robot = Eigen::Isometry3d(est_pose.matrix());
    }

    // update map cloud
    map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    RCLCPP_INFO(this->get_logger(), "create map cloud");
    for (size_t i = 0; i < frames_.size(); i++) {
      gtsam_points::PointCloudCPU::Ptr cloud_lidar = frames_[i]->cloud;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      cloud_world->points.resize(cloud_lidar->size());
      for (size_t j = 0; j < cloud_lidar->size(); ++j) {
        const Eigen::Vector4d& p_lidar = cloud_lidar->points_storage[j];  // ←統一
        const Eigen::Vector4d p_world = frames_[i]->T_world_robot.matrix() * p_lidar;
        cloud_world->points[j].x = static_cast<float>(p_world.x());
        cloud_world->points[j].y = static_cast<float>(p_world.y());
        cloud_world->points[j].z = static_cast<float>(p_world.z());
      }

      RCLCPP_INFO(this->get_logger(), "kotekote");
      *map_cloud_ += *cloud_world;
    }

    RCLCPP_INFO(this->get_logger(), "Optimization done");
  }
};

}  // namespace c107
