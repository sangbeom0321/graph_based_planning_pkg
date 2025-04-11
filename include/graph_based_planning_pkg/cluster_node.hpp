/**
 * @file cluster_node.hpp
 * @brief ClusterNode 클래스 정의 헤더 파일
 *
 * @author  우상범
 * @date    2025-04-08
 * @version 1.0
 *
 * @details
 * 이 클래스는 ROS2 노드로, LIO-SAM 기반의 로컬 맵에서 PointCloud를 받아 필터링하고,
 * 유클리디언 클러스터링을 통해 나무 중심점들을 추출합니다. 이후 OpenCV의 Subdiv2D를 이용해
 * 보로노이 다이어그램을 계산하고, OccupancyGrid 및 Marker 형태로 결과를 시각화합니다.
 * 
 * - 입력:  /lio_sam/mapping/map_local (sensor_msgs::msg::PointCloud2)
 * - 출력:  filtered_cloud, tree_markers, tree_map, voronoi_lines, voronoi_points_array
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class ClusterNode : public rclcpp::Node {
public:
  ClusterNode();

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  geometry_msgs::msg::Point make_point(float x, float y, float z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  float x_min_;
  float x_max_;
  float y_min_;
  float y_max_;
  float z_min_;
  float z_max_;
  float tree_cluster_tolerance;
  int tree_cluster_min_size;
  int tree_cluster_max_size;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_outline_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_lines_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_voronoi_points_;
  
  nav_msgs::msg::OccupancyGrid tree_map_;
};
