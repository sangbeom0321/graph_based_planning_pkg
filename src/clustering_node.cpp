/**
 * @file cluster_node.cpp
 * @brief 노드 기능 설명: 포인트 클라우드 클러스터링, 보로노이 기반 시각화, OccupancyGrid 생성 등 수행
 * 
 * @author  우상범
 * @date    2025-04-08
 * @version 1.0
 * 
 * @details
 * 이 노드는 /lio_sam/mapping/map_local에서 수신한 PointCloud2 데이터를 필터링하고,
 * 클러스터링하여 나무 중심을 추출한 뒤,
 * OpenCV의 Subdiv2D를 통해 보로노이 다이어그램을 계산하고 시각화합니다.
 * OccupancyGrid로 나무 분포를 2D 맵에 투영하며, 각 처리 결과를 시각화용 토픽으로 퍼블리시합니다.
 * 
 * - 입력:  /lio_sam/mapping/map_local (sensor_msgs::msg::PointCloud2)
 * - 출력:  filtered_cloud, tree_markers, tree_map, voronoi_lines, voronoi_points, voronoi_points_array
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <set> 
#include "graph_based_planning_pkg/cluster_node.hpp"
#include "graph_based_planning_pkg/marker_utils.hpp"

ClusterNode::ClusterNode() : Node("cluster_node") {
  // Declare parameters with default values
  this->declare_parameter("x_min", 0.0);
  this->declare_parameter("x_max", 95.0);
  this->declare_parameter("y_min", -9.0);
  this->declare_parameter("y_max", 48.0);
  this->declare_parameter("z_min", 0.3);
  this->declare_parameter("z_max", 1.5);
  this->declare_parameter("treeClusterTolerance", 1.5);
  this->declare_parameter("treeClusterMinSize", 5);
  this->declare_parameter("treeClusterMaxSize", 10000);

  // Load parameter values
  this->get_parameter("x_min", x_min_);
  this->get_parameter("x_max", x_max_);
  this->get_parameter("y_min", y_min_);
  this->get_parameter("y_max", y_max_);
  this->get_parameter("z_min", z_min_);
  this->get_parameter("z_max", z_max_);
  this->get_parameter("treeClusterTolerance", tree_cluster_tolerance);
  this->get_parameter("treeClusterMinSize", tree_cluster_min_size);
  this->get_parameter("treeClusterMaxSize", tree_cluster_max_size);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lio_sam/mapping/map_global", 10,
    std::bind(&ClusterNode::cloud_callback, this, std::placeholders::_1));

  pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tree_markers", 10);
  pub_markers_outline_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/outline_markers", 10);
  pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/tree_map", 10);
  pub_lines_ = this->create_publisher<visualization_msgs::msg::Marker>("/voronoi_lines", 10);
  pub_voronoi_points_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/voronoi_points_array", 10);

  tree_map_.info.resolution = 0.2;
  tree_map_.info.width = (1/tree_map_.info.resolution)*(x_max_-x_min_);
  tree_map_.info.height = (1/tree_map_.info.resolution)*(y_max_-y_min_);
  tree_map_.info.origin.position.x = x_min_;
  tree_map_.info.origin.position.y = y_min_;
  tree_map_.info.origin.position.z = 0.0;
  tree_map_.info.origin.orientation.w = 1.0;
  tree_map_.data.resize(tree_map_.info.width * tree_map_.info.height, -1);
  tree_map_.header.frame_id = "odom";

}

void ClusterNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *input_cloud);

  // z-filtering & box filtering & delete z-demention
  pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto pt : input_cloud->points) {
    if (pt.z >= z_min_ && pt.z <= z_max_  &&
        pt.x >= x_min_ && pt.x <= x_max_ &&
        pt.y >= y_min_ && pt.y <= y_max_) {
      pt.z=0.0;
      z_filtered->points.push_back(pt);
    }
  }

  // Euclidean Clustering 시행 (KD-Tree based)
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(z_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tree_cluster_tolerance);
  ec.setMinClusterSize(tree_cluster_min_size);
  ec.setMaxClusterSize(tree_cluster_max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(z_filtered);
  ec.extract(cluster_indices);

  // Cluster Centroid 계산 & Occupancy Grid 투영
  visualization_msgs::msg::MarkerArray marker_array;
  std::vector<cv::Point2f> cluster_centers;
  int id = 0;
  for (const auto& indices : cluster_indices) {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (int idx : indices.indices) {
      const auto& pt = z_filtered->points[idx];
      sum_x += pt.x; sum_y += pt.y; sum_z += pt.z;
    }
    int n = indices.indices.size();
    float cx = sum_x / n, cy = sum_y / n, cz = sum_z / n;

    cluster_centers.emplace_back(cx, cy);

    // 클러스터 결과 시각화
    visualize_point_markers(cx, cy, cz, id++, msg->header, marker_array, "tree_centers");

    // 2차원 맵에 투영
    int mx = static_cast<int>((cx - tree_map_.info.origin.position.x) / tree_map_.info.resolution);
    int my = static_cast<int>((cy - tree_map_.info.origin.position.y) / tree_map_.info.resolution);
    if (mx >= 0 && mx < static_cast<int>(tree_map_.info.width) &&
        my >= 0 && my < static_cast<int>(tree_map_.info.height)) {
      int index = my * tree_map_.info.width + mx;
      tree_map_.data[index] = 100;
    }
  }

  // 위 아래 경계 테투리에 4m 간격 점 추가
  visualization_msgs::msg::MarkerArray outline_marker_array;
  int id_border=0;
  for (float y = y_min_ + 4.0f; y < y_max_; y += 4.0f) {
    std::vector<float> xs = {x_min_, x_max_};
    for (float x : xs) {
      cluster_centers.emplace_back(x, y);
      visualize_point_markers(x, y, 0.0, id_border++, msg->header, outline_marker_array, "tree_outline");
    }
  }
  
  // Voronoi 연산, 연산 가능 영역 넉넉하게 1씩 추가
  cv::Rect2f bounding_rect(x_min_-1, y_min_-1, std::abs(x_max_-x_min_)+2, std::abs(y_max_-y_min_)+2);
  cv::Subdiv2D subdiv(bounding_rect);
  for (auto& pt : cluster_centers) subdiv.insert(pt);

  std::vector<std::vector<cv::Point2f>> facets;
  std::vector<cv::Point2f> centers;
  subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);

  visualization_msgs::msg::Marker lines;
  lines.header.frame_id = msg->header.frame_id;
  lines.header.stamp = this->get_clock()->now();
  lines.ns = "voronoi";
  lines.id = 0;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.scale.x = 0.05;
  lines.color.r = 0.0;
  lines.color.g = 0.5;
  lines.color.b = 1.0;
  lines.color.a = 1.0;
  lines.lifetime = rclcpp::Duration::from_seconds(1.0);

  geometry_msgs::msg::PoseArray voronoi_points_array;
  voronoi_points_array.header.frame_id = msg->header.frame_id;
  voronoi_points_array.header.stamp = this->get_clock()->now();

  // 중복 방지를 위한 set (정수 기반 좌표 해시)
  std::set<std::pair<int, int>> unique_points;

  for (const auto& facet : facets) {
    for (const auto& pt : facet) {
      int ix = static_cast<int>(pt.x * 100);
      int iy = static_cast<int>(pt.y * 100);
      std::pair<int, int> key = {ix, iy};

      if (unique_points.find(key) != unique_points.end())
        continue;

      // 거리 기준으로 중복 여부 확인
      bool too_close = false;
      for (const auto& existing_pose : voronoi_points_array.poses) {
        float dx = existing_pose.position.x - pt.x;
        float dy = existing_pose.position.y - pt.y;
        float dist_sq = dx * dx + dy * dy;
        if (dist_sq < 1.0f * 1.0f) {
          too_close = true;
          break;
        }
      }
      // 중복도 없고 거리도 충분히 멀면 추가
      unique_points.insert(key);

      // 최종적으로 탐색 영역 안에 있는 좌표만 추가
      if ((x_min_ < pt.x && pt.x < x_max_) && (y_min_ < pt.y && pt.y < y_max_))  {
        geometry_msgs::msg::Pose p;
        p.position = make_point(pt.x, pt.y, 0.0);
        p.orientation.w = 1.0;
        voronoi_points_array.poses.push_back(p);
      }
    }

    // 시각화를 위한 반복문. (SKIP 가능)
    for (size_t i = 0; i < facet.size(); ++i) {
      const auto& pt1 = facet[i];
      const auto& pt2 = facet[(i + 1) % facet.size()];
      lines.points.push_back(make_point(pt1.x, pt1.y, 0.0));
      lines.points.push_back(make_point(pt2.x, pt2.y, 0.0));
    } 
  }


  tree_map_.header.stamp = this->get_clock()->now();
  pub_markers_->publish(marker_array);
  pub_markers_outline_->publish(outline_marker_array);
  pub_map_->publish(tree_map_);
  pub_lines_->publish(lines);
  pub_voronoi_points_->publish(voronoi_points_array);

  // z filter publish
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*z_filtered, output);
  output.header = msg->header;
  pub_filtered_->publish(output);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClusterNode>());
  rclcpp::shutdown();
  return 0;
}