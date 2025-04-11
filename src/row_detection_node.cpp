#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>
#include <string>
#include <algorithm>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>

#include <Eigen/Core>
#include <Eigen/Dense>

struct linear_curve
{
  pcl::PointXYZ point;
  Eigen::Vector3f eigen_vector;
};

class OrchardRowEndpointMarkerProcessor : public rclcpp::Node
{
public:
  OrchardRowEndpointMarkerProcessor()
  : Node("orchard_row_endpoint_marker_processor")
  {
    // 알고리즘 파라미터 선언 (원본 상수값 유지)
    this->declare_parameter("x_min", 0.0);
    this->declare_parameter("x_max", 95.0);
    this->declare_parameter("y_min", -9.0);
    this->declare_parameter("y_max", 48.0);
    this->declare_parameter("rowClusterTolerance", 4.0);
    this->declare_parameter("rowClusterMinSize", 4);
    this->declare_parameter("rowClusterMaxSize", 70);
    this->declare_parameter("cosThetaThreshold", 0.9);
    this->declare_parameter("dot2LineThreshold", 2.0);
    
    this->get_parameter("x_min", x_min_);
    this->get_parameter("x_max", x_max_);
    this->get_parameter("y_min", y_min_);
    this->get_parameter("y_max", y_max_);
    this->get_parameter("rowClusterTolerance", cluster_tolerance);
    this->get_parameter("rowClusterMinSize", cluster_min_size);
    this->get_parameter("rowClusterMaxSize", cluster_max_size);
    this->get_parameter("cosThetaThreshold", cos_theta_threshold);
    this->get_parameter("dot2LineThreshold", dot2line_threshold);

    // MarkerArray 메시지를 "tree_markers" 토픽에서 구독합니다.
    marker_array_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/tree_markers", 10,
      std::bind(&OrchardRowEndpointMarkerProcessor::markerArrayCallback, this, std::placeholders::_1));
    
    // outline_markers를 구독하여 경계선 포인트들을 받을 수 있도록 합니다.
    outline_marker_array_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/outline_markers", 10,
      std::bind(&OrchardRowEndpointMarkerProcessor::outlineMarkerArrayCallback, this, std::placeholders::_1));
    
    // 최종 endpoint 마커(예: LINE_LIST)를 "line_segments" 토픽으로 발행합니다.
    // (이 예제에서는 단일 Marker 메시지를 발행)
    endpoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/line_segments", 10);


  }

private:
  // outline 마커를 구독하여 경계선 포인트들을 저장 (실제 경계선 생성에 활용 가능)
  void outlineMarkerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
  
    for (const auto &marker : msg->markers) {
      outline_cloud->push_back(pcl::PointXYZ(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr find_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(cluster_min_size);
    ec.setMaxClusterSize(cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    int j = 0;
    for (const auto &cluster : cluster_indices) {
      for (const auto &idx : cluster.indices) {
        pcl::PointXYZI point;
        point.x = (*cloud)[idx].x;
        point.y = (*cloud)[idx].y;
        point.z = (*cloud)[idx].z;
        point.intensity = static_cast<float>(j);
        cloud_cluster->push_back(point);
      }
      cloud_cluster->width = cloud_cluster->size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      j++;
    }
    return cloud_cluster;
  }

  linear_curve pca_fitting_linear_curve(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    // 2D 투영: 모든 포인트의 Z값을 0으로 설정
    for (auto &point : *cloud)
      point.z = 0.0;
    linear_curve curve_fittied;

    // PCA 수행
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    if ((eigen_vectors.col(0)[0] < 0.0) && (eigen_vectors.col(0)[1] < 0.0)) {
      eigen_vectors.col(0)[0] = -eigen_vectors.col(0)[0];
      eigen_vectors.col(0)[1] = -eigen_vectors.col(0)[1];
    }
    if ((eigen_vectors.col(0)[0] < 0.0) && (eigen_vectors.col(0)[1] > 0.0)) {
      eigen_vectors.col(0)[0] = -eigen_vectors.col(0)[0];
      eigen_vectors.col(0)[1] = -eigen_vectors.col(0)[1];
    }
    // 원본 알고리즘: 두 번째 포인트를 대표 포인트로 사용
    if(cloud->points.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Not enough points for PCA");
      return curve_fittied;
    }
    curve_fittied.point.x = (*cloud)[1].x;
    curve_fittied.point.y = (*cloud)[1].y;
    curve_fittied.point.z = (*cloud)[1].z;
    curve_fittied.eigen_vector[0] = eigen_vectors.col(0)[0];
    curve_fittied.eigen_vector[1] = eigen_vectors.col(0)[1];
    curve_fittied.eigen_vector[2] = eigen_vectors.col(0)[2];

    return curve_fittied;
  }

  bool linear_curve_is_same(linear_curve a_line, linear_curve b_line)
  {
    float dot_product = a_line.eigen_vector.dot(b_line.eigen_vector);
    float cos_theta = std::abs(dot_product / (a_line.eigen_vector.norm() * b_line.eigen_vector.norm()));
    if (cos_theta > cos_theta_threshold) {
      Eigen::Vector3f A2B(a_line.point.x - b_line.point.x,
                           a_line.point.y - b_line.point.y,
                           a_line.point.z - b_line.point.z);
      Eigen::Vector3f crossProduct = A2B.cross(a_line.eigen_vector);
      float dist_between_dot2line = crossProduct.norm() / a_line.eigen_vector.norm();
      if (dist_between_dot2line < dot2line_threshold)
        return true;
    }
    return false;
  }

  int find_set(int element)
  {
    if (element_to_set[element] == element)
      return element;
    else
      return element_to_set[element] = find_set(element_to_set[element]);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr find_tree_line(pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud)
  {
    cloudsById.clear();
    for (const auto &point : *clustered_cloud) {
      int id = static_cast<int>(point.intensity);
      if (cloudsById.find(id) == cloudsById.end())
        cloudsById[id] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
      cloudsById[id]->push_back(point);
    }

    std::vector<linear_curve> curves;
    for (size_t i = 0; i < cloudsById.size(); i++) {
      curves.push_back(pca_fitting_linear_curve(cloudsById[i]));
    }

    std::unordered_map<int, std::vector<int>> id_candidates;
    for (size_t i = 0; i < curves.size(); i++)
      element_to_set[i] = i;

    for (size_t i = 0; i < curves.size(); i++) {
      for (size_t j = i + 1; j < curves.size(); j++) {
        if (linear_curve_is_same(curves[i], curves[j]))
          element_to_set[find_set(j)] = find_set(i);
      }
    }

    for (size_t i = 0; i < curves.size(); i++) {
      int set_id = find_set(i);
      id_candidates[set_id].push_back(i);
    }

    for (size_t i = 0; i < curves.size(); i++) {
      if (id_candidates.find(i) != id_candidates.end()) {
        for (const auto &j : id_candidates[i]) {
          if (i == j) continue;
          if (cloudsById.find(j) != cloudsById.end()) {
            auto &cloud_j = cloudsById[j];
            for (auto &point : *cloud_j)
              point.intensity = static_cast<float>(i);
            if (cloudsById.find(i) != cloudsById.end())
              cloudsById[i]->insert(cloudsById[i]->end(), cloud_j->begin(), cloud_j->end());
            cloudsById.erase(j);
          }
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &pair : cloudsById) {
      for (auto &point : *(pair.second))
        mergedCloud->push_back(point);
    }
    return mergedCloud;
  }



  // MarkerArray Callback: tree_markers의 MarkerArray를 입력받아 endpoint를 계산하고 선분(Line List) Marker를 발행
  void markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // 람다 함수: 주어진 원점(origin)와 단위 방향(dir)에 대해, 
    // 2D 경계 사각형([x_min_, x_max_] x [y_min_, y_max_])과의 ray intersection을 계산
    auto getExtendedEndpoint = [&](const Eigen::Vector2f &origin, const Eigen::Vector2f &dir) -> Eigen::Vector2f {
        float t_candidate = std::numeric_limits<float>::max();
        // 수직 경계 체크 (x 방향)
        if (std::fabs(dir.x()) > 1e-6) {
        float t1 = (x_min_ - origin.x()) / dir.x();
        if (t1 > 0) {
            Eigen::Vector2f p = origin + t1 * dir;
            if (p.y() >= y_min_ && p.y() <= y_max_) {
            t_candidate = std::min(t_candidate, t1);
            }
        }
        float t2 = (x_max_ - origin.x()) / dir.x();
        if (t2 > 0) {
            Eigen::Vector2f p = origin + t2 * dir;
            if (p.y() >= y_min_ && p.y() <= y_max_) {
            t_candidate = std::min(t_candidate, t2);
            }
        }
        }
        // 수평 경계 체크 (y 방향)
        if (std::fabs(dir.y()) > 1e-6) {
        float t3 = (y_min_ - origin.y()) / dir.y();
        if (t3 > 0) {
            Eigen::Vector2f p = origin + t3 * dir;
            if (p.x() >= x_min_ && p.x() <= x_max_) {
            t_candidate = std::min(t_candidate, t3);
            }
        }
        float t4 = (y_max_ - origin.y()) / dir.y();
        if (t4 > 0) {
            Eigen::Vector2f p = origin + t4 * dir;
            if (p.x() >= x_min_ && p.x() <= x_max_) {
            t_candidate = std::min(t_candidate, t4);
            }
        }
        }
        return origin + t_candidate * dir;
    };
    // 모든 Marker의 points를 모아서 하나의 pcl::PointCloud<pcl::PointXYZ> 생성
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &marker : msg->markers) {
      input_cloud->push_back(pcl::PointXYZ(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
    }
    if (input_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received MarkerArray with no points");
      return;
    }

    // 1. find_cluster: 입력 클라우드에 대해 클러스터링 수행
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud = find_cluster(input_cloud);
    // 2. find_tree_line: 클러스터들을 병합하여 tree‑line 그룹 생성
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud = find_tree_line(clustered_cloud);

    // 3. merged_cloud를 intensity(정수)별로 그룹화
    std::unordered_map<int, std::vector<pcl::PointXYZ>> tree_line_groups;
    for (const auto &pt : merged_cloud->points) {
      int id = static_cast<int>(pt.intensity);
      tree_line_groups[id].push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }

    // 새 Marker 생성 (LINE_LIST 타입)
    visualization_msgs::msg::Marker output_marker;
    std_msgs::msg::Header header = msg->markers.front().header;  // 첫 Marker의 header 사용
    header.frame_id = "map";
    output_marker.header = header;
    output_marker.ns = "segments";
    output_marker.id = 0;
    output_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    output_marker.action = visualization_msgs::msg::Marker::ADD;
    output_marker.scale.x = 0.1;  // 선 굵기
    output_marker.color.r = 0.59;
    output_marker.color.g = 0.29;
    output_marker.color.b = 0.0;
    output_marker.color.a = 1.0;

    // 각 그룹별로 endpoint 쌍 산출 후 연장
    for (const auto &group_pair : tree_line_groups)
    {
      const auto &points = group_pair.second;
      if (points.size() < 2)
        continue;

      // 중심(centroid) 계산
      Eigen::Vector3f centroid(0.f, 0.f, 0.f);
      for (const auto &pt : points)
        centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
      centroid /= static_cast<float>(points.size());

      // 공분산 행렬 계산
      Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
      for (const auto &pt : points)
      {
        Eigen::Vector3f diff(pt.x - centroid.x(), pt.y - centroid.y(), pt.z - centroid.z());
        covariance += diff * diff.transpose();
      }
      covariance /= static_cast<float>(points.size());

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
      if (solver.info() != Eigen::Success)
        continue;
      // 주성분: 가장 큰 고유값에 해당하는 eigenvector (컬럼 2)
      Eigen::Vector3f principal_axis = solver.eigenvectors().col(2);
      // 2D 방향 (정규화)
      Eigen::Vector2f d = Eigen::Vector2f(principal_axis.x(), principal_axis.y()).normalized();

      // 각 포인트를 주성분 방향으로 투영하여 최소/최대 투영값 산출
      float min_proj = std::numeric_limits<float>::max();
      float max_proj = std::numeric_limits<float>::lowest();
      for (const auto &pt : points)
      {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        float proj = (p - centroid).dot(principal_axis);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
      }
      // 원래 endpoint 산출
      Eigen::Vector3f endpoint_min = centroid + principal_axis * min_proj;
      Eigen::Vector3f endpoint_max = centroid + principal_axis * max_proj;

      // 2D 확장: 주성분 방향을 따라 endpoint를 경계까지 확장
      // endpoint_min는 -d 방향, endpoint_max는 d 방향으로 확장
      Eigen::Vector2f orig_min(endpoint_min.x(), endpoint_min.y());
      Eigen::Vector2f orig_max(endpoint_max.x(), endpoint_max.y());
      Eigen::Vector2f extended_min = getExtendedEndpoint(orig_min, -d);
      Eigen::Vector2f extended_max = getExtendedEndpoint(orig_max, d);

      // z 좌표는 그대로 사용 (예: centroid의 z 또는 0)
      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = extended_min.x();
      p_start.y = extended_min.y();
      p_start.z = endpoint_min.z();
      p_end.x = extended_max.x();
      p_end.y = extended_max.y();
      p_end.z = endpoint_max.z();

      // output_marker.points에 두 점을 추가 (LINE_LIST 타입에서 두 점은 하나의 선분)
      output_marker.points.push_back(p_start);
      output_marker.points.push_back(p_end);
    }

    // 최종 Marker 발행
    endpoint_marker_pub_->publish(output_marker);
  }

  // 멤버 변수
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr outline_marker_array_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr endpoint_marker_pub_;

  // 경계값: outline 영역 (예시)
  double x_min_, x_max_, y_min_, y_max_;

  // outline 데이터를 누적할 pcl 포인트 클라우드 (필요 시 활용)
  pcl::PointCloud<pcl::PointXYZ>::Ptr outline_cloud{new pcl::PointCloud<pcl::PointXYZ>};

  // 알고리즘 파라미터 및 자료구조
  double cluster_tolerance;
  int cluster_min_size;
  int cluster_max_size;
  double cos_theta_threshold;
  double dot2line_threshold;
  std::unordered_map<int, int> element_to_set;
  std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudsById;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrchardRowEndpointMarkerProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
