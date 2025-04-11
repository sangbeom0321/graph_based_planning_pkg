#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <utility>

class VoronoiPathNode : public rclcpp::Node {
public:
  VoronoiPathNode() : Node("voronoi_path_node") {
    sub_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/voronoi_points_array", 10,
      std::bind(&VoronoiPathNode::pose_array_callback, this, std::placeholders::_1));

    rclcpp::QoS qos_profile_odom(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_profile_odom.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile_odom.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/lio_sam/mapping/odometry", qos_profile_odom,
      std::bind(&VoronoiPathNode::odom_callback, this, std::placeholders::_1));

    sub_line_segments_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "/line_segments", 10,
    std::bind(&VoronoiPathNode::line_segments_callback, this, std::placeholders::_1));

    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/generated_path", 10);
    pub_filtered_points_ = this->create_publisher<visualization_msgs::msg::Marker>("/filtered_voronoi_points", 10);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_poses_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_filtered_points_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_line_segments_;

  std::vector<geometry_msgs::msg::Point> past_positions_;
  std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> line_segments_;

  geometry_msgs::msg::Point current_position_;
  bool has_odom_ = false;

  void line_segments_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    line_segments_.clear();
    for (size_t i = 0; i < msg->points.size(); i += 2) {
      line_segments_.emplace_back(msg->points[i], msg->points[i + 1]);
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    has_odom_ = true;
    if (past_positions_.empty() || std::hypot(current_position_.x - past_positions_.back().x,
                                              current_position_.y - past_positions_.back().y) > 0.2) {
      past_positions_.push_back(current_position_);
    }
  }

  bool segmentsIntersect(const geometry_msgs::msg::Point& a1, const geometry_msgs::msg::Point& a2,
                         const geometry_msgs::msg::Point& b1, const geometry_msgs::msg::Point& b2) {
    auto ccw = [](const geometry_msgs::msg::Point& A, const geometry_msgs::msg::Point& B, const geometry_msgs::msg::Point& C) {
      return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
    };
    return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) && (ccw(a1, a2, b1) != ccw(a1, a2, b2));
  }

  void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty() || !has_odom_) return;

    std::vector<geometry_msgs::msg::Pose> filtered_poses;
    for (const auto& pose : msg->poses) {
      bool too_close = false;
      for (const auto& past : past_positions_) {
        float dx = pose.position.x - past.x;
        float dy = pose.position.y - past.y;
        if (dx * dx + dy * dy < 16.0f) {
          too_close = true;
          break;
        }
      }
      if (!too_close) filtered_poses.push_back(pose);
    }

    if (filtered_poses.empty()) return;

    std::vector<bool> visited(filtered_poses.size(), false);
    std::vector<int> visit_order;

    int start_idx = -1;
    float min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < filtered_poses.size(); ++i) {
      float dx = filtered_poses[i].position.x - current_position_.x;
      float dy = filtered_poses[i].position.y - current_position_.y;
      float dist = dx * dx + dy * dy;

      bool intersects = false;
      for (const auto& seg : line_segments_) {
        if (segmentsIntersect(current_position_, filtered_poses[i].position, seg.first, seg.second)) {
          intersects = true;
          break;
        }
      }

      if (!intersects && dist < min_dist) {
        min_dist = dist;
        start_idx = i;
      }
    }

    // fallback: intersect를 허용하고라도 가장 가까운 점을 찾아야 할 경우
    if (start_idx == -1) {
      float fallback_min = std::numeric_limits<float>::max();
      for (size_t i = 0; i < filtered_poses.size(); ++i) {
        float dx = filtered_poses[i].position.x - current_position_.x;
        float dy = filtered_poses[i].position.y - current_position_.y;
        float dist = dx * dx + dy * dy;
        if (dist < fallback_min) {
          fallback_min = dist;
          start_idx = i;
        }
      }
    }
    visit_order.push_back(start_idx);
    visited[start_idx] = true;

    while (visit_order.size() < filtered_poses.size()) {
      int current_idx = visit_order.back();
      const auto& current_pose = filtered_poses[current_idx];
      int next_idx = -1, fallback_idx = -1;
      float best_dist = std::numeric_limits<float>::max();
      float fallback_dist = std::numeric_limits<float>::max();

      for (size_t i = 0; i < filtered_poses.size(); ++i) {
        if (visited[i]) continue;
        float dx = current_pose.position.x - filtered_poses[i].position.x;
        float dy = current_pose.position.y - filtered_poses[i].position.y;
        float dist = dx * dx + dy * dy;
        
        bool intersects = false;
        for (const auto& seg : line_segments_) {
          if (segmentsIntersect(current_pose.position, filtered_poses[i].position, seg.first, seg.second)) {
            intersects = true;
            break;
          }
        }
        if (!intersects && dist < best_dist) {
          best_dist = dist;
          next_idx = i;
        } else if (intersects && dist < fallback_dist) {
          fallback_dist = dist;
          fallback_idx = i;
        }
      }

      if (next_idx != -1) {
        visited[next_idx] = true;
        visit_order.push_back(next_idx);
      } else if (fallback_idx != -1) {
        visited[fallback_idx] = true;
        visit_order.push_back(fallback_idx);
      } else {
        break;
      }
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = msg->header.frame_id;
    path_msg.header.stamp = this->get_clock()->now();

    //  Step 1: 로봇 현재 위치를 시작점으로 추가
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header = path_msg.header;
    robot_pose.pose.position = current_position_;
    robot_pose.pose.orientation.w = 1.0;  // 단순 초기값
    path_msg.poses.push_back(robot_pose);


    for (int i = 0; i < visit_order.size()-1; ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose = filtered_poses[visit_order[i]];

      path_msg.poses.push_back(ps);
    }
    pub_path_->publish(path_msg);

    visualization_msgs::msg::Marker marker;
    marker.header = path_msg.header;
    marker.ns = "filtered_voronoi_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    for (const auto& pose : filtered_poses) {
      marker.points.push_back(pose.position);
    }
    pub_filtered_points_->publish(marker);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiPathNode>());
  rclcpp::shutdown();
  return 0;
}