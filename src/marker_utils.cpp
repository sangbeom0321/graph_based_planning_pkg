/**
 * @brief 마커 하나를 생성하여 MarkerArray에 추가합니다.
 *
 * RViz 시각화를 위한 유틸 함수로, 지정된 위치에 원하는 모양과 색상의 Marker를 생성하여
 * 전달된 MarkerArray에 삽입합니다. 주로 Sphere, Cube, Points 등 시각화 목적에 따라 다양하게 활용됩니다.
 * 
 * @author  우상범
 * @date    2025-04-08
 * @version 1.0
 * 
 * @param x         마커의 x 좌표
 * @param y         마커의 y 좌표
 * @param z         마커의 z 좌표
 * @param id        마커의 고유 ID
 * @param header    std_msgs::msg::Header (frame_id 및 timestamp 포함)
 * @param marker_array 마커를 삽입할 visualization_msgs::msg::MarkerArray
 * @param ns        마커 네임스페이스 (RViz에서 그룹핑에 사용)
 * @param marker_type 마커 타입 (e.g. Marker::SPHERE, Marker::CUBE, Marker::POINTS 등)
 * @param scale_x   마커의 X축 크기
 * @param scale_y   마커의 Y축 크기
 * @param scale_z   마커의 Z축 크기
 * @param r         마커 색상의 R값 (0.0 ~ 1.0)
 * @param g         마커 색상의 G값 (0.0 ~ 1.0)
 * @param b         마커 색상의 B값 (0.0 ~ 1.0)
 * 
 * @note 마커의 alpha 값은 1.0, lifetime은 10초로 고정되어 있습니다.
 */

#include "graph_based_planning_pkg/marker_utils.hpp"

void visualize_point_markers(
  float x, float y, float z,
  int id,
  const std_msgs::msg::Header& header,
  visualization_msgs::msg::MarkerArray& marker_array,
  const std::string& ns,
  int marker_type,
  float scale_x, float scale_y, float scale_z,
  float r, float g, float b)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = marker_type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale.x = scale_x;
  marker.scale.y = scale_y;
  marker.scale.z = scale_z;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
  marker.lifetime.sec = 10;
  marker_array.markers.push_back(marker);
}
