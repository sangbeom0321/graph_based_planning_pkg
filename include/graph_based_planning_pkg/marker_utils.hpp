/// @brief 지정 위치에 시각화 마커를 추가합니다.
/// @param x, y, z 위치 좌표
/// @param id 마커 ID
/// @param header 메시지 헤더 (frame_id, stamp 포함)
/// @param marker_array 추가할 MarkerArray
/// @param ns RViz에서의 네임스페이스
/// @param marker_type Marker 종류 (SPHERE, CUBE 등)
/// @param scale_x, scale_y, scale_z 크기
/// @param r, g, b 색상 (0~1)

#pragma once

#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

void visualize_point_markers(
  float x, float y, float z,
  int id,
  const std_msgs::msg::Header& header,
  visualization_msgs::msg::MarkerArray& marker_array,
  const std::string& ns="default",
  int marker_type=visualization_msgs::msg::Marker::SPHERE,
  float scale_x=0.5, float scale_y=0.5, float scale_z=0.5,
  float r=0.2f, float g=0.8f, float b=0.2f);