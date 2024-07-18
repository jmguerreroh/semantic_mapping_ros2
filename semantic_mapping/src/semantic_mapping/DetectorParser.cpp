/*
  Copyright (c) 2024 José Miguel Guerrero Hernández

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

#include "semantic_mapping/DetectorParser.hpp"

namespace semantic_mapping
{

using namespace std::placeholders;

DetectorParser::DetectorParser() : Node("semantic_detector_parser") {
  
  subscription_yolo_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/yolo/detections",
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&DetectorParser::topic_callback_yolo, this, _1));

  publisher_semantic_detections_ = this->create_publisher<semantic_mapping_interfaces::msg::SemanticDetectionArray>(
    "/semantic_detections",
    rclcpp::SensorDataQoS().reliable());
}

void DetectorParser::topic_callback_yolo(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg) {
  semantic_mapping_interfaces::msg::SemanticDetectionArray semantic_msg;
  for (auto detection : msg->detections) {
    semantic_mapping_interfaces::msg::SemanticDetection semantic_detection;
    semantic_detection.class_id = detection.class_id;
    semantic_detection.class_name = detection.class_name;
    semantic_detection.score = detection.score;
    semantic_detection.center3d = detection.bbox3d.center;
    semantic_detection.bbox3d = detection.bbox3d.size;
    semantic_msg.detections.push_back(semantic_detection);
  }
  publisher_semantic_detections_->publish(semantic_msg);
}

} // namespace semantic_mapping
