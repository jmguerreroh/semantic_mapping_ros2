/*
  Copyright (c) 2024 José Miguel Guerrero Hernández

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

#include "semantic_mapping/SemanticMapping.hpp"

namespace semantic_mapping
{

using namespace std::placeholders;

SemanticMapping::SemanticMapping() : Node("cv_subscriber") {

  subscription_semantic_detections_ = this->create_subscription<semantic_mapping_interfaces::msg::SemanticDetectionArray>(
    "/semantic_detections",
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&SemanticMapping::topic_callback_semantic_detections, this, _1));

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void SemanticMapping::topic_callback_semantic_detections(const semantic_mapping_interfaces::msg::SemanticDetectionArray::ConstSharedPtr & msg) {
 
}


} // namespace semantic_mapping
