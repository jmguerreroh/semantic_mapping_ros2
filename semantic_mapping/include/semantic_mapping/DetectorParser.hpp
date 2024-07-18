/*
  Copyright (c) 2024 José Miguel Guerrero Hernández

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

#ifndef INCLUDE_SEMANTIC_MAPPING__PARSER_HPP_
#define INCLUDE_SEMANTIC_MAPPING__PARSER_HPP_

#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "depth_image_proc/depth_traits.hpp"
#include <pcl/filters/statistical_outlier_removal.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_ros/transforms.hpp"
#include "semantic_mapping_interfaces/msg/semantic_detection_array.hpp"
#include "semantic_mapping_interfaces/msg/semantic_detection.hpp"



namespace semantic_mapping
{

class DetectorParser : public rclcpp::Node
{
public:
  DetectorParser();

private:
  void topic_callback_yolo(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg);

  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr subscription_yolo_;

  rclcpp::Publisher<semantic_mapping_interfaces::msg::SemanticDetectionArray>::SharedPtr publisher_semantic_detections_; 
};

} // namespace semantic_mapping

#endif  // INCLUDE_SEMANTIC_MAPPING__PARSER_HPP_
