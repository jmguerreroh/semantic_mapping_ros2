/*
  Copyright (c) 2024 José Miguel Guerrero Hernández

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

#ifndef INCLUDE_SEMANTIC_MAPPING__SEMANTICMAPPING_HPP_
#define INCLUDE_SEMANTIC_MAPPING__SEMANTICMAPPING_HPP_

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


namespace semantic_mapping
{

struct SemanticDetection
{
  int class_id;
  std::string class_name;
  double score;
  cv::Point3d location;
  cv::Point3d size;
};


class SemanticMapping : public rclcpp::Node
{
public:
  SemanticMapping();

private:
  void topic_callback_semantic_detections(const semantic_mapping_interfaces::msg::SemanticDetectionArray::ConstSharedPtr & msg);

  rclcpp::Subscription<semantic_mapping_interfaces::msg::SemanticDetectionArray>::SharedPtr subscription_semantic_detections_;
  
  std::map<cv::Point3d, SemanticDetection> semantic_detections_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string frame_id_;
 
};

} // namespace semantic_mapping

#endif  // INCLUDE_SEMANTIC_MAPPING__SEMANTICMAPPING_HPP_
