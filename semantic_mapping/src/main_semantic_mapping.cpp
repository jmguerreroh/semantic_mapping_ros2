/*
  Copyright (c) 2024 José Miguel Guerrero Hernández

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

#include "semantic_mapping/SemanticMapping.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto cv_node = std::make_shared<semantic_mapping::SemanticMapping>();
  rclcpp::spin(cv_node);

  rclcpp::shutdown();
  return 0;
}
