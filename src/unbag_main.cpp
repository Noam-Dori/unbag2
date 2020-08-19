//
// Created by Noam Dori on 11/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/unbag_node.hpp>

using rclcpp::init;
using rclcpp::shutdown;
using unbag2::UnbagNode;

int main(int argc, char ** argv)
{
  init(argc, argv);
  auto success = UnbagNode::run_on_args(argc, argv);
  shutdown();
  return success;
}