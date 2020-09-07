//
// Created by Noam Dori on 06/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/pipe/odom_pipe.hpp>

using Json::Value;
using rclcpp::Node;
using nav_msgs::msg::Odometry;

namespace unbag2
{

OdomPipe::OdomPipe() : JsonPipe<Odometry>("odom_pipe")
{
}

void OdomPipe::load_json_params(Node * node)
{
  covariance_ = !node->declare_parameter<bool>(to_param("remove_covariance"), false);
}

Value OdomPipe::to_json(Odometry msg)
{
  Value entry;
  entry["child_frame_id"] = msg.child_frame_id;
  entry["header"]["frame_id"] = msg.header.frame_id;
  entry["header"]["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

  entry["pose"]["pose"]["position"] = to_json(msg.pose.pose.position);
  entry["pose"]["pose"]["orientation"] = to_json(msg.pose.pose.orientation);
  entry["twist"]["twist"]["linear"] = to_json(msg.twist.twist.linear);
  entry["twist"]["twist"]["angular"] = to_json(msg.twist.twist.angular);

  add_covariance(entry["pose"]["covariance"], msg.pose.covariance);
  add_covariance(entry["twist"]["covariance"], msg.twist.covariance);
  return entry;
}
}