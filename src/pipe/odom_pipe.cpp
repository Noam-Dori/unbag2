//
// Created by Noam Dori on 06/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/pipe/odom_pipe.hpp>
#include <unbag2/pipe/json_utils.hpp>

using Json::Value;
using nav_msgs::msg::Odometry;
using rclcpp::Node;

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

  entry["pose"]["pose"]["position"] = JsonUtils::to_json(msg.pose.pose.position);
  entry["pose"]["pose"]["orientation"] = JsonUtils::to_json(msg.pose.pose.orientation);
  entry["twist"]["twist"]["linear"] = JsonUtils::to_json(msg.twist.twist.linear);
  entry["twist"]["twist"]["angular"] = JsonUtils::to_json(msg.twist.twist.angular);

  JsonUtils::add_covariance(entry["pose"]["covariance"], msg.pose.covariance, covariance_);
  JsonUtils::add_covariance(entry["twist"]["covariance"], msg.twist.covariance, covariance_);
  return entry;
}
}