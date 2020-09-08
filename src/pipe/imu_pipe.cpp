//
// Created by Noam Dori on 06/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/pipe/imu_pipe.hpp>
#include <unbag2/pipe/json_utils.hpp>

using Json::Value;
using rclcpp::Node;
using sensor_msgs::msg::Imu;

namespace unbag2
{

ImuPipe::ImuPipe() : JsonPipe<Imu>("imu_pipe")
{
}

void ImuPipe::load_json_params(Node * node)
{
  covariance_ = !node->declare_parameter<bool>(to_param("remove_covariance"), false);
}

Value ImuPipe::to_json(Imu msg)
{
  Value entry;
  entry["header"]["frame_id"] = msg.header.frame_id;
  entry["header"]["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

  entry["orientation"] = JsonUtils::to_json(msg.orientation);
  entry["linear_acceleration"] = JsonUtils::to_json(msg.linear_acceleration);
  entry["angular_velocity"] = JsonUtils::to_json(msg.angular_velocity);

  JsonUtils::add_covariance(entry["orientation_covariance"], msg.orientation_covariance, covariance_);
  JsonUtils::add_covariance(entry["linear_acceleration_covariance"], msg.linear_acceleration_covariance, covariance_);
  JsonUtils::add_covariance(entry["angular_velocity_covariance"], msg.angular_velocity_covariance, covariance_);
  return entry;
}
}