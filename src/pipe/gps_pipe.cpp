//
// Created by Noam Dori on 31/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/pipe/gps_pipe.hpp>

using Json::Value;
using rclcpp::Node;
using sensor_msgs::msg::NavSatFix;
using std::string;

namespace unbag2
{

GpsPipe::GpsPipe() : JsonPipe<NavSatFix>("gps_pipe")
{
}

void GpsPipe::load_json_params(Node * node)
{
  covariance_ = !node->declare_parameter<bool>(to_param("remove_covariance"), false);
}

Value GpsPipe::to_json(NavSatFix msg)
{
  Value entry;
  entry["latitude"] = msg.latitude;
  entry["longitude"] = msg.longitude;
  entry["altitude"] = msg.altitude;
  entry["header"]["frame_id"] = msg.header.frame_id;
  entry["header"]["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  if (covariance_ || msg.position_covariance_type != NavSatFix::COVARIANCE_TYPE_UNKNOWN)
  {
    auto & covariance = entry["position_covariance"];
    for (auto item : msg.position_covariance)
    {
      covariance.append(item);
    }
  }
  return entry;
}
}