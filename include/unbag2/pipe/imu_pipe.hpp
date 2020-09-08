//
// Created by Noam Dori on 07/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "json_pipe.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/imu.hpp>

#ifndef UNBAG2_IMU_PIPE_HPP
#define UNBAG2_IMU_PIPE_HPP

namespace unbag2
{

/**
 * \brief a pipe specializing in converting Imu messages to JSON files.
 */
class ImuPipe : public JsonPipe<sensor_msgs::msg::Imu>
{
public:
  /**
   * \brief construct a new Imu pipe. figures.
   */
  ImuPipe();

protected:
  void load_json_params(rclcpp::Node * node) override;

  Json::Value to_json(sensor_msgs::msg::Imu msg) override;
private:
  bool covariance_ = false;
};
}

PLUGINLIB_EXPORT_CLASS(unbag2::ImuPipe, unbag2::Pipe) // NOLINT(cert-err58-cpp)

#endif //UNBAG2_GPS_PIPE_HPP
