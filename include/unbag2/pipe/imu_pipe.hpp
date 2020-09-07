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

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
  template <class T>
  void try_w(Json::Value & out, const T & potential_w, decltype(&T::w))
  {
    out["w"] = potential_w.w;
  }
#pragma clang diagnostic pop
  template<class T>
  void try_w(Json::Value &, const T &, ...)
  {
  }

  template<class T>
  Json::Value to_json(T point_or_quaternion)
  {
    Json::Value ret;
    ret["x"] = point_or_quaternion.x;
    ret["y"] = point_or_quaternion.y;
    ret["z"] = point_or_quaternion.z;
    try_w(ret, point_or_quaternion, nullptr);
    return ret;
  }

  template<size_t size>
  void add_covariance(Json::Value & out, std::array<double, size> covariance)
  {
    if (covariance_ || std::any_of(covariance.begin(), covariance.end(), [](double d){return d != 0;}))
    {
      for (auto item : covariance)
      {
        out.append(item);
      }
    }
  }
};
}

PLUGINLIB_EXPORT_CLASS(unbag2::ImuPipe, unbag2::Pipe) // NOLINT(cert-err58-cpp)

#endif //UNBAG2_GPS_PIPE_HPP
