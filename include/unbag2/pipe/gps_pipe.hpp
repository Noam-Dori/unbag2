//
// Created by Noam Dori on 31/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "../pipe.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <jsoncpp/json/json.h>

#ifndef UNBAG2_GPS_PIPE_HPP
#define UNBAG2_GPS_PIPE_HPP

namespace unbag2
{
/**
 * \brief
 */
class GpsPipe : public PipeBase<sensor_msgs::msg::NavSatFix>
{
public:
  GpsPipe();

  void load_pipe_params(rclcpp::Node * node) override;

  void process(sensor_msgs::msg::NavSatFix msg, const std::string & topic) override;

  void on_bag_end() override;

  void on_unbag_end() override;
private:
  bool covariance_ = false;
  double split_by_time_ = -1;
  std::map<std::string, Json::Value> out_;
  std::map<std::string, int> folders_;
  Json::FastWriter writer_;
  std::string file_name_;
  rclcpp::Time last_split_{0};
  void write_file(const boost::filesystem::path & path,
                  const Json::Value & json_to_write,
                  const std::string& topic,
                  const std::string& seq);
};
}

PLUGINLIB_EXPORT_CLASS(unbag2::GpsPipe, unbag2::Pipe) // NOLINT(cert-err58-cpp)

#endif //UNBAG2_GPS_PIPE_HPP
