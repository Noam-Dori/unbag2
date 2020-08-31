//
// Created by Noam Dori on 31/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/pipe/gps_pipe.hpp>
#include <regex>

using Json::Value;
using boost::filesystem::path;
using boost::filesystem::ofstream;
using rclcpp::Duration;
using rclcpp::Node;
using rclcpp::Time;
using sensor_msgs::msg::NavSatFix;
using std::regex;
using std::regex_replace;
using std::string;
using std::to_string;

namespace unbag2
{

GpsPipe::GpsPipe() : PipeBase<NavSatFix>("gps_pipe")
{
}

void GpsPipe::load_pipe_params(Node * node)
{
  covariance_ = !node->declare_parameter<bool>(to_param("remove_covariance"), false);
  split_by_time_ = node->declare_parameter<double>(to_param("time"), -1);
  file_name_ = node->declare_parameter<string>(to_param(""), "{topic}_{seq}.json");
}

void GpsPipe::process(NavSatFix msg, const string & topic)
{
  Time time = msg.header.stamp;
  if (last_split_ == Time(0))
  {
    last_split_ = time;
  }
  else if (time >= last_split_ + Duration(split_by_time_))
  {
    last_split_ = time;
    on_bag_end();
  }
  if (out_.find(topic) == out_.end())
  {
    out_[topic] = Value();
  }
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
  out_[topic].append(entry);
}

void GpsPipe::on_bag_end()
{
  for (auto & entry : out_)
  {
    auto topic = regex_replace(entry.first, regex("/"), "_").substr(1);
    auto itr = folders_.find(entry.first);
    string seq;
    if  (itr != folders_.end())
    {
      seq = to_string(itr->second++);
    }
    else
    {
      seq = "0";
      folders_[entry.first] = 1;
    }
    write_file(target_dir_ / topic, entry.second, topic, seq);
  }
  out_.clear();
}

void GpsPipe::on_unbag_end()
{
  for (auto & entry : out_)
  {
    auto topic = regex_replace(entry.first, regex("/"), "_").substr(1);
    path p;
    auto itr = folders_.find(entry.first);
    string seq;
    if  (itr != folders_.end())
    {
      p = target_dir_ / topic;
      seq = to_string(itr->second);
    }
    else
    {
      p = target_dir_;
    }
    write_file(p, entry.second, topic, seq);
  }
}

void GpsPipe::write_file(const path & path, const Value & json_to_write, const string& topic, const string& seq)
{
  string file_name = regex_replace(
      regex_replace(file_name_, regex("\\{topic\\}"), topic), regex("\\{seq\\}"), seq);
  ofstream file(path / file_name);
  file << writer_.write(json_to_write);
  file.close();
}
}