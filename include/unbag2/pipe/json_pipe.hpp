//
// Created by Noam Dori on 06/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "../pipe.hpp"
#include <jsoncpp/json/json.h>
#include <regex>

#ifndef UNBAG2_JSON_PIPE_HPP
#define UNBAG2_JSON_PIPE_HPP

namespace unbag2
{
/**
 * \brief a useful interface for a pipe that processes only one type of messages into JSON files.
 * \tparam RosMsg the class of message this pipe processes
 */
template<class RosMsg>
class JsonPipe : public Pipe // pipe is not an interface so no virtual
{
public:
  /**
   * \brief constructs a new pipe
   * \param name the pipe's name.
   * \note when implementing, you must set the name IN THE CONSTRUCTOR.
   */
  explicit JsonPipe(const std::string & name) : Pipe(name)
  {
  }

  bool can_process(const WildMsg & msg) override
  {
    return msg.type() == get_msg_type<RosMsg>();
  }

  void process(const WildMsg & wild_msg) override
  {
    RosMsg msg = wild_msg.deserialize<RosMsg>();
    try_split<RosMsg>(msg, nullptr);
    if (out_.find(wild_msg.topic()) == out_.end())
    {
      out_[wild_msg.topic()] = Json::Value();
    }
    out_[wild_msg.topic()].append(to_json(msg));
  }

  void on_bag_end() override
  {
    for (auto & entry : out_)
    {
      auto topic = regex_replace(entry.first, std::regex("/"), "_").substr(1);
      auto itr = folders_.find(entry.first);
      std::string seq;
      if  (itr != folders_.end())
      {
        seq = std::to_string(itr->second++);
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

  void on_unbag_end() override
  {
    for (auto & entry : out_)
    {
      auto topic = regex_replace(entry.first, std::regex("/"), "_").substr(1);
      boost::filesystem::path p;
      auto itr = folders_.find(entry.first);
      std::string seq;
      if  (itr != folders_.end())
      {
        p = target_dir_ / topic;
        seq = std::to_string(itr->second);
      }
      else
      {
        p = target_dir_;
      }
      write_file(p, entry.second, topic, seq);
    }
  }
protected:
  /**
   * \brief convert a ROS message to JSON
   * \param msg (RosMsg) the message to process
   */
  virtual Json::Value to_json(RosMsg)
  {
    return Json::Value();
  }

  void load_pipe_params(rclcpp::Node * node) override
  {
    split_by_time_ = node->declare_parameter<double>(to_param("time"), -1);
    file_name_ = node->declare_parameter<std::string>(to_param("file_name"), "{topic}_{seq}.json");
    load_json_params(node);
  }

  /**
   * \brief load parameters this pipe uses for configuring the json output
   * \param node (rclcpp::Node *) the node parameters are read from
   */
  virtual void load_json_params(rclcpp::Node *)
  {
  }

private:
  double split_by_time_ = -1;
  std::map<std::string, Json::Value> out_;
  std::map<std::string, int> folders_;
  Json::FastWriter writer_;
  std::string file_name_;
  rclcpp::Time last_split_{0};

  void write_file(const boost::filesystem::path & path, const Json::Value & json_to_write, const std::string & topic,
                  const std::string& seq)
  {
    std::string file_name = regex_replace(
        regex_replace(file_name_, std::regex("\\{topic\\}"), topic), std::regex("\\{seq\\}"), seq);
    boost::filesystem::ofstream file(path / file_name);
    file << writer_.write(json_to_write);
    file.close();
  }

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
  template <class T> void try_split(RosMsg msg, decltype(T::header))
  {
    rclcpp::Time time = msg.header.stamp;
    if (last_split_ == rclcpp::Time(0))
    {
      last_split_ = time;
    }
    else if (split_by_time_ > 0 && time >= last_split_ + rclcpp::Duration(split_by_time_))
    {
      last_split_ += rclcpp::Duration(split_by_time_);
      on_bag_end();
    }
  }
#pragma clang diagnostic pop
  template <class> void try_split(RosMsg, ...)
  {
    if (split_by_time_ > 0)
    {
      RCLCPP_WARN(get_logger(), "parameter [%s.time] ignored: message type %s has no header.",
                  get_name().c_str(), get_msg_type<RosMsg>().c_str());
    }
  }
};
}
#endif //DEV_WS_JSON_PIPE_HPP
