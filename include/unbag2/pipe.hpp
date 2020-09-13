//
// Created by Noam Dori on 12/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "wild_msg.hpp"
#include <rclcpp/rclcpp.hpp>
#include <boost/filesystem.hpp>
#include <regex>

#ifndef UNBAG2_PIPE_HPP
#define UNBAG2_PIPE_HPP

namespace unbag2
{
/**
 * \brief the interface class used to process ROS messages into static data.
 */
class Pipe
{
public:
  /**
   * \brief constructs a new pipe
   * \param name the pipe's name, used in the yaml file.
   * \param priority determines the order in which plugins will be used.
   *                 Higher numbers mean the pipe will come before pipes with lower priority.
   * \param force_bag_split whether or not this pipe MUST know about bag splits.
   * \note when implementing, you must set the name IN THE CONSTRUCTOR.
   */
  explicit Pipe(std::string name, double priority = 0, bool force_bag_split = false) : prefix_(move(name))
  {
    priority_ = priority;
    force_split_ = force_bag_split;
  }

  /**
   * \return true if the pipe is enabled meaning it should be running, false if the user toggled it off.
   */
  bool enabled() const
  {
    return enabled_;
  }

  /**
   * \return true if the pipe MUST know about bag splits no matter what, false otherwise.
   */
  bool force_split() const
  {
    return force_split_;
  }

  /**
   * \brief whether or not msg can be processed by this pipe
   * \param msg the serialized message to check
   * \return true if msg can be processed, false otherwise.
   */
  virtual bool can_process(const WildMsg & msg) = 0;

  /**
   * \brief a comparison operator used to sort sets of this stuff
   * \param other the pipe to compare this one to
   * \return true if this pipe should be placed before the other, false otherwise.
   */
  bool operator<(const Pipe & other) const
  {
    return other.priority_ < priority_;
  }

  /**
   * \brief uses the node to load pipe specific parameters.
   * \param node the reference node to pick up parameters.
   */
  void load_params(rclcpp::Node * node)
  {
    enabled_ = node->declare_parameter(to_param("enabled"), true);
    logger_ = std::make_shared<rclcpp::Logger>(node->get_logger());
    boost::filesystem::path target_dir = node->get_parameter("target_dir").as_string();
    if(!target_dir.is_absolute())
    {
      target_dir = boost::filesystem::current_path() / target_dir;
    }

    // next we need to take care of ../ and ./ in the path
    std::regex folder_regex("/\\.(?=/|$)"), parent_regex("/[^/]+/\\.\\.(?=/|$)");
    target_dir = regex_replace(target_dir.string(), folder_regex, "");
    target_dir = regex_replace(target_dir.string(), parent_regex, "");

    if (!exists(target_dir))
    {
      target_dir_ = target_dir;
      create_directories(target_dir_);
    }
    else
    {
      target_dir_ = is_directory(target_dir) ? target_dir : target_dir.parent_path();
    }

    load_pipe_params(node);
  }

  /**
   * \brief process the wild message
   * \param msg a message that satisfies can_process(msg) == true
   */
  virtual bool process(const WildMsg & msg) = 0;

  /**
   * \brief a signal given to the pipe when a bag finished processing,
   *        and the user indicated they want to split by bag-file. Only happens in "post" mode.
   */
  virtual void on_bag_end()
  {
  }

  /**
   * \brief A signal given to the pipe when unbag finished processing all bag files. Only happens in "post" mode.
   */
  virtual void on_job_end()
  {
  }

  /**
   * \brief A utility that gets the message type of the class.
   * \tparam Msg the class to check
   * \return a human readable string of the message type.
   */
  template <class Msg>
  static std::string get_msg_type()
  {
    std::string raw_type(typeid(Msg).name());
    int index = 1;
    for (; raw_type[index + 1] <= '9' && raw_type[index + 1] >= '0'; index++);
    auto len = stoi(raw_type.substr(1, index));
    std::string ns = raw_type.substr(index + 1, len);
    len = index += 4 + len; // len is now type_start
    for (; raw_type[index + 1] <= '9' && raw_type[index + 1] >= '0'; index++);
    std::string type = raw_type.substr(index + 1, stoi(raw_type.substr(len + 1, index - len)) - 1);
    return ns + "/" + type;
  }
protected:

  /**
   * \return gets the target logger for the pipe. This is just the standard unbag logger.
   */
  rclcpp::Logger & get_logger() const
  {
    return *logger_;
  }

  /**
   * \return the name assigned to this pipe.
   */
  std::string get_name() const
  {
    return prefix_;
  }

  /**
   * \brief allows parameter fetching from the yaml tab assigned to the pipe.
   * \param param the unqualified parameter name
   * \return the fully qualified parameter name to query in the config.
   */
  std::string to_param(const std::string& param) const
  {
    return prefix_ + "." + param;
  }

  /**
   * \brief load parameters this pipe uses for configuration
   * \param node (rclcpp::Node *) the node parameters are read from
   */
  virtual void load_pipe_params(rclcpp::Node *)
  {
  }

  boost::filesystem::path target_dir_;
private:
  bool enabled_ = true, force_split_ = false;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::string prefix_;
  double priority_;
};
}

#endif //UNBAG2_PIPE_HPP
