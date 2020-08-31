//
// Created by Noam Dori on 12/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "wild_msg.hpp"
#include <rclcpp/rclcpp.hpp>
#include <boost/filesystem.hpp>

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
   * \param name the pipe's name.
   * \note when implementing, you must set the name IN THE CONSTRUCTOR.
   */
  explicit Pipe(std::string name);
  /**
   * \brief uses the node to load pipe specific parameters.
   * \param node the reference node to pick up parameters.
   */
  void load_params(rclcpp::Node * node);
  /**
   * \return true if the pipe is enabled meaning it should be running, false if the user toggled it off.
   */
  bool enabled() const;

  /**
   * \brief whether or not msg can be processed by this pipe
   * \param msg the serialized message to check
   * \return true if msg can be processed, false otherwise.
   */
  virtual bool can_process(const WildMsg & msg) = 0;

  /**
   * \brief process the wild message
   * \param msg a message that satisfies can_process(msg) == true
   */
  virtual void process(const WildMsg & msg) = 0;

  /**
   * \brief a signal given to the pipe when a bag finished processing,
   *        and the user indicated they want to split by bag-file. Only happens in "post" mode.
   */
  virtual void on_bag_end();

  /**
   * \brief A signal given to the pipe when unbag finished processing all bag files. Only happens in "post" mode.
   */
  virtual void on_unbag_end();

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
  virtual void load_pipe_params(rclcpp::Node * node);

  std::string to_param(const std::string& name);

  boost::filesystem::path target_dir_;
private:
  bool enabled_ = true;
  std::string prefix_;
};

/**
 * \brief a useful interface for a pipe that processes only one type of messages.
 * \tparam RosMsg the class of message this pipe processes
 */
template <class RosMsg>
class PipeBase : public Pipe // pipe is not an interface so no virtual
{
public:
  /**
   * \brief constructs a new pipe
   * \param name the pipe's name.
   * \note when implementing, you must set the name IN THE CONSTRUCTOR.
   */
  explicit PipeBase(const std::string& name) : Pipe(name)
  {
  }

  bool can_process(const WildMsg & msg) override
  {
    return msg.type() == get_msg_type<RosMsg>();
  }

  void process(const WildMsg & msg) override
  {
    process(msg.deserialize<RosMsg>(), msg.topic());
  }
protected:
  /**
   * \brief process a ROS message
   * \param msg (RosMsg) the message to process
   * \param topic (const std::string &) the topic from which the message came from
   */
  virtual void process(RosMsg, const std::string &)
  {
  }
};
}

#endif //UNBAG2_PIPE_HPP
