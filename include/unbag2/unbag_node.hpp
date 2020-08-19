//
// Created by Noam Dori on 11/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/generic_subscription.hpp>
#include <unbag2/pipe.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/converter.hpp>

#ifndef UNBAG2_UNBAG_NODE_H
#define UNBAG2_UNBAG_NODE_H

namespace unbag2
{
class UnbagNode : public rclcpp::Node
{
public:
  /**
   * \brief creates the unbag node and runs it according to the parameters set.
   * \param argc argument count
   * \param argv argument values
   * \return a success code for how well unbag ran.
   */
  static int run_on_args(int argc, char ** argv);
  /**
   * \brief constructs an unbag node.
   */
  UnbagNode();
private:
  void init_plugins();
  void init_subscribers();
  void add_source(const std::string& input_path);
  void run_on_files();
  void unbag_callback(const std::shared_ptr<rclcpp::SerializedMessage>& data,
                      const std::string& topic,
                      const rosidl_message_type_support_t * ts);
  std::list<std::shared_ptr<Pipe>> pipes_;
  std::list<std::string> bag_uris_;
  std::list<GenericSubscription> subscriptions_;
};
}

#endif //UNBAG2_UNBAG_NODE_H
