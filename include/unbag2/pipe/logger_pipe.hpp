//
// Created by Noam Dori on 08/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "../pipe.hpp"
#include <pluginlib/class_list_macros.hpp>

#ifndef UNBAG2_LOGGER_PIPE_HPP
#define UNBAG2_LOGGER_PIPE_HPP

namespace unbag2
{
/**
 * \brief a unique class used to log progress
 */
class LoggerPipe : public Pipe
{
public:
  LoggerPipe();

  bool can_process(const WildMsg &) override;

  bool process(const WildMsg & msg) override;

  void on_bag_end() override;

  void on_job_end() override;

protected:
  void load_pipe_params(rclcpp::Node * node) override;

private:
  static size_t get_terminal_width();

  static constexpr auto TOPIC_FAIL = "Topic %s was incompatible for all plugins and was not processed.";
  static constexpr auto TOPIC_FAIL_SHORT = "Topic %s failed processing.";
  bool next_bag_ = true, start_ = true;
  std::multiset<std::string, int> msgs_, fail_;
  bool failed_topics_post_{}, job_done_{}, count_msgs_{}, start_job_{}, bag_name_{}, failed_topics_live_{},
  job_bar_{}, job_percent_{}, bag_bar_{}, bag_percent_{}, estimated_time_{}, bags_done_{};
  void do_progress_bar();
  std::string get_estimated_time();
  static std::string get_bar(double progress, size_t bar_size);
};
}

PLUGINLIB_EXPORT_CLASS(unbag2::LoggerPipe, unbag2::Pipe) // NOLINT(cert-err58-cpp)

#endif //UNBAG2_LOGGER_PIPE_HPP
