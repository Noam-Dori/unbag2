//
// Created by Noam Dori on 08/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/pipe/logger_pipe.hpp>
#include <unbag2/unbag_context.hpp>

#if __linux__
#include <sys/ioctl.h>
#include <unistd.h>
#elif _WIN32
#include <windows.h>
#endif

#define DO_WARN_IF(predicate, ...) if(predicate) RCLCPP_WARN(get_logger(), __VA_ARGS__)
#define DO_INFO_IF(predicate, ...) if(predicate) RCLCPP_INFO(get_logger(), __VA_ARGS__)
#define DO_INFO(...) RCLCPP_INFO(get_logger(), __VA_ARGS__)
#define DO_WARN(...) RCLCPP_WARN(get_logger(), __VA_ARGS__)

using rclcpp::Duration;
using rclcpp::Node;
using std::advance;
using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::string;
using std::stringstream;
using std::to_string;

namespace unbag2
{
UnbagContext & api()
{
  return UnbagContext::get_instance();
}

LoggerPipe::LoggerPipe() : Pipe("logger", -1, true)
{
}

void LoggerPipe::load_pipe_params(Node * node)
{
  job_done_ = node->declare_parameter(to_param("after_job.job_done"), true);
  count_msgs_ = node->declare_parameter(to_param("after_job.num_messages_processed"), true);
  failed_topics_post_ = node->declare_parameter(to_param("after_job.failed_topics"), true);
  failed_topics_live_ = node->declare_parameter(to_param("live_updates.failed_topics"), true);
  job_bar_ = node->declare_parameter(to_param("status_bar.job_progress_bar"), true);
  job_percent_ = node->declare_parameter(to_param("status_bar.job_percent_done"), true);
  bag_bar_ = node->declare_parameter(to_param("status_bar.bag_progress_bar"), true);
  bag_percent_ = node->declare_parameter(to_param("status_bar.bag_percent_done"), true);
  estimated_time_ = node->declare_parameter(to_param("status_bar.estimated_time_left"), true);
  bags_done_ = node->declare_parameter(to_param("status_bar.bags_done"), true);
  start_job_ = node->declare_parameter(to_param("job_start"), true);
  bag_name_ = node->declare_parameter(to_param("live_updates.bag_name"), true);
}

bool LoggerPipe::can_process(const WildMsg &)
{
  return true;
}

bool LoggerPipe::process(const WildMsg & msg)
{
  if(start_)
  {
    start_ = false;
    DO_INFO_IF(start_job_, "Starting job...");
  }
  if(next_bag_)
  {
    next_bag_ = false;
    if (api().is_post_mode())
    {
      DO_INFO_IF(bag_name_, "Processing %s", api().bag_uri().c_str());
    }
    else
    {
      DO_INFO_IF(bag_name_, "Processing incoming messages...");
    }
  }
  if (!msg.processed() && fail_.find(msg.topic()) != fail_.end())
  {
    DO_WARN_IF(failed_topics_live_, sizeof(TOPIC_FAIL) + msg.topic().size() < get_terminal_width() ? TOPIC_FAIL : TOPIC_FAIL_SHORT,
               msg.topic().c_str());
    fail_.insert(msg.topic());
  }
  else
  {
    msgs_.insert(msg.topic());
  }
  if (api().is_post_mode())
  {
    do_progress_bar();
  }
  return false; // this means the logger does not change the success status of the message.
}

void LoggerPipe::on_bag_end()
{
  next_bag_ = true;
}

void LoggerPipe::on_job_end()
{
  cout << endl;
  DO_INFO_IF(job_done_, "Unbag completed.");
  if (count_msgs_)
  {
    for (auto it = msgs_.begin(); it != msgs_.end(); advance(it,msgs_.count(*it)))
    {
      DO_INFO("Messages from %s: %d", it->c_str(), msgs_.count(*it));
    }
  }
  if (failed_topics_post_)
  {
    for (auto it = fail_.begin(); it != fail_.end(); advance(it,fail_.count(*it)))
    {
      DO_WARN(sizeof(TOPIC_FAIL) + it->size() < get_terminal_width() ? TOPIC_FAIL : TOPIC_FAIL_SHORT, it->c_str());
    }
  }
}

void LoggerPipe::do_progress_bar()
{
  // get strings for whatever we need to print, save for the bars
  list<pair<bool, string>> subjects;
  double bag_progress = double(api().processed_messages()) / api().messages_in_bag();
  double job_progress = double(api().processed_bags()) / api().bags_in_job();
  subjects.emplace_back(bag_percent_, (bag_bar_ ? "" : "bag: ") + to_string(100. * bag_progress) + "%");
  subjects.emplace_back(job_percent_, (job_bar_ ? "" : "job: ") + to_string(100. * job_progress) + "%");
  subjects.emplace_back(estimated_time_, "est: " + get_estimated_time());
  subjects.emplace_back(bags_done_, to_string(api().processed_bags()) + "/" + to_string(api().bags_in_job()));
  // calculate the bar size
  size_t column = get_terminal_width() + 1;
  for (const auto & subject : subjects)
  {
    if (subject.first)
    {
      column -= subject.second.size() + 1;
    }
  }
  size_t bar_size = bag_bar_ && job_bar_ ? (column - 14) / 2 : column - 3;
  // add bar strings
  subjects.emplace(subjects.begin(), job_bar_, "job: " + get_bar(job_progress, bar_size));
  subjects.emplace_front(bag_bar_, "bag: " + get_bar(bag_progress, bar_size));
  // print
  column = 0;
  for (const auto & subject : subjects)
  {
    if (subject.first)
    {
      cout << (column == 0 ? "" : " ") << subject.second;
      column++;
    }
  }
  (cout << "\r").flush();
}

string LoggerPipe::get_bar(double progress, size_t bar_size)
{
  stringstream ret("[");
  size_t pos = bar_size * progress;
  for (size_t i = 0; i < bar_size; i++)
  {
    if (i < pos)
    {
      ret << "#";
    }
    else
    {
      ret << "-";
    }
  }
  ret << "]";
  return ret.str();
}

string LoggerPipe::get_estimated_time()
{
  Duration time_per_msg = api().elapsed_time() * (1/double(msgs_.size() + fail_.size()));
  Duration time_per_bag = (api().elapsed_time() - (time_per_msg * api().messages_in_bag())) *
      (1./api().processed_bags());
  Duration estimated_time = time_per_bag * (api().bags_in_job() - api().processed_bags() - 1) +
      time_per_msg * (api().messages_in_bag() - api().processed_messages());
  size_t seconds = estimated_time.seconds();
  string ret;
  if (seconds > 3600)
  {
    ret += to_string(seconds / 3600) + ":";
  }
  if (seconds > 60)
  {
    ret += to_string(seconds % 60 / 60) + ":";
  }
  else
  {
    ret += "0:";
  }
  ret += seconds % 60;
  return ret;
}

size_t LoggerPipe::get_terminal_width()
{
#if __linux__
  winsize w{};
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

  return w.ws_col;
#elif _WIN32
  CONSOLE_SCREEN_BUFFER_INFO csbi;

  GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
  return csbi.srWindow.Right - csbi.srWindow.Left + 1;
#endif
}
}