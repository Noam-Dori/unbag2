//
// Created by Noam Dori on 10/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <rclcpp/time.hpp>

#ifndef UNBAG2_UNBAG_CONTEXT_HPP
#define UNBAG2_UNBAG_CONTEXT_HPP

namespace unbag2
{
class UnbagContext
{
public:
  /**
   * \return the URI of the bag file currently being processed
   */
  std::string bag_uri() const
  {
    return bag_uri_;
  }
  /**
   * \return the total messages to process in the bag (slowly reduced as messages are skipped)
   */
  int messages_in_bag() const
  {
    return msgs_in_bag_;
  }
  /**
   * \return the total amount of bag files in the job
   */
  int bags_in_job() const
  {
    return bags_in_job_;
  }
  /**
   * \return how much time passed since the job started.
   */
  rclcpp::Duration elapsed_time() const
  {
    return clock_.now() - start_;
  }
  /**
   * \return in post: how many messages were processed in this bag file
   *         in live: how many messages were processed.
   */
  int processed_messages() const
  {
    return processed_msgs_;
  }
  /**
   * \return how many bag files were processed in this job
   */
  int processed_bags() const
  {
    return processed_bags_;
  }
  /**
   * \return whether or not unbag is in "post" mode.
   */
  bool is_post_mode() const
  {
    return post_mode_;
  }

  /**
   * \brief tells the context unbag skipped over a message since it was excluded by the user.
   */
  void skip_message()
  {
    msgs_in_bag_--;
  }

  /**
   * \brief tells the context unbag tried to process a message.
   *        It does not indicate whether a plugin successfully did so.
   */
  void done_message()
  {
    processed_msgs_++;
  }

  /**
   * \brief tell the context that unbag is now processing a new bag file.
   * \param num_messages the number of messages unbag is expected to process in this bag file.
   * \param uri the URI of the bag.
   */
  void new_bag(int num_messages, const std::string & uri)
  {
    processed_msgs_ = 0;
    msgs_in_bag_ = num_messages;
    bag_uri_ = uri;
    processed_bags_++;
  }

  /**
   * \brief tells the context that unbag started a new job (in post mode)
   * \param bags_in_job the number of job unbag is expected to process (including empty ones)
   */
  void new_job(int bags_in_job)
  {
    post_mode_ = true;
    bags_in_job_ = bags_in_job;
    start_ = clock_.now();
    processed_bags_ = -1;
  }

  /**
   * \brief tells the context that unbag finished processing all bags.
   */
  void job_done()
  {
    processed_bags_++;
  }

  /**
   * \return this singleton instance.
   */
  static UnbagContext & get_instance()
  {
    return *instance_;
  }
private:
  std::string bag_uri_;
  int msgs_in_bag_, bags_in_job_, processed_msgs_, processed_bags_;
  rclcpp::Time start_;
  bool post_mode_ = false;

  static std::shared_ptr<UnbagContext> instance_;
  static rclcpp::Clock clock_;
};
}

#endif //UNBAG2_UNBAG_CONTEXT_HPP
