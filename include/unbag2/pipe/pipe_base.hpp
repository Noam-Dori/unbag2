//
// Created by Noam Dori on 08/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "../pipe.hpp"

#ifndef UNBAG2_PIPE_BASE_HPP
#define UNBAG2_PIPE_BASE_HPP

namespace unbag2
{
/**
 * \brief a useful interface for a pipe that processes only one type of messages.
 * \tparam RosMsg the class of message this pipe processes
 * \see JsonPipe for JSON file oriented message pipe.
 */
template<class RosMsg>
class PipeBase : public Pipe // pipe is not an interface so no virtual
{
public:
  /**
   * \brief constructs a new pipe
   * \param name the pipe's name.
   * \note when implementing, you must set the name IN THE CONSTRUCTOR.
   */
  explicit PipeBase(const std::string & name) : Pipe(name)
  {
  }

  bool can_process(const WildMsg & msg) override
  {
    return msg.type() == get_msg_type<RosMsg>();
  }

  bool process(const WildMsg & msg) override
  {
    return process(msg.deserialize<RosMsg>(), msg.topic());
  }
protected:
  /**
   * \brief process a ROS message
   * \param msg (RosMsg) the message to process
   * \param topic (const std::string &) the topic from which the message came from
   */
  virtual bool process(RosMsg, const std::string &)
  {
  }
};
}

#endif //UNBAG2_PIPE_BASE_HPP
