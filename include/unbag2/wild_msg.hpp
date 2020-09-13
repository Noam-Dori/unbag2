//
// Created by Noam Dori on 17/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#ifndef UNBAG2_WILD_MSG_HPP
#define UNBAG2_WILD_MSG_HPP

#include <memory>
#include <rosidl_runtime_c/action_type_support_struct.h>
#include <rcutils/types.h>
#include <rmw/rmw.h>

namespace unbag2
{
/**
 * \brief describes a deserializable message along with its topic and associated type.
 */
class WildMsg
{
public:
  /**
   * \brief wrap a serialized message
   * \param data the serialized data
   * \param topic the topic from which the data came from
   * \param type the message type of the data.
   */
  WildMsg(std::shared_ptr<rcutils_uint8_array_t> data, std::string topic, rosidl_message_type_support_t type);

  /**
   * \return whether or not a pipe successfully processed this message
   */
  bool processed() const;

  /**
   * \return the human readable message type of this message
   */
  std::string type() const;

  /**
   * \return the topic where this message was obtained
   */
  std::string topic() const;

  /**
   * \brief attempt to write this message into a deserialized form.
   * \tparam Msg the type of the deserialized message
   * \return the deserialized message.
   */
  template<class Msg> Msg deserialize() const
  {
    Msg ret;
    if(rmw_deserialize(data_.get(), &type_, &ret))
    { // fairly useless bit of code used to suppress a warning.
    }
    return ret;
  }

  /**
   * \brief adds a pipe's process to this message.
   * \param success the success of the pipe.
   */
  void feed_pipe(bool success);
private:
  std::shared_ptr<rcutils_uint8_array_t> data_;
  std::string topic_;
  rosidl_message_type_support_t type_;
  bool processed_ = false;
};
}

#endif //UNBAG2_WILD_MSG_HPP
