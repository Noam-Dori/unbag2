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
class WildMsg
{
public:
  WildMsg(std::shared_ptr<rcutils_uint8_array_t> data, std::string topic, rosidl_message_type_support_t type);

  std::string type() const;

  std::string topic() const;

  template<class Msg> Msg deserialize() const
  {
    Msg ret;
    rmw_deserialize(data_.get(), &type_, &ret);
    return ret;
  }
private:
  std::shared_ptr<rcutils_uint8_array_t> data_;
  std::string topic_;
  rosidl_message_type_support_t type_;
};
}

#endif //UNBAG2_WILD_MSG_HPP