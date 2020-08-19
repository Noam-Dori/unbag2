//
// Created by Noam Dori on 17/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/wild_msg.hpp>

using std::move;
using std::shared_ptr;
using std::string;

namespace unbag2
{
WildMsg::WildMsg(shared_ptr<rcutils_uint8_array_t> data, string topic, rosidl_message_type_support_t type) :
    data_(move(data)), topic_(move(topic)), type_(type)
{
}

string WildMsg::type() const
{
  return type_.typesupport_identifier;
}

string WildMsg::topic() const
{
  return topic_;
}
}