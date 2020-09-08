//
// Created by Noam Dori on 08/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <jsoncpp/json/json.h>
#include <algorithm>

#ifndef UNBAG2_JSON_UTILS_HPP
#define UNBAG2_JSON_UTILS_HPP

namespace unbag2
{
class JsonUtils
{
public:
  template<size_t size>
  static void add_covariance(Json::Value & out, std::array<double, size> covariance, bool force)
  {
    if (force || std::any_of(covariance.begin(), covariance.end(), [](double d){return d != 0;}))
    {
      for (auto item : covariance)
      {
        out.append(item);
      }
    }
  }

  template <class T>
  static void try_w(Json::Value & out, const T & potential_w, decltype(&T::w))
  {
    out["w"] = potential_w.w;
  }
  template<class T>
  static void try_w(Json::Value &, const T &, ...)
  {
  }

  template<class T>
  static Json::Value to_json(T point_or_quaternion)
  {
    Json::Value ret;
    ret["x"] = point_or_quaternion.x;
    ret["y"] = point_or_quaternion.y;
    ret["z"] = point_or_quaternion.z;
    try_w(ret, point_or_quaternion, nullptr);
    return ret;
  }
};
}

#endif //UNBAG2_JSON_UTILS_HPP
