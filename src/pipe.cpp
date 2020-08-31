//
// Created by Noam Dori on 12/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/pipe.hpp>
#include <regex>
#include <utility>

using boost::filesystem::create_directories;
using boost::filesystem::current_path;
using boost::filesystem::exists;
using boost::filesystem::is_directory;
using boost::filesystem::path;
using rclcpp::Node;
using std::move;
using std::regex;
using std::string;

namespace unbag2
{
bool Pipe::enabled() const
{
  return enabled_;
}
void Pipe::load_params(Node * node)
{
  enabled_ = node->declare_parameter(to_param("enabled"), true);
  path target_dir = node->declare_parameter("target_dir", "");

  if(!target_dir.is_absolute())
  {
    target_dir = current_path() / target_dir;
  }

  // next we need to take care of ../ and ./ in the path
  regex folder_regex("/\\.(?=/|$)"), parent_regex("/[^/]+/\\.\\.(?=/|$)");
  target_dir = regex_replace(target_dir.string(), folder_regex, "");
  target_dir = regex_replace(target_dir.string(), parent_regex, "");

  if (!exists(target_dir))
  {
    target_dir_ = target_dir;
    create_directories(target_dir_);
  }
  else
  {
    target_dir_ = is_directory(target_dir) ? target_dir : target_dir.parent_path();
  }

  load_pipe_params(node);
}
string Pipe::to_param(const string & param)
{
  return prefix_ + "." + param;
}

Pipe::Pipe(string name): prefix_(move(name))
{
}

void Pipe::load_pipe_params(rclcpp::Node *)
{
}

void Pipe::on_bag_end()
{
}

void Pipe::on_unbag_end()
{
}
}
