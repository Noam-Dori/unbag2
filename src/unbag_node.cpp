//
// Created by Noam Dori on 11/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/unbag_node.hpp>
#include <unbag2/generic_subscription.hpp>
#include <unbag2/wild_msg.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <glob.h>
#include <regex>

using boost::filesystem::current_path;
using boost::filesystem::directory_iterator;
using boost::filesystem::is_directory;
using boost::filesystem::is_regular_file;
using boost::filesystem::path;
using pluginlib::ClassLoader;
using pluginlib::PluginlibException;
using rclcpp::Node;
using rclcpp::SerializedMessage;
using rclcpp::sleep_for;
using rclcpp::spin;
using rosbag2_cpp::ConverterOptions;
using rosbag2_cpp::Reader;
using rosbag2_cpp::StorageOptions;
using rosbag2_cpp::readers::SequentialReader;
using rosbag2_cpp::get_typesupport_handle;
using rosbag2_cpp::get_typesupport_library;
using std::chrono_literals::operator""ms;
using std::istream_iterator;
using std::istringstream;
using std::regex;
using std::regex_replace;
using std::shared_ptr;
using std::string;
using std::unordered_map;
using std::vector;

namespace unbag2
{
UnbagNode::UnbagNode() : Node("unbag"), plugin_loader_("unbag2","unbag2::Pipe")
{
  declare_parameter("mode", "post");
  declare_parameter("split_by_bag",false);
  declare_parameter("target_dir", ".");
  declare_parameter("files", "");
}

int UnbagNode::run_on_args()
{
  auto unbag = std::make_shared<UnbagNode>();
  unbag->init_plugins();
  if (unbag->pipes_.empty())
  {
    RCLCPP_FATAL(unbag->get_logger(), "no pipe found to process data... aborting");
    return 1;
  }
  auto mode = unbag->get_parameter("mode").as_string();
  if (mode == "post")
  {
    istringstream file_param(unbag->get_parameter("files").as_string());
    for (const auto & path : vector<string>{istream_iterator<string>{file_param}, istream_iterator<string>{}})
    {
      unbag->add_source(path);
    }
    unbag->run_on_files();
  }
  else if (mode == "live")
  {
    unbag->init_subscribers();
    spin(unbag);
  }
  else
  {
    RCLCPP_FATAL(unbag->get_logger(), "unknown unbag mode %s", mode.c_str());
    return 1;
  }
  return 0;
}
void UnbagNode::init_plugins()
{
  for (const auto & class_name : plugin_loader_.getDeclaredClasses())
  {
    try
    {
      auto pipe = plugin_loader_.createSharedInstance(class_name);
      pipe->load_params(this);
      if (pipe->enabled())
      {
        pipes_.push_back(pipe);
      }
    }
    catch (const PluginlibException& e)
    {
      RCLCPP_ERROR(get_logger(), "plugin %s failed to load. Reason: %s", class_name.c_str(), e.what());
    }
  }
}
void UnbagNode::init_subscribers()
{
  sleep_for(100ms); // gives the node time to register all the available topics
  for(const auto & topic : get_topic_names_and_types())
  {
    auto ts = get_typesupport_handle(topic.second[0], "rosidl_typesupport_cpp",
                                     get_typesupport_library(topic.second[0], "rosidl_typesupport_cpp"));
    subscriptions_.emplace_back(get_node_base_interface().get(), *ts, topic.first, [this, &topic, &ts](auto && msg)
    {
      unbag_callback(msg, topic.first, ts);
    });
  }
}

void UnbagNode::add_source(const string& input_path)
{
  path query_path = input_path[0] == '/' ? input_path : (current_path() / input_path);

  // next we need to take care of ../ and ./ in the path
  regex folder_regex("/\\.(?=/|$)"), parent_regex("/[^/]+/\\.\\.(?=/|$)");
  query_path = regex_replace(query_path.string(), folder_regex, "");
  query_path = regex_replace(query_path.string(), parent_regex, "");

  // now we convert wildcards (*) to be all files
  glob_t glob_result;
  int glob_status = glob(query_path.c_str(), uint(1) << uint(12), nullptr, &glob_result);
  if (glob_status != 0)
  {
    globfree(&glob_result);
    RCLCPP_ERROR(get_logger(), "no file found matching pattern %s derived from %s, skipping...",
                 query_path.c_str(), input_path.c_str());
    return;
  }
  for (size_t i = 0; i < glob_result.gl_pathc; ++i)
  {
    // now that we have all URIs we need to make sure the paths are not directories.
    // If they are, we add their children to the list.
    path file_path = glob_result.gl_pathv[i];
    if (is_directory(file_path))
    {
      for (auto& child : directory_iterator(file_path))
      {
        if (is_regular_file(child.path()) && child.path().extension() == ".bag")
        {
          bag_uris_.push_back("file://" + child.path().string());
        }
      }
    }
    else
    {
      bag_uris_.push_back("file://" + file_path.string());
    }
  }
}
void UnbagNode::run_on_files()
{
  // bag files are apparently SQL databases using SQLite3 library.
  StorageOptions storage_options{"","sqlite3"};
  ConverterOptions converter_options{"cdr","cdr"};

  unordered_map<string, rosidl_message_type_support_t> ts_map;
  bool split = get_parameter("split_by_bag").as_bool();

  for (const auto & uri : bag_uris_)
  {
    storage_options.uri = uri;
    Reader reader(std::make_unique<SequentialReader>());
    reader.open(storage_options, converter_options); // in the future all we will need is the URI thanks to API upgrades
    if (!reader.has_next())
    {
      continue;
    }

    for(auto const & topic_type : reader.get_all_topics_and_types())
    {
      auto ts = get_typesupport_handle(topic_type.type,"rosidl_typesupport_cpp",
                                       get_typesupport_library(topic_type.type,"rosidl_typesupport_cpp"));
      ts_map.emplace(topic_type.name, *ts);
    }

    for (auto serialized = reader.read_next(); reader.has_next() ; serialized = reader.read_next())
    {
      WildMsg msg(serialized->serialized_data, serialized->topic_name, ts_map[serialized->topic_name]);
      for (const auto & pipe : pipes_)
      {
        if (pipe->can_process(msg))
        {
          pipe->process(msg);
        }
      }
    }
    if (split && uri != bag_uris_.back())
    {
      for (const auto & pipe : pipes_)
      {
        pipe->on_bag_end();
      }
    }
  }
  for (const auto & pipe : pipes_)
  {
    pipe->on_unbag_end();
  }
}
void UnbagNode::unbag_callback(const shared_ptr<SerializedMessage> & data, const string & topic,
                               const rosidl_message_type_support_t * ts)
{
  WildMsg msg(std::make_shared<rcutils_uint8_array_t>(data->release_rcl_serialized_message()), topic, *ts);
  for (const auto & pipe : pipes_)
  {
    if (pipe->can_process(msg))
    {
      pipe->process(msg);
    }
  }
}
}