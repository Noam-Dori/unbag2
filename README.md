# Unbag2
This is the ROS2 version of unbag. Its job is to extract the data recorded in bag files or ROS topics into file formats
non-ROS people can use.

## Run
after running `colcon build` (`--symlink-install` is recommended):
```sh
ros2 run unbag2 unbag <files>
```
The script that runs unbag is also available in the cloned git repo:
```sh
~/dev_ws/src/unbag2/unbag <files>
```
You can probably alias either command to make it easier for you.

`<files>` is a list of the bag files you want to extract
This is a string of a list of paths split by spaces.
- Relative paths are relative to the current working directory.
- Wildcards (`*`) are allowed.
- `..` and `.` are also correctly parsed.
- Paths to directories will find all bag files in the directory (only 1 level)

Additional is done from `cfg/unbag2.yml`. See it for extra documentation about configuration.

## Extension
You can add more processors, or "pipes" to unbag via plugins.
To add a plugin:
1. Declare a class that extends `unbag2::Pipe`, `unbag2::PipeBase<class>`, or `unbag2::JsonPipe<class>`
   and add the plugin macro to it.
   For example:
```cpp
#include <unbag2/pipe/pipe_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>

#ifndef MY_PIPE_STRING_PIPE_HPP
#define MY_PIPE_STRING_PIPE_HPP

namespace my_pipe
{
/**
 * \brief a pipe specializing in logging incoming strings.
 */
class StringPipe : public PipeBase<std_msgs::msg::String>
{
public:
  /**
   * \brief construct a new String pipe. figures.
   */
  GpsPipe();

  void load_pipe_params(rclcpp::Node * node) override;

  void process(std_msgs::msg::String msg, const std::string & topic) override;

  void on_bag_end() override;

  void on_unbag_end() override;
private:
  bool print_warning_ = false;
};
}

PLUGINLIB_EXPORT_CLASS(unbag2::StringPipe, unbag2::Pipe) // NOLINT(cert-err58-cpp)

#endif //MY_PIPE_STRING_PIPE_HPP
```
2. Implement `process`, `on_bag_end`, `on_unbag_end`, and optionally `load_pipe_params` or `can_process`.
   See `include/unbag2/pipe.hpp`, `include/unabg2/pipe/pipe_base.hpp` and `include/unabg2/pipe/json_pipe.hpp` for API documentation.
   Example:
```cpp
#include <my_pipe/string_pipe.hpp>

using rclcpp::Node;
using std_msgs::msg::String;
using std::string;

namespace my_pipe
{

StringPipe::StringPipe() : PipeBase<String>("string_pipe")
{
}

void StringPipe::load_pipe_params(Node * node)
{
  print_warning_ = !node->declare_parameter<bool>(to_param("print_warning"), false);
}

void StringPipe::process(String msg, const string & topic)
{
  auto message = "got string [%s] from topic [%s]";
  if(print_warning_)
  {
    RCLCPP_WARN(get_logger(), message, msg.data.c_str(), topic.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), message, msg.data.c_str(), topic.c_str());
  }
}

void StringPipe::on_bag_end()
{
  RCLCPP_INFO(get_logger(), "finished processing a bag file");
}

void StringPipe::on_unbag_end()
{
  RCLCPP_INFO(get_logger(), "finished processing all files");
}
}
```
3. create a plugin.xml file that declares a new library and describes your new class(es):
```xml
<library path="<LIBRARY NAME>">
  <class type="<PKG>::<YOUR CLASS>" base_class_type="unbag2::Pipe">
    <description>SOME DESCRIPTION</description>
  </class>
  ... more classes if you added more than 1
</library>
```
4. add CMake commands in your CMakeLists.txt:
```cmake
# this segment of code is used to add a plugin
add_library(<LIBRARY NAME> SHARED <IMPL FILES...>)
target_include_directories(<LIBRARY NAME> PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
ament_target_dependencies(<LIBRARY NAME> rclcpp pluginlib)
pluginlib_export_plugin_description_file(<PKG> default_pipes.xml)
```
These will declare and export your plugin so unbag2 knows about them.