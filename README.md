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
1. Declare a class that implements `unbag2::Pipe` or `unbag2::PipeBase<class>`
   and add the plugin macro to it.
   See `include/unbag2/gps_pipe.hpp` as an example
2. Implement `process`, `on_bag_end`, `on_unbag_end`, and optionally `load_pipe_params` or `can_process`.
   If you implemented PipeBase, your life will be easier.
   See `src/pipe/gps_pipe.cpp` as an example for implementation. See `include/unbag2/pipe.hpp` for API documentation.
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
target_link_libraries(<LIBRARY NAME> unbag2_pipe)
pluginlib_export_plugin_description_file(<PKG> default_pipes.xml)
```
These will declare and export your plugin so unbag2 knows about them.