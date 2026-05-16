# Ianvs

Ianvs (Janus) is a ROS2 c++ package that provides a transition path for common ROS1 design patterns that aren't as easily supported in ROS2.

This includes:
- A `NodeHandle` class that works with any ROS2 `NodeBase` implementation and provides namespaced access to underlying node
- Utility functions for handling getting single messages
- Specialized image publishers and subscribers that work with message filters

Ianvs is intended to only depend on a very select number of core ROS2 dependencies. These are currently:
- `message_filters`
- `rclcpp`
- `std_srvs`
- `sensor_msgs`

## Launch File Extensions

Ianvs includes a custom launch file extension for `node` that allows the use of specific virtual environments
for a node besides the system environment (or the environment the node is built with). This works with `--symlink-install`.

The underlying mechanism for this is to use the interpreter for the virtual environment for the node launch prefix, which is appended
to the launch prefix of the node if it's already specified, or becomes the launch prefix of the node otherwise.
You can manually replicate the same behavior if you so choose.
This method will not work (at the moment) for child processes (because the child process will use the python interpreter specified by `PATH` which is not overridden).

You can enable this via:
```yaml
launch:
  # other launch file contents
  - pyenv_node:
      pkg: some_package
      exec: some_python_node
      name: some_node_name
      pyenv: /path/to/python/environment
      # whatever else you want
```
This will fail for non-python executables.

## Utilities

Ianvs includes an utility for playing a rosbag while modifying remappings, `/tf_static` and `/tf`.
The general usage is below:
```
ros2 run ianvs /path/to/bag [-p prefix] [-s expr:sub]... [-f filter]... [-- ros2 bag play args]
```
The `--` separator is optional in general, but necessary to deconflict with the `--start-paused` or `--storage` flags for the `ros2 bag play` command.

For the most part, this has identical options to the `ros2 bag play` command, but does not handle QoS service overrides at the moment (the parsing of the QoS overrides from YAML happens in the `ros2bag` python executable instead of `rosbag2_transport`). We hope to (mostly) maintain feature parity.

> :warning: **Warning** </br>
> The way the underlying rosbag player is wrapped is a little sketchy still and might occasionally die

# About the Name

Ianvs is the latin spelling of Janus, the Roman deity of doorways and transitions
