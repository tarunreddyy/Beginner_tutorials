# ROS 2 Beginner Tutorials

This ROS 2 package is part of a series of tutorials that provide hands-on instructions for learning ROS 2 functionalities. It features three main components:

1. `talker` Node: A publisher that continuously broadcasts a string message on the `chatter` topic.
2. `listener` Node: A subscriber that listens to the `chatter` topic and logs the messages it receives.
3. `change_string_client` Node: A service client that requests a change in the string message being published by the `talker`.

## Prerequisites

Ensure you have ROS 2 installed on your system. This tutorial assumes you are using the Humble Hawksbill distribution, but it can be adapted for others like Foxy Fitzroy or Galactic Geochelone.

## Building the Package

To build the package:

1. Open a terminal and navigate to your ROS 2 workspace directory:
   ```bash
   cd ~/ros2_ws
   ```

2. Source your ROS 2 installation to set up the environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build your workspace using `colcon`:
   ```bash
   colcon build
   ```

## Running Nodes Individually

After building the package, make sure to source the setup file of your workspace:

```bash
source install/setup.bash
```

To run the `talker` node:

```bash
ros2 run beginner_tutorials talker
```

To execute the `listener` node:

```bash
ros2 run beginner_tutorials listener
```

To invoke the `change_string_client` node, which will request a new string to be published by the `talker`:

```bash
ros2 run beginner_tutorials change_string_client "Your New String Here"
```

## Running Nodes Using a Launch File

Launch files in ROS 2 are used to start multiple nodes with specific configurations. In this package, the launch file is set up to initialize the `talker` and `listener` nodes and then, after a delay of 5 seconds, to execute the `change_string_client` node to change the published string.

- To execute nodes using the launch file:
  
  ```bash
  ros2 launch beginner_tutorials talker_listener.launch.py
  ```

- To pass a custom string to the `talker` node via the launch file and have the `change_string_client` execute with a delay:

  ```bash
  ros2 launch beginner_tutorials talker_listener.launch.py new_string:='Your New String Here'
  ```

The `change_string_client` node will be invoked after a 5-second delay, demonstrating how to pass parameters and delay execution in a launch file.

## License

This package is provided under the MIT License, which allows for a wide range of uses with minimal restrictions.

## Contributing

To contribute to this package, please follow the guidelines set forth in the [ROS 2 Contributing Guidelines](https://index.ros.org/doc/ros2/Contributing/). Contributions that enhance the tutorials or improve the codebase are highly encouraged.