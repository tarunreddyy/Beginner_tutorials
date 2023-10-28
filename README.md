# ROS 2 Beginner Tutorials

This package is part of a series of tutorials for learning ROS 2, the Robot Operating System version 2. It contains a simple publisher node named `talker` that publishes a custom string message to a topic named `chatter`.

## Prerequisites

- ROS 2 installation (e.g., [Foxy Fitzroy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/), [Galactic Geochelone](https://index.ros.org/doc/ros2/Releases/Release-Galactic-Geochelone/), or [Humble Hawksbill](https://index.ros.org/doc/ros2/Releases/Release-Humble-Hawksbill/))
- Basic understanding of ROS 2 concepts and terminology

## Building the Package

* Navigate to the root of your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws
   ```

* Source your ROS 2 installation:
   ```bash
   source /opt/ros/<distro>/setup.bash
   ```

* Build your workspace:
   ```bash
   colcon build
   ```

## Assignment 1
### Running the Publisher Node

* Source the setup files for your ROS 2 workspace:
   ```bash
   source install/setup.bash
   ```

* Run the `talker` node:
   ```bash
   ros2 run beginner_tutorials talker
   ```

### Verifying the Published Messages

* Open a new terminal and source the setup files for your ROS 2 workspace:
   ```bash
   source /opt/ros/<distro>/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

* Echo the messages published on the `chatter` topic:
   ```bash
   ros2 topic echo /chatter std_msgs/msg/String
   ```

You should see messages printed to the terminal indicating that the `talker` node is publishing messages to the `chatter` topic.

## License

This package is licensed under the MIT License. For more details, see the [LICENSE](./LICENSE) file.

## Contributing

Contributions to this package are welcome. Please adhere to the [ROS 2 Contributing Guidelines](https://index.ros.org/doc/ros2/Contributing/) when making a contribution to this package.


Make sure to replace `<distro>` with the codename of your ROS 2 distribution. If your ROS 2 distribution is Humble Hawksbill, replace `<distro>` with `humble`. This README provides a basic overview of your package, instructions for building and running the `talker` node, and information on verifying the published messages.