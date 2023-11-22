# ROS 2 Beginner Tutorials

This ROS 2 package is part of a series of tutorials that provide hands-on instructions for learning ROS 2 functionalities. It features three main components:

1. `talker` Node: A publisher that continuously broadcasts a string message on the `chatter` topic and a TF frame called `/talk` with parent `/world`.
2. `listener` Node: A subscriber that listens to the `chatter` topic and logs the messages it receives.
3. `change_string_client` Node: A service client that requests a change in the string message being published by the `talker`.

## Prerequisites

Ensure you have ROS 2 installed on your system. This tutorial assumes you are using the Humble Hawksbill distribution, but it can be adapted for others like Foxy Fitzroy or Galactic Geochelone.

## Building the Package

Follow these steps to build the package:

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

After building the package, source the setup file of your workspace:

```bash
source install/setup.bash
```

- To run the `talker` node:

  ```bash
  ros2 run beginner_tutorials talker
  ```

- To execute the `listener` node:

  ```bash
  ros2 run beginner_tutorials listener
  ```

- To invoke the `change_string_client` node:

  ```bash
  ros2 run beginner_tutorials change_string_client "Your New String Here"
  ```

## Inspecting TF Frames

- To verify the TF frames broadcasted by the `talker` node:

  ```bash
  ros2 run tf2_ros tf2_echo /world /talk
  ```

- To visualize the TF frames using `view_frames`:

  ```bash
  ros2 run tf2_tools view_frames
  ```

  The output PDF will be saved in the `frames.pdf` file in your current directory.

## Running Nodes Using a Launch File

Launch files in ROS 2 are used to start multiple nodes with specific configurations. This package includes a launch file that initializes the `talker` node and records ROS 2 bag files.

- To start nodes with bag recording enabled (records for ~15 seconds):

  ```bash
  ros2 launch beginner_tutorials record_bag.launch.py record_bag:=true
  ```

- To inspect the recorded bag file:

  ```bash
  ros2 bag info output_bag_0.db3
  ```

- To replay the recorded bag file:

  1. Run the `listener` node in one terminal:

     ```bash
     ros2 run beginner_tutorials listener
     ```

  2. Navigate to the output_bag directory and play the bag file in another terminal:

     ```bash
     ros2 bag play output_bag_0.db3
     ```

## Integration Testing with gtest

- The package includes integration tests written with GoogleTest. To run the tests:

  ```bash
  colcon test --packages-select beginner_tutorials
  ```

  Check the test results with:

  ```bash
  cat log/latest_test/beginner_tutorials/stdout_stderr.log
  ```

## Test Output

```bash
test 1
    Start 1: test_talker

1: Test command: /usr/bin/python3.10 "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/root/new_ros2_ws/build/beginner_tutorials/test_results/beginner_tutorials/test_talker.gtest.xml" "--package-name" "beginner_tutorials" "--output-file" "/root/new_ros2_ws/build/beginner_tutorials/ament_cmake_gtest/test_talker.txt" "--command" "/root/new_ros2_ws/build/beginner_tutorials/test_talker" "--gtest_output=xml:/root/new_ros2_ws/build/beginner_tutorials/test_results/beginner_tutorials/test_talker.gtest.xml"
1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '/root/new_ros2_ws/build/beginner_tutorials':
1:  - /root/new_ros2_ws/build/beginner_tutorials/test_talker --gtest_output=xml:/root/new_ros2_ws/build/beginner_tutorials/test_results/beginner_tutorials/test_talker.gtest.xml
1: [==========] Running 2 tests from 1 test suite.
1: [----------] Global test environment set-up.
1: [----------] 2 tests from TestTalkerNode
1: [ RUN      ] TestTalkerNode.TalkerPublishesMessage
1: [INFO] [1700611010.972829140] [talker_test_node]: I heard: 'Hello World!'
1: [       OK ] TestTalkerNode.TalkerPublishesMessage (781 ms)
1: [ RUN      ] TestTalkerNode.TalkerBroadcastsTransform
1: [WARN] [1700611011.083362778] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [WARN] [1700611011.185956951] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [WARN] [1700611011.288185255] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [WARN] [1700611011.390542673] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [WARN] [1700611011.492943595] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [WARN] [1700611011.595089536] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [WARN] [1700611011.697522505] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [WARN] [1700611011.799711663] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [WARN] [1700611011.901995709] [talker_test_node]: TF Exception: "world" passed to lookupTransform argument target_frame does not exist. 
1: [INFO] [1700611011.973884049] [talker_test_node]: I heard: 'Hello World!'
1: [       OK ] TestTalkerNode.TalkerBroadcastsTransform (1001 ms)
1: [----------] 2 tests from TestTalkerNode (1782 ms total)
1: 
1: [----------] Global test environment tear-down
1: [==========] 2 tests from 1 test suite ran. (1782 ms total)
1: [  PASSED  ] 2 tests.
1: -- run_test.py: return code 0
1: -- run_test.py: inject classname prefix into gtest result file '/root/new_ros2_ws/build/beginner_tutorials/test_results/beginner_tutorials/test_talker.gtest.xml'
1: -- run_test.py: verify result file '/root/new_ros2_ws/build/beginner_tutorials/test_results/beginner_tutorials/test_talker.gtest.xml'
1/4 Test #1: test_talker ......................   Passed    1.93 sec

100% tests passed, 0 tests failed out of 4

Label Time Summary:
cppcheck      =   0.09 sec*proc (1 test)
gtest         =   1.93 sec*proc (1 test)
lint_cmake    =   0.07 sec*proc (1 test)
linter        =   0.55 sec*proc (3 tests)
xmllint       =   0.39 sec*proc (1 test)

Total Test time (real) =   2.48 sec
```

## Coding Standards and Quality Checks

- Ensure the code follows the Google C++ Style Guide.
- Run `cpplint` and `cppcheck` for code quality checks:

  ```bash
  cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ...
  cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem ...
  ```

- The outputs from these tools should be saved as text files in the repository.

## Contributing

To contribute to this package, please follow the guidelines set forth in the [ROS 2 Contributing Guidelines](https://index.ros.org/doc/ros2/Contributing/). Contributions that enhance the tutorials or improve the codebase are highly encouraged.

## License

This package is provided under the MIT License, which allows for a wide range of uses with minimal restrictions.