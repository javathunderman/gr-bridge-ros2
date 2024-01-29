# gr-bridge

A C++ based ROS2 package for sending data from a GNURadio runtime to a ROS2 pub/sub topic.

Originally intended for use with the SDR DOA project, but the [Python version](https://github.com/javathunderman/gr-ros2-pubsub) does the DOA
in the ROS2 package runtime rather than awkwardly relying on binding to a Python file and calling the functions from C++.