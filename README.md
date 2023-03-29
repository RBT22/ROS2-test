# ROS2-test

![colcon workflow](https://github.com/RBT22/ROS2-test/actions/workflows/main.yml/badge.svg)


## Checklist

- [x] Create a public repository on GitHub
- [x] Create a dockerized development environment that includes a ROS workspace
    - [x] Create a Dockerfile and a corresponding docker-compose.yml in the root of the
repo
    - [x] Create a ROS workspace for your packages inside the repo
    - [x] Use ROS Humble
    - [x] Create an apt-dependencies.txt and pip-dependencies.txt file that contains
dependencies to be installed from the Dockerfile by apt and pip
    - [x] Mount the workspace as a volume
- [x] Inside this environment, install and set up the Navigation2 stack (see the Getting started
guide)
- [x] Create your own ROS package with build type ament_python, named
“navigation_controller”
- [x] Add a launch file to your package that launches the nav2_bringup package’s
tb3_simulation_launch.py
- [x] Add a python script that implements a node
    - [x] The node should implement an action client that calls nav2’s NavigateToPose
and sends the turtlebot to a location
    - [x] The goal location should be given as ROS parameters (x and y positions and
theta yaw rotation)
    - [x] The node should print feedback to the terminal and terminate when the action
terminates
    - [x] Add this node to the launch file
- [x] Add RViz to the launch file (share the host machine’s X server to be able to see the RViz
window)
- [x] Commit and push your work with descriptive commit messages


