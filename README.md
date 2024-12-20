# Robot Collision Checking

A lightweight package to use FCL with ROS messages that is heavily inspired by [MoveIt's version](https://moveit.ros.org/documentation/concepts/developer_concepts/).  

The `robot_collision_checking` package can be utilised to perform distance and collision checking of objects by creating and maintaining a collision world and/or by using utility functions (see the [API Documentation](docs/api.md) for more information). This package can handle objects represented as [shape_msgs](http://wiki.ros.org/shape_msgs), 
[OctoMaps](https://github.com/OctoMap/octomap_msgs), and [VoxelGrids](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/VoxelGrid.msg).

Depending on which git branch is used, implementations for the following ROS distros are available:
- ROS 1 Noetic on the `noetic-devel` branch,
- ROS 2 Foxy on the `foxy` branch, and 
- ROS 2 Humble on the default `humble` branch.

The package was developed and tested on Ubuntu 20.04 for Noetic/Foxy and Ubuntu 22.04 for Humble. However, any operating systems supported by the ROS distros available
to this package should also work.
We recommend using the default ROS 2 Humble implementation, as this continues to have ongoing support.

In terms of third-party software, this package requires:
 * [FCL](https://github.com/flexible-collision-library/fcl) version **0.7.0**
 * [libccd](https://github.com/danfis/libccd)
 * [Octomap](https://octomap.github.io/) 

## Installation

FCL and `libccd-dev` may already be installed on your machine via your ROS distro, but these versions are likely outdated for the current repository's use. 
The following instructions will enable you to build the `robot_collision_checking` package within a ROS 2 workspace using `colcon build` (or `catkin build` if using ROS 1).

### libccd

Run the following commands to build `libccd` from source:
```
git clone https://github.com/danfis/libccd.git
cd libccd && mkdir build && cd build
cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON ..
make
sudo make install
```

### FCL

**Important:** Before installing FCL, make sure to have `liboctomap-dev` installed, e.g.,
```
sudo apt install liboctomap-dev
```
as FCL will ignore building `OcTree` collision geometries otherwise.
Once Octomap is installed, run the following commands to build `fcl` from source:

```
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl && mkdir build && cd build
cmake ..
make
sudo make install
```

If there are errors, such as constants not being found, then you are probably still using the older version of FCL.

### Workspace Setup

You can now clone the `robot_collision_checking` repository in your ROS workspace (set to the appropriate `$ROS_DISTRO` branch). Don't forget to install 
any other system dependencies through `rosdep` after installing the above libraries, e.g., in the root directory of your ROS workspace run:
```
rosdep install --from-paths src --ignore-src -y
```

## Alternative - Docker Image

If you instead wish to explore the package in a Docker image, there is a `Dockerfile` available. Simply clone the repository or download the `Dockerfile` and
then run:
```
docker build --tag 'robot_collision_checking' . && docker run -it 'robot_collision_checking' bash
```

## Testing

You can run tests for the `robot_collision_checking` package as described in [this ROS 2 tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/CLI.html). First compile the tests:
```
colcon test --ctest-args tests
```

And then examine the results:
```
colcon test-result --all --verbose
```

## Examples

A toy example is provided in the `examples` directory and can be run as follows:
```
ros2 run robot_collision_checking fcl_interface_example
```
Separately run an instance of `rviz2` and set the global fixed frame to "world" to visualize the collision world. You can install `rviz2` on Debian systems by running:
```
sudo apt install ros-$ROS_DISTRO-rviz2
```

Within this ROS node, a few key pieces of functionality are provided:
- First, the `initCollisionWorld()` method demonstrates how a collision world composed of different geometric shapes and types (meshes, planes, voxel grids, etc.) can be constructed
and maintained using the package's interface.
- Second, the main publishing loop indicates how these different geometric types can be translated into [visualization_msgs/Marker](https://wiki.ros.org/rviz/DisplayTypes/Marker) 
messages for visualization in RViz.
- Finally, the example shows how the created collision world can be used to check for collisions and distances between its constituent objects. 
The output of the example node prints information about any objects currently in collision.

While this example only contains static objects, the package also works with dynamic objects. 

A more extensive use-case of this package is provided in [constrained_manipulability](https://github.com/philip-long/constrained_manipulability).
Here, the `robot_collision_checking` interface checks for collisions and distances between environmental objects and a robot manipulator (based on the geometric shapes
present in its URDF model). 

## Contributing

Contributions are always welcome! If you encounter any issues or need assistance, feel free to open a GitHub issue. 
We'd also love to hear how you're using the package—don't hesitate to share your experience or ideas with the community.
