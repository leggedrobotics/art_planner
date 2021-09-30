# ANYmal Rough Terrain Planner

Sampling based path planning for ANYmal, based on 2.5D height maps.
More detailed instructions still to come.

**Author:** Lorenz Wellhausen

**Maintainer:** Lorenz Wellhausen, [lorenwel@ethz.ch](lorenwel@ethz.ch)

©2021 ETH Zurich

## Build
[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/art_planner/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/art_planner/job/master/)

## Dependencies

### art_planner

The base package has the following dependencies:

- [OMPL \[v1.4.2\]](https://github.com/ompl/ompl)
- [grid\_map\_core](git@github.com:ANYbotics/grid_map.git)

You can install them from source if you want to use the planner independent of ROS.
If you're using the ROS interface, installation is even easier and can be done through PPA.
We ship our own catkinized version of ODE, which has modifications in the height field collision checking.
This means that if you build this package in a workspace with other packages which also require ODE this version might be used.

`sudo apt install ros-melodic-ompl ros-melodic-grid-map-core`

**Warning:** Do NOT install the `libompl-dev` package from PPA as that one is an imcompatible version which breaks things.

### art\_planner\_ros

The dependencies of the ROS interface can be installed with the following command:

`sudo apt install ros-melodic-actionlib ros-melodic-geometry-msgs ros-melodic-grid-map-msgs ros-melodic-grid-map-ros ros-melodic-nav-msgs ros-melodic-roscpp ros-melodic-tf2-geometry-msgs ros-melodic-tf2-ros`

## Usage

We provide a launch file which should be everything you need, if you work with ANYmal.

`roslaunch art_planner_ros art_planner.launch`

In case you do not have your own path follower, you can use our hacky and unsupported path follower.

`rosrun art_planner_ros path_follower.py`

For this one to work you need to manually start your desired motion controller.

### Configuration

The config file which is loaded when following the instructions above is located in `art_planner_ros/config/params.yaml`.
It has extensive comments describing the function of each parameter.

The defaults should be fine for ANYmal C.

You can use the `2D Nav Goal` in RViz to set a goal pose for the planner.

## TODO

Although the planner is overall pretty :fire::fire::fire::100::fire::fire::fire: some things are still :poop:.

### Known issues

- Catkinized ODE version might be pulled in as dependency by other packages in workspace :bowling:
