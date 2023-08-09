# ANYmal Rough Terrain Planner

Sampling based path planning for ANYmal, based on 2.5D height maps using learned motion cost.

**Author:** Lorenz Wellhausen

**Maintainer:** Lorenz Wellhausen, [lorenwel@ethz.ch](lorenwel@ethz.ch)

©2021 ETH Zurich

If you use this work in an academic context, please cite:

Use with motion cost (_Default_, `prm_motion_cost`) [Paper link](https://arxiv.org/abs/2303.01420):

```
@inproceedings{wellhausen2023artplanner,
  title={ArtPlanner: Robust Legged Robot Navigation in the Field},
  author={Wellhausen, Lorenz and Hutter, Marco},
  booktitle={Field Robotics},
  year={2023}
}
```

Use without motion cost (`lazy_prm_star_min_update`) [Paper link](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/507668/1/2021_iros_wellhausen_planner_final_version.pdf):

```
@inproceedings{wellhausen2021rough,
  title={Rough Terrain Navigation for Legged Robots using Reachability Planning and Template Learning},
  author={Wellhausen, Lorenz and Hutter, Marco},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2021)},
  year={2021}
}
```

## Build
[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/art_planner/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/art_planner/job/master/)

## Dependencies

### Cloning

Before cloning this repo, make sure that [Git LFS](https://git-lfs.com/) is installed:

```
sudo apt install git-lfs
git lfs install
```

### art_planner

The base package has the following dependencies:

- [OMPL \[v1.4.2\]](https://github.com/ompl/ompl)
- [OpenCV](https://github.com/opencv/opencv)
- [grid\_map\_core](https://github.com/ANYbotics/grid_map)

You can install them from source if you want to use the planner independent of ROS.
If you're using the ROS interface, installation is even easier and can be done through PPA (and OpenCV should be installed by default).
We ship our own catkinized version of ODE, which has modifications in the height field collision checking.
This means that if you build this package in a workspace with other packages which also require ODE this version might be used.

`sudo apt install ros-noetic-ompl ros-noetic-grid-map-core`

**Warning:** Do NOT install the `libompl-dev` package from PPA which is an imcompatible version and breaks things.

### art\_planner\_ros

The dependencies of the ROS interface can be installed with the following command:

`sudo apt install ros-noetic-actionlib ros-noetic-geometry-msgs ros-noetic-grid-map-msgs ros-noetic-grid-map-ros ros-noetic-nav-msgs ros-noetic-roscpp ros-noetic-tf2-geometry-msgs ros-noetic-tf2-ros`

Tested on ROS noetic.

### art\_planner\_motion\_cost

This package contains everything related to motion cost inference.
This is all done in Python and Pytorch, therefore, you need to install these Python packages:

`pip3 install opencv-python rospkg`

Also, [install Pytorch](https://pytorch.org/) (tested with version 1.13). For example with:

`pip3 install torch torchvision torchaudio`

## Usage

We provide a launch file which should be everything you need, if you work with ANYmal.
Note that the ANYmal simulation stack is only available through [ANYmal Research](https://www.anymal-research.org/) and I can therefore not provide a minimal working solution.

`roslaunch art_planner_ros art_planner.launch`

In case you do not have your own path follower, you can use our hacky and unsupported path follower.

`rosrun art_planner_ros path_follower.py`

For this one to work you need to manually start your desired motion controller.

### Configuration

The config file which is loaded when following the instructions above is located in `art_planner_ros/config/params.yaml`.
It has extensive comments describing the function of each parameter.

The defaults should be fine for ANYmal C.

You can use the `2D Nav Goal` in RViz to set a goal pose for the planner.

#### **Planning Method**

By default, `prm_motion_cost` is used as planning method. This uses a learned motion cost as planning objective, as described in `wellhausen2023artplanner`.
The included neural network weights for computing the motion cost are trained for blind and perceptive policies of the ANYmal robot.
Unforunately, we cannot provide the environment for you to retrain it for your robot.
Therefore, if you want to try this planner on a robot with very different traversability characteristics than ANYmal, you might be better off using the purely geometric `lazy_prm_star_min_update` method, as described in `wellhausen2021rough`.



### Height Map

We recommend using the [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) package for height mapping.
It's what we use as input to the planner and will provide a `traversability` layer by default, which is used to restrict steppable terrain.
It can also output an `upper_bound` layer, which computes the maximum terrain height in unobserved map cells via ray-tracing and improves navigation in complex terrain.

We know that [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) also works, if you only have a CPU but will not provide `traversability` or `upper_bound`.

### ROS Interface

#### **Subscribers**

`~elevation_map`: Expects a `grid_map_msgs::GridMap` message containing the height map used for planning

The robot pose is obtained from the TF tree (see `robot/base_frame` in the config file).

#### **Publishers**

`~path`: Outputs a `nav_msgs::Path` message with the computed path.

`~map`: Outputs the height map used for computing the published path as `grid_map_msgs::GridMap`, with additional layers created during planning (such as sampling probability and inpainted traversability).
These are a lot of layers, which makes the published message quite heavy. Therefore, only subscribe to it, when you also really want to look at it (it's only published if there are subscribers).

`~planning_time`: Publishes the time used for planning for the latest output path as `std_msgs::Float64`.

#### **Action Server**

`~plan_to_goal`: Custom action `art_planner_msgs::PlanToGoalAction` to contiuously plan towards a goal. Can be called with a `2D Nav Goal` in Rviz using `art_planner_ros/scripts/plan_to_goal_client.py`.

#### **Service Server**

`~plan`: A `nav_msgs::GetPlan` service to request a single path. The `tolerance` field is ignored in favor of the `planner/start_goal_search/goal_radius` parameter.

## TODO

Although the planner is overall pretty :fire::fire::fire::100::fire::fire::fire: some things are still :poop:.

### Known issues (PRs welcome!)

- Catkinized ODE version might be pulled in as dependency by other packages in workspace :bowling:
- This method does not use LLMs or RL, so using it basically makes you a boomer :older_man:
