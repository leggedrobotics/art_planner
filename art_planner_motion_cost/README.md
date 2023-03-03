# Cost query ROS service

This package contains a Python node which provides a ROS service to compute motion cost.

Originally authored by Bowen Yang.
Restructured and maintained by Lorenz Wellhausen.

---

## ROS nodes

1. scripts/cost_query_server.py: Subscribes and receives a height map through ROS topic broadcast. Offers a service server, which computes the cost for a batch of motions

---

## How to use

### Setup

1. `catkin build art_planner_motion_cost`
2. In config/config.yaml:
    1. Change the `map_layer` of the grid_map_msgs which provides height information.
    2. Specify your desired `model_file`
3. Change height map topic in `launch/cost_query_server.launch`
4. Run `roslaunch art_planner_motion_cost cost_query_server.launch`.
5. Publish height maps and query service.

### Query

The service call has the following structure:

```
std_msgs/Header header
float32[]       query_poses
---
float32[]       cost_power
float32[]       cost_time
float32[]       cost_risk
```

We only use the `frame_id` field id of the `header`, to make sure that the `query_poses` are in the same frame as the height map we subscribe to.

`query_poses` is the flattened vector of a `[B x 6]`-shaped matrix, where `B` is the number of queried motions.
The matrix content, before flattening, should be:

```
[[target_x, target_y, target_yaw, start_x, start_y, start_yaw]
                               .
                               .
                               .
 [target_x, target_y, target_yaw, start_x, start_y, start_yaw]]
```

Make sure this matrix is stored in row-major format, before flattening.

### Services

There are two service servers to compute the motion cost, one which updates the map information, one which does not:

- `~/cost_query` Updates the height map and then queries the motion cost. This should be your default to use.
- `~/cost_query_no_update` This directly queries the motion cost, without first updating the height map. This can be useful to avoid processing overhead, if you want to query costs multiple times on the same height map. `~/cost_query` has to be called at least once before, otherwise this service call will fail.

## Citation

If you use this package in isolation in an academic context, please cite the following paper:

```
@inproceedings{yang2021real,
  title={Real-time optimal navigation planning using learned motion costs},
  author={Yang, Bowen and Wellhausen, Lorenz and Miki, Takahiro and Liu, Ming and Hutter, Marco},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={9283--9289},
  year={2021},
  organization={IEEE}
}
```
