# tinyslam_ros

tinySLAM adapted to run as a ROS 2 Jazzy package on the ICTE4001 differential-drive robot in Gazebo.

In a test in the simulator, the robot drove a 40 m loop through the office with five turns. tinySLAM's worst position error against Gazebo's ground truth was **0.09 m**. Raw wheel odometry reached **0.43 m**.

## Running it

Use two terminals. In each one, go to `~/Uni/AutoBots_Case_study_ws` and run `source install/setup.bash` first.

1. Start the simulator:

   ```bash
   ros2 launch diff_drive_robot_gz_sim ICTE4001_MRobot.launch.py world:=office_world
   ```

2. Start tinySLAM:

   ```bash
   ros2 launch tinyslam_ros tinyslam.launch.py
   ```

The RViz window from step 1 already displays `/map`, so the map appears as it is built.

To drive the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

To save the map as an image (`tinyslam_map.pgm`, written to the folder you launched tinySLAM from):

```bash
ros2 service call /tinyslam/save_pgm std_srvs/srv/Trigger
```

## Running on a TurtleBot 4

`config/tinyslam_turtlebot4.yaml` and `launch/tinyslam_turtlebot4.launch.py` are set up for a TurtleBot 4 or 4 Lite (Create 3 base, RPLIDAR A1):

```bash
ros2 launch tinyslam_ros tinyslam_turtlebot4.launch.py                      # no namespace
ros2 launch tinyslam_ros tinyslam_turtlebot4.launch.py namespace:=robot1    # namespaced robot
ros2 launch tinyslam_ros tinyslam_turtlebot4.launch.py use_sim_time:=true   # turtlebot4_simulator
```

Differences from the Gazebo settings:

| Setting | Value | Why |
|---|---|---|
| `base_frame` | `base_link` | The Create 3 publishes `odom → base_link`. |
| `left_wheel_frame`, `right_wheel_frame` | empty | The Create 3's `base_link` is already at the wheel axle, so no correction is needed. |
| `use_sim_time` | `false` | Real robot. |
| `max_range_m` | 8.0 | The RPLIDAR A1 is noisy near its 12 m limit. |
| `min_scan_interval` | 0.0 | The RPLIDAR only publishes at about 7–10 Hz, so every scan is used. |
| `map_publish_period` | 2.0 s | Less WiFi traffic. |

With a namespace, the node subscribes to `<namespace>/scan` and `<namespace>/tf`, and publishes `<namespace>/map`.

Before the first run, check the frame names on your robot with `ros2 run tf2_tools view_frames`.

Notes:

- Don't run it alongside the TurtleBot 4's own `slam.launch.py` or `localization.launch.py`. Those also publish `map → odom`.
- Turn slowly while mapping, at about 0.3–0.5 rad/s. The RPLIDAR spins, so the robot moves during each scan, and the node doesn't correct for that.

## How it works

- The tinySLAM core (`CoreSLAM.c`, `CoreSLAM_random.c`, `CoreSLAM_ext.c`, `CoreSLAM.h`) is copied into `core/`. The original clone in `tinyslam/` is unchanged except for an empty `COLCON_IGNORE` file, which stops `colcon build` from building the clone as a separate package.
- A ROS node (`src/tinyslam_node.cpp`) feeds the core the `/scan` topic and the odometry from TF.
- On each scan, the node:
  1. predicts the new pose from the odometry,
  2. matches the scan against the map with tinySLAM's Monte Carlo search,
  3. adds the scan to the map at the corrected pose.
- It publishes:

  | Output | Type | Contents |
  |---|---|---|
  | `/map` | `nav_msgs/OccupancyGrid` | the map, 0.05 m per cell |
  | `map → odom` | TF | correction that places the robot at the SLAM pose |
  | `/tinyslam/pose` | `geometry_msgs/PoseStamped` | current SLAM pose |
  | `/tinyslam/path` | `nav_msgs/Path` | trajectory so far |

- Settings are in `config/tinyslam.yaml`.
- For how each part maps to the original tinySLAM code, see [docs/translation_from_tinyslam.md](docs/translation_from_tinyslam.md).

## Fixes needed to make it accurate

| Problem | Effect | Fix |
|---|---|---|
| Map too small | The original map is 2048 px at 10 mm per pixel, only 20.48 m across, and the office is 38.5 m long. | The map is now 2048 px at 2.5 cm per pixel, 51.2 m across. The size can be changed in `CMakeLists.txt`. |
| Broken random-number generator | It assumed 32-bit integers. On a 64-bit machine, 99.9% of its samples were garbage, which would have made the scan matching useless. | Changed the copy to fixed-width 32-bit types. Tested: mean 0, standard deviation 1.00, no outliers. |
| Undefined swap macro | The line-drawing code swapped values with an XOR trick that is undefined behaviour in C, so an optimising compiler could miscompile it. | Replaced it with a swap through a temporary variable. |
| Heading drift | The extra rays used for mapping were spread to one side of each beam. That rotated the map slightly against the scans, so the heading drifted by 17° over the loop. | Centred the extra rays on each beam. The heading error is now under 0.6° over the loop. |

## Known issue: wheel position in the robot model

The robot's wheels sit 0.221 m behind `base_footprint`, but the diff drive controller assumes they are at it. So whenever the robot turns, its odometry reports the wrong point:

| Turn | Odometry position error |
|---|---|
| 90° | 0.31 m (0.221 × √2) |
| 180° | 0.43 m (2 × 0.221) |

The SLAM node corrects for this itself. It reads the wheel positions from TF using the `left_wheel_frame` and `right_wheel_frame` settings. Anything else that uses the odometry still sees the error, including Nav2. The proper fix is in the URDF: put the wheel axle at `base_footprint`'s origin.

## Test scripts

The scripts used to test the package are in `tools/`. Run them with ROS sourced, using `python3 <script>`.

| Script | Purpose |
|---|---|
| `headless_sim.launch.py` | Starts the simulator with no windows: Gazebo server only, no RViz, office world. |
| `drive_and_score.py` | Drives the robot through a loop of waypoints and, at each one, compares the tinySLAM and odometry poses with Gazebo's ground truth. |
| `rot_test.py` | Checks heading drift with the robot standing still and then rotating in place. |
| `grab_map.py` | Saves the current `/map` as a PNG: `python3 grab_map.py map.png`. |

To repeat the accuracy test, start `headless_sim.launch.py` with `ros2 launch`, start tinySLAM, then run `drive_and_score.py`.
