# How tinyslam_ros is translated from the tinySLAM source

The tinySLAM repository (`openslam_tinyslam`) has almost no documentation: its `readme` only lists the build commands. So this package was translated from two parts of the code itself:

- **The core functions** in `CoreSLAM.c` and `CoreSLAM_random.c`. These are kept and called unchanged.
- **The driver loop** in `CoreSLAM_state.c`: the functions `ts_iterative_map_building` and `ts_build_scan`. This is the reference for how a robot is meant to feed data in. Because it is tied to the original robot's hardware, it was rewritten in C++ as the ROS node `src/tinyslam_node.cpp`.

The algorithm itself is described in the authors' paper: Steux & El Hamzaoui, *"tinySLAM: a SLAM algorithm in less than 200 lines C-language program"* (ICARCV 2010).

## What was kept unchanged

| Original function | Role | Called from the node |
|---|---|---|
| `ts_map_init` | Fill the map with "unknown" | constructor |
| `ts_monte_carlo_search` | Random-search scan matching | `on_scan` |
| `ts_distance_scan_to_map` | Score a pose (used inside the search) | via the search |
| `ts_map_update` / `ts_map_laser_ray` | Draw the V-shaped wall valleys | `on_scan` |
| `ts_random_*` | Gaussian sampling for the search | via the search |
| `ts_save_map_pgm` | PGM export | `/tinyslam/save_pgm` service |

Parameter defaults also come straight from the original's test program, `test_loop_closing.c`:

| Parameter | Value |
|---|---|
| σ_xy | 100 mm |
| σ_θ | 20° |
| Hole width | 600 mm |
| Map quality | 50 |
| Search iterations | 1000 |
| Interpolated rays per beam (mapping) | 3 |

## Changes to the core files (`core/`)

| Change | Why |
|---|---|
| Map size and scale macros can be overridden; set to 2048 px at 25 mm | The original's 20.48 m map can't hold the 38.5 m office. |
| Random-number state switched to 32-bit types | The original assumes 32-bit `long`; on 64-bit Linux it produced garbage. |
| XOR `SWAP` macro replaced | It was undefined behaviour in C. |
| `extern "C"` added to `CoreSLAM.h` | Lets C++ call the C code. |

## How the driver loop was translated

The original `ts_iterative_map_building` runs three steps per scan, and the node keeps all three in order:

1. predict the pose from odometry,
2. run the Monte Carlo search,
3. update the map.

The inputs to each step had to change for this robot.

### 1. Odometry

The original reads raw wheel encoder counts (`q1`, `q2`) and integrates them using:

- the wheel radius `r`,
- half the wheel spacing `R`,
- the encoder counts per wheel turn `inc`.

This robot has no encoder counts available: the diff drive controller has already integrated them into the `odom → base_footprint` transform. So the node reads that transform at the scan's timestamp and applies the change since the last scan.

The node also corrects that change for the robot's wheel-axle offset (the wheels sit 0.221 m behind `base_footprint`). The original never needed this.

### 2. Laser position

The original uses one number, `laser_params.offset`: how far the lidar is in front of the rotation centre, along the heading. It adds that offset before the search and subtracts it afterwards.

The node reads the full planar mount (x, y and yaw) from TF instead, so it also handles a lidar mounted off-centre or rotated. The idea is the same in both: tinySLAM tracks the sensor's pose, not the robot's.

### 3. Units and frames

| | tinySLAM core | ROS |
|---|---|---|
| Distance | millimetres | metres |
| Angle | degrees | radians |
| Origin (0, 0) | corner of the map | where the robot started |

The node's `to_ts` and `from_ts` functions convert between the two, and put ROS's (0, 0) at the centre of the tinySLAM map.

### 4. Scan conversion

The original `ts_build_scan` converts the scan in `ts_sensor_data_t`, a fixed array of integer distances in millimetres. The node's `build_scan` reads a ROS `LaserScan` instead.

| Original `ts_build_scan` | Node's `build_scan` | Reason |
|---|---|---|
| Beam angles spread evenly from `angle_min` to `angle_max` (integer degrees) | `angle_min + i × angle_increment` from the message | Uses the sensor's real geometry. |
| Distance `0` means no return; a free-space point is placed at `distance_no_detection` | `inf` or ≥ `range_max` means no return; the free-space point is placed at `range_max` | That's how ROS and Gazebo mark missing returns. |
| Keeps hits farther than `hole_width / 2` | Same rule, plus readings below `range_min` are dropped | Original rule kept. |
| `detection_margin` skips beams at both ends of the scan | Dropped | The original's 240° Hokuyo saw the robot's own body at the ends of its scan. This robot's 360° lidar sits above the chassis. |
| Corrects for motion during the scan, using the previous step's speeds (`v`, `psidot`) | Dropped | A real Hokuyo takes about 100 ms per sweep, so the robot moves during a scan. Gazebo's `gpu_lidar` captures the whole scan at once. |
| 3 rays per beam for mapping, angles spread so the first and last land exactly on the scan limits | 3 rays per beam, centred on each beam | See below. |

#### The 3 rays per beam

The first version of the translation got this wrong. It spread each beam's extra rays to one side, from the beam angle forward. That rotated the map relative to the matching scan, and the heading drifted by 17° over a 40 m loop.

The original spreads the extra rays across the whole scan instead, which averages out to no bias. Centring them on each beam has the same effect for a 360° lidar, and brought the heading error over the loop under 0.6°. So the drift was a translation error, not a flaw in tinySLAM.

### 5. Things the original didn't need

- **Throttling:** the lidar publishes at 62 Hz, so the node processes at most one scan every 0.1 s (`min_scan_interval`). The original processed every scan in a recorded file.
- **ROS outputs:**
  - `/map` as an `OccupancyGrid`
  - the `map → odom` transform
  - `/tinyslam/pose` and `/tinyslam/path`
- **First scan:** the node skips the search on the first scan, since there is nothing to match against yet. The original runs it anyway, which does nothing on an empty map.

## What was left out

- **Loop closing** (`CoreSLAM_loop_closing.c`). The original runs a recorded dataset forwards and backwards, then blends the two trajectories. That only works offline on a full recording, not on live data.
- **`ts_state_t` and `ts_sensor_data_t`.** These are built around wheel encoder counts and fixed-size integer arrays. The node keeps the same state (pose, map, random generator) in its own member variables instead.
