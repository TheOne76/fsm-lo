<h1 align='center' style="text-align:center; font-weight:bold; font-size:2.0em;letter-spacing:2.0px;"> FSM: Correspondenceless scan-matching of panoramic 2D range scans </h1>

<div align="center">

[![ieeexplore.ieee.org](https://img.shields.io/badge/IEEE/RSJ_IROS_2022_paper-00629B)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9981228)
[![youtube.com](https://img.shields.io/badge/1'_presentation-YouTube-FF0000)](https://www.youtube.com/watch?v=hB4qsHCEXGI)
[![github.com](https://img.shields.io/badge/pdf_presentation-333333)](https://mozilla.github.io/pdf.js/web/viewer.html?file=https://raw.githubusercontent.com/phd-li9i/fsm_presentation_iros22/master/main.pdf)

</div>

<h3 align="center">
    <a href="#nodes">I/O</a>
    <span> · </span>
    <a href="#installation">install</a>
    <span> · </span>
    <a href="#run">run</a>
</h3>

`fsm_lo` is a ROS 2 package written in C++ that provides LIDAR odometry from measurements of a single panoramic 2D LIDAR sensor, that is: a sensor whose field of view is 360 degrees. `fsm_lo` is the ROS wrapper of [`fsm`](https://github.com/li9i/fsm).

<p align="center">
  <img src="https://i.imgur.com/hUsBImy.png">
</p>

Lidar odometry is achieved via scan-matching _but without establishing correspondences_ between elements of the input scans or their properties, but by leveraging the range signal's periodicity. Hence FSM may exploit properties of the Discrete Fourier Transform. These two pillars support the robustness of FSM's pose error against (a) sensor noise and (b) distance between consecutive poses, as exhibited in the figure which summarises key experiments below.

## Why use FSM

![Experimental results at a glance](https://i.imgur.com/GvFlHgF.png)

Table of Contents
=================

* [Requirements](#requirements)
* [Installation](#installation)
* [Run](#run)
  * [Launch](#launch)
  * [Call](#call)
* [Nodes](#nodes)
  * [`fsm_lo`](#fsm_lo)
    * [Subscribed topics](#subscribed-topics)
    * [Published topics](#published-topics)
    * [Services offered](#services-offered)
    * [Parameters](#parameters)
    * [Transforms published](#transforms-published)
* [Upgrading from the ROS 1 version](#upgrading-from-the-ros-1-version)
* [Motivation and Under the hood](#motivation-and-under-the-hood)
  * [1 min summary video](#1-min-summary-video)
  * [IROS 2022 paper](#iros-2022-paper)

## Requirements

ROS 2 Lyrical, a compiler with C++23 support, and FFTW3, CGAL and Eigen3. Neither FFTW3 nor CGAL has an ament wrapper, so on Debian and Ubuntu:

```bash
sudo apt install libfftw3-dev libcgal-dev libeigen3-dev
```

## Installation

Via Docker, which brings its own ROS and dependencies:

```bash
git clone git@github.com:li9i/fsm-lo.git
cd fsm-lo
docker compose -f docker/docker-compose.yml build
```

Or into an existing workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:li9i/fsm-lo.git
cd ~/ros2_ws
colcon build --packages-select fsm_lo
```

## Run

### Launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fsm_lo fsm_lo.launch.xml
```

or, with Docker:

```bash
docker compose -f docker/docker-compose.yml up
```

### Call

Launching `fsm_lo` puts it into stand-by; it processes nothing until told to. To start:

```bash
ros2 service call /fsm_lo/start std_srvs/srv/Trigger
```

## Nodes

### `fsm_lo`

The executable is `fsm_lo_interface_node`. It runs on a multi-threaded executor, so a service call cannot be held up behind a scan being matched.

#### Subscribed topics

| Topic                | Type                                          | Utility                                                                                |
| -------------------- | --------------------------------------------- | -------------------------------------------------------------------------------------- |
| `scan_topic`         | `sensor_msgs/msg/LaserScan`                   | 2d panoramic scans are published here                                                  |
| `initial_pose_topic` | `geometry_msgs/msg/PoseWithCovarianceStamped` | optional---for setting the very first pose estimate to something other than the origin |

#### Published topics

| Topic                 | Type                            | Utility                                                                       |
| --------------------- | ------------------------------- | ----------------------------------------------------------------------------- |
| `pose_estimate_topic` | `geometry_msgs/msg/PoseStamped` | the current pose estimate relative to the global frame is published here      |
| `path_estimate_topic` | `nav_msgs/msg/Path`             | the total estimated trajectory relative to the global frame is published here |
| `lo_topic`            | `nav_msgs/msg/Odometry`         | the odometry is published here                                                |

Every message is stamped with the timestamp of the scan that produced it, not with the clock at the moment it was produced. Replaying a recording therefore gives the same output every time.

#### Services offered

All four take `std_srvs/srv/Trigger` and return a success flag and a message.

| Service                             | Utility                                                                                                                                                          |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `fsm_lo/clear_estimated_trajectory` | clears the vector of estimated poses and returns the accumulated pose to the origin                                                                              |
| `fsm_lo/set_initial_pose`           | node waits for one message on `initial_pose_topic`, sets fsm's initial pose from it, and returns. Waits for as long as it takes      |
| `fsm_lo/start`                      | commences node functionality                                                                                                                                     |
| `fsm_lo/stop`                       | halts node functionality (node remains alive)                                                                                                                    |

#### Parameters

Found in `config/params.yaml`:

| IO Topics             | Description                                                         |
| --------------------- | ------------------------------------------------------------------- |
| `scan_topic`          | 2d panoramic scans are published here                               |
| `initial_pose_topic`  | (optional) the topic where an initial pose estimate may be provided |
| `pose_estimate_topic` | `fsm_lo`'s pose estimates are published here                        |
| `path_estimate_topic` | `fsm_lo`'s total trajectory estimate is published here              |
| `lo_topic`            | `fsm_lo`'s odometry estimate is published here                      |

| Frame ids         | Description                                                    |
| ----------------- | -------------------------------------------------------------- |
| `global_frame_id` | the global frame id (e.g. `map`)                               |
| `base_frame_id`   | the lidar sensor's reference frame id (e.g. `base_laser_link`) |
| `lo_frame_id`     | the (lidar) odometry's frame id                                |

Frame ids carry no leading slash. tf2 rejects them.

| FSM-specific parameters  | Description                                                                                                       | Default value |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------- |:-------------:|
| `size_scan`              | the size of scans that are matched (execution time is proportional to scan size, hence subsampling may be needed) | 360           |
| `min_magnification_size` | base angular oversampling                                                                                         | 0             |
| `max_magnification_size` | maximum angular oversampling                                                                                      | 3             |
| `num_iterations`         | Greater sensor velocity requires higher values                                                                    | 2             |
| `xy_bound`               | Axis-wise radius for randomly generating a new initial position estimate in case of recovery                      | 0.2           |
| `t_bound`                | Angular-wise radius for randomly generating a new initial orientation estimate in case of recovery                | π/4           |
| `max_counter`            | Lower values decrease execution time                                                                              | 200           |
| `max_recoveries`         | Lower values decrease execution time                                                                              | 10            |
| `rng_seed`               | 0 draws the recovery search from hardware entropy, as always. Any other value pins it so a run can be reproduced  | 0             |
| `ray_search`             | `angular` or `windowed`. How each ray is matched to the wall it meets; see below                                   | `angular`     |

`ray_search` picks between two ways of finding the wall each ray of a scan
meets.

`angular` offers each wall only to the rays that can reach it. It returns the
nearest wall in front of every ray whatever shape the room is, and its
execution time rises in step with `size_scan`.

`windowed` narrows the search for each ray to the neighbourhood of the wall the
previous ray met. It is what this algorithm shipped with, and it is kept so
that a run can be compared against results published before `angular` existed.
Its execution time rises with the square of `size_scan`, and where a room turns
back on itself it can return a wall standing behind the nearest one.

Measured over 13908 matches drawn from a recorded dataset at `size_scan: 360`,
`angular` completes a match in 17.9 ms against `windowed`'s 25.4 ms, median.
The two disagree on 0.67% of matches, by a median of 3.5 mm. At `size_scan:
1440` the ray casting alone is 3.2 times faster, and at 5760, 12.4 times.

| Node parameters        | Description                                                                                                                | Default value |
| ---------------------- | ---------------------------------------------------------------------------------------------------------------------------- |:-------------:|
| `scan_qos_reliability` | `reliable` or `best_effort`. Sensor drivers commonly publish `best_effort`, and a mismatched subscription receives nothing | `reliable`    |
| `scan_qos_depth`       | subscription queue depth                                                                                                   | 1             |

A parameter outside its permitted range is refused at start-up with an explanation, rather than tripping an assertion that release builds compile away.

#### Transforms published

```
lo_frame_id <- base_frame_id
```

in other words `fsm_lo` publishes the transform from `base_laser_link` (or equivalent) to the equivalent of `/odom` (in this case `lo_frame_id`).

#### Diagnostics

The matching core reports in plain strings and does not write to a terminal itself. It hands each line to whatever destination the host installed, through `fsm_lo::setDiagnosticSink`, and drops the line where nothing was installed. The node installs a destination that forwards to `RCLCPP_INFO`, so anything the core says arrives in the ordinary ROS log.

In an ordinary build there is almost nothing to say. The stage timings, which are the bulk of it, are compiled out unless the core is built with `FSM_LO_TRACE`:

```sh
colcon build --cmake-args "-DCMAKE_CXX_FLAGS=-DFSM_LO_TRACE"
```

That build is for finding out where the time goes and is not the one to run a robot with: it reads the clock around every stage of every iteration.

## Upgrading from the ROS 1 version

The matcher itself is unchanged, and this is measured rather than asserted: driven over identical synthetic scans, the two versions agree on every published quantity to better than 4e-15, six orders of magnitude inside the 1e-9 bar the comparison demands. Both sides are checked against themselves first, three runs each, so that the comparison rests on something reproducible.

What did change:

- The four services return `std_srvs/srv/Trigger` instead of taking and returning nothing, so callers now get a success flag and a message. The names are unchanged.
- Output is stamped from the incoming scan rather than from the clock at the moment of processing. The reported twist follows from the interval between scan stamps, so it no longer varies with machine load.
- `std_msgs/Header` has no sequence number in ROS 2, so the published messages no longer carry one.
- The scan subscription's reliability and depth are parameters. The default mirrors ROS 1, but a `best_effort` sensor driver now needs `scan_qos_reliability` set to match, or no scans arrive at all.
- Frame id defaults lost their leading slash.
- A scan carrying fewer ranges than `size_scan` is refused with a warning instead of being read past its end.

Eight defects were corrected before the port, and the numbers above are measured against a ROS 1 build carrying those corrections rather than against the published one. Output from this version therefore differs from the published ROS 1 version by more than the figures above. The corrections:

- The rotation matrix that accumulates the trajectory mixed single and double precision, so it was not a rotation. Its determinant was 1.7e-8 away from unity, and since it is composed once per scan the error grew along the whole path.
- The initial pose supplied through `set_initial_pose` was built in single precision, degrading it to about seven significant figures.
- The convergence criterion of the translation stage took a single precision square root of a double.
- Gap filling read one element before the start of an empty list whenever a scan contained no invalid returns at all, which crashed the node. Real sensors nearly always return at least one bad ray, which is why this survived so long.
- With `pose_estimate_topic` unset, the fallback was written to the wrong field: the pose publisher was given an empty topic name and the node threw during construction, so it could not start at all without that parameter.
- Frame id fallbacks carried a leading slash, which tf2 rejects.
- Two assertions checked that an unsigned value was at least zero.
- Output was stamped from the wall clock rather than from the scan.

## Motivation and Under the hood

### 1 min summary video

[![IMAGE ALT TEXT](http://img.youtube.com/vi/hB4qsHCEXGI/0.jpg)](http://www.youtube.com/watch?v=hB4qsHCEXGI "1 min summary video")

### IROS 2022 paper

```bibtex
@INPROCEEDINGS{9981228,
  author={Filotheou, Alexandros and Sergiadis, Georgios D. and Dimitriou, Antonis G.},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={FSM: Correspondenceless scan-matching of panoramic 2D range scans},
  year={2022},
  pages={6968-6975},
  doi={10.1109/IROS47612.2022.9981228}}
```
