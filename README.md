# Champ in Moon

The [CHAMP](https://github.com/chvmp/champ) quadruped robot walking and mapping a real lunar
terrain in Gazebo Classic (ROS 2 Humble).

The terrain is a real NASA LRO heightmap of the **Lunar Tranquillitatis Pit**
(from Gazebo's Fuel asset library, OpenRobotics), dropped into a custom world alongside CHAMP's
existing gait controller and LiDAR-based SLAM stack.

![CHAMP walking on the lunar terrain, RViz showing live LiDAR scan](media/demo.gif)

![Screenshot of Gazebo and RViz side by side](media/screenshot.png)

CHAMP also carries a front-facing RGB-D camera, so you can see the terrain from its own point of
view while it walks:

![First-person view from CHAMP's onboard camera walking across the lunar terrain](media/champ_eyes.gif)

## What's here

Two packages: `champ_moon_bringup`, which layers the lunar world and bringup config on top of the
upstream CHAMP packages (not included here, see setup below), and `champ_nav2_controller`, a
custom Nav2 controller plugin purpose-built for CHAMP's gait.

**`champ_moon_bringup`:**
- `worlds/moon.world` - the lunar terrain + CHAMP robot world
- `models/lunar_tranquillitatis_pit/` - the downloaded heightmap terrain model
- `launch/moon_gazebo.launch.py` - custom launch file; adds a `world_init_z` spawn-height
  override that CHAMP's own launch wrapper doesn't expose
- `launch/moon_navigate.launch.py` - launches Nav2 (`nav2_bringup`) + RViz on the saved map
- `config/slam.yaml` - CHAMP's slam_toolbox config, with `transform_timeout` raised from 0.2s to
  1.5s for stability on this terrain
- `config/nav2_params.yaml` - Nav2 stack config, using the custom controller below for path
  following
- `rviz/nav_view.rviz` - RViz config for viewing the robot, LiDAR scan, map, breadcrumb trail, and
  the onboard camera feed
- `maps/moon_map.pgm` / `.yaml` - a saved SLAM map from manual teleop exploration (~27.6m x 25m)
- `scripts/breadcrumb_node.py` - records the robot's actual traveled path (from `/amcl_pose`) as a
  trail of waypoints, and exposes a `/return_to_start` service that reverses the trail and feeds it
  to Nav2's `FollowWaypoints` action - a "retrace exactly where I've been" behavior, distinct from
  just re-planning a fresh route back

**`champ_nav2_controller`:** a custom `nav2_core::Controller` plugin, written because Nav2's stock
`RegulatedPurePursuitController` assumes wheeled-robot-like velocity tracking that CHAMP's legged
gait can't deliver - a direct `cmd_vel`-vs-`odom` measurement showed the gait only achieves ~41% of
a commanded angular velocity when rotating in place. This controller:
- follows the path with pure-pursuit + a rotate-to-heading gate, using speed limits that reflect
  what the gait can actually do
- continuously compares what it *commanded* last cycle against what Nav2 reports the robot
  *actually* achieved (from odometry) every control cycle, and maintains a live self-correcting
  scale factor instead of trusting a fixed assumption

On the same test goal, the stock (tuned) controller racked up 10 recovery behaviors and ultimately
failed to reach the goal; this controller reached it in 1 recovery and ~146s.

## Setup

Requires ROS 2 Humble and Gazebo Classic 11.

```bash
mkdir -p champ_moon_ws/src && cd champ_moon_ws/src
git clone https://github.com/biswajeetpandaisreal-arch/Champ_in_moon.git
git clone https://github.com/chvmp/champ.git
git clone https://github.com/chvmp/champ_teleop.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

(the cloned repo contains both `champ_moon_bringup` and `champ_nav2_controller` directly under
`src/` - colcon discovers packages recursively, so no special layout is needed)

## Running

Launch the robot in the lunar world:

```bash
ros2 launch champ_moon_bringup moon_gazebo.launch.py rviz:=true
```

Drive it around with teleop, and run `slam_toolbox` to map the terrain (see `config/slam.yaml`).
The included `maps/moon_map.*` is a map already produced this way.

To navigate autonomously on the saved map instead:

```bash
ros2 launch champ_moon_bringup moon_navigate.launch.py
```

then set a goal in RViz with the "Nav2 Goal" tool. Add `ros2 run champ_moon_bringup
breadcrumb_node.py` in another terminal to record the trail, and `ros2 service call
/return_to_start std_srvs/srv/Trigger` to retrace it back.

**One-click version:** open the repo in VS Code (`code .`) and run the **"Launch Full Stack"**
task (`Ctrl+Shift+P` -> "Tasks: Run Task") - launches Gazebo, Nav2+RViz, and the breadcrumb node
together in separate terminal panels with the right startup delays baked in. "0. Kill All Champ
Processes" cleanly tears everything down again.

## Notes / known issues

- **Gravity is Earth-normal (-9.8), not lunar (-1.622).** Real lunar gravity was tried first, but
  CHAMP's gait controller is tuned for Earth ground-reaction forces and hopped/bounced
  uncontrollably under it, which also broke SLAM scan-matching. Terrain and lighting are lunar;
  physics is not (yet).
- Spawn point `(-42.4, -42.4)` was chosen after verifying it's flat via direct heightmap pixel
  analysis, well clear of the crater near the terrain's center.
- There's a harmless `gazebo_ros2_control` "Parameter 'hold_joints' has already been declared"
  warning that causes a slight idle standing wobble; walking is unaffected.
- SLAM mapping so far was done manually via teleop, not autonomous exploration.

## Roadmap

- [x] Nav2 on the saved map for autonomous navigation - with a custom controller purpose-built for
  CHAMP's gait, see `champ_nav2_controller` above
- [x] VS Code task automation to launch Gazebo/Nav2/breadcrumb node together - see `.vscode/tasks.json`
- [x] Breadcrumb trail recording + retrace-to-start, via `scripts/breadcrumb_node.py`
- [ ] Autonomous frontier exploration for mapping
- [ ] Compare global planner algorithms (A*, Dijkstra, and a custom RRT plugin)

## Credits

- [chvmp/champ](https://github.com/chvmp/champ) and
  [chvmp/champ_teleop](https://github.com/chvmp/champ_teleop) - the underlying quadruped stack
- Lunar Tranquillitatis Pit terrain - NASA LRO data via Gazebo Fuel, OpenRobotics
