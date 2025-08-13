# ROS1 Testing — TortoiseBot Waypoints (Task 1)

This is my **ROS 1** solution for **Checkpoint 23 - Testing (Task 1)**.  
I implement a Waypoints **Action Server** for the TortoiseBot and provide **node-level tests** (with `rostest`) that verify **final position** and **final yaw**.

If you're grading or trying this locally, follow the quickstart below to launch the simulation, start my action server, and run both **passing** and **failing** test configurations.

---

## Repository

**Repo:** https://github.com/mathrosas/ros1_testing

Main package in this repo:

```
tortoisebot_waypoints/
├── scripts/
│   └── tortoisebot_action_server.py   # Action server (ROS1, Python)
└── test/
    ├── test_waypoints.py              # rostest test node (Python 3)
    └── waypoints_test.test            # rostest launch with parameters
```

I use the `WaypointAction.action` interface from `tortoisebot_msgs`.

> Note: While the checkpoint mentions *Python 2*, ROS **Noetic** uses **Python 3**. My test node uses f-strings and runs with `python3`. Make sure scripts are executable and have a Python 3 shebang.

---

## Prerequisites

- ROS **Noetic**.
- A catkin workspace with the TortoiseBot simulation available, typically at `~/simulation_ws` with package `tortoisebot_gazebo`.
- Gazebo working.
- If your environment doesn't already include the `tortoisebot_msgs` action definitions, see **“Action interface”** below.

---

## Clone & Build

```bash
# Clone my repo into your simulation workspace
cd ~/simulation_ws/src
git clone https://github.com/mathrosas/ros1_testing.git

# Build and source
cd ~/simulation_ws
catkin_make
source devel/setup.bash
```

If your `tortoisebot_waypoints/scripts/tortoisebot_action_server.py` has `#!/usr/bin/env python`, switch it to Python 3 and ensure it's executable:

```bash
sed -i '1s|python$|python3|' ~/simulation_ws/src/ros1_testing/tortoisebot_waypoints/scripts/tortoisebot_action_server.py
chmod +x ~/simulation_ws/src/ros1_testing/tortoisebot_waypoints/scripts/tortoisebot_action_server.py
```

---

## Action interface (if you don't already have it)

If your system doesn't have `tortoisebot_msgs` with `WaypointAction.action`, create it quickly:

```bash
cd ~/simulation_ws/src
catkin_create_pkg tortoisebot_msgs actionlib_msgs geometry_msgs message_generation
mkdir -p tortoisebot_msgs/action
```

Create `tortoisebot_msgs/action/WaypointAction.action`:

```action
# Goal
geometry_msgs/Point position
---
# Result
bool success
---
# Feedback
geometry_msgs/Point position
string state
```

Update **CMakeLists.txt** (key lines):

```cmake
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
)

add_action_files(FILES WaypointAction.action)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime)
```

Update **package.xml** (key deps):

```xml
<build_depend>message_generation</build_depend>
<build_depend>actionlib_msgs</build_depend>
<build_depend>geometry_msgs</build_depend>
<exec_depend>message_runtime</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>

<!-- for tests -->
<test_depend>rostest</test_depend>
```

Rebuild:

```bash
cd ~/simulation_ws
catkin_make
source devel/setup.bash
```

---

## Launch the Simulation (ROS 1)

**Terminal 1:**

```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch
```

If the world fails to load the first time, close and relaunch. If `gzserver` lingers:

```bash
ps faux | grep gz
kill -9 <pid>
```

---

## Start my Waypoints Action Server

**Terminal 2:**

```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_server.py
```

You should see logs like **“Action server started”** and state updates (“fix yaw” / “go to point”).  
The server advertises the action **`/tortoisebot_as`**, publishes to **`/cmd_vel`**, and subscribes to **`/odom`**.

---

## Run the Tests (rostest)

My tests live in `tortoisebot_waypoints/test/`. They:

- Send a goal to `/tortoisebot_as`.
- Wait for the result.
- Read `/odom` to check:
  - **Position** error ≤ `pos_tol`
  - **Yaw** error ≤ `yaw_tol_deg` (degrees; converted to radians)

**Terminal 3:**

```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
```

Using `--reuse-master` makes `rostest` use the already running master from the simulation.

### Parameters I expose in `waypoints_test.test`

```xml
<launch>
  <param name="goal_x" value="-0.5"/>
  <param name="goal_y" value="0.0"/>
  <param name="expected_x" value="-0.5"/>
  <param name="expected_y" value="0.0"/>
  <param name="pos_tol" value="0.25"/>
  <param name="yaw_tol_deg" value="20"/>
  <test test-name="test_waypoints"
        pkg="tortoisebot_waypoints"
        type="test_waypoints.py"
        time-limit="300.0"/>
</launch>
```

You can tweak these to deliberately **pass** or **fail** as needed for grading.

---

## Grading: Passing vs Failing

### ✅ Passing run

Use the default parameters above (goal and expected match; reasonable tolerances).

Expected summary:

```
[ROSTEST]-----------------------------------------------------------------------
SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```

### ❌ Failing run (two quick ways)

Open `tortoisebot_waypoints/test/waypoints_test.test` and choose one:

**Option A — Position failure**

```xml
<param name="expected_x" value="2.0"/>
<param name="expected_y" value="2.0"/>
<param name="pos_tol"   value="0.05"/>
```

**Option B — Yaw failure**

```xml
<param name="yaw_tol_deg" value="0"/>
```

Re-run:

```bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
```

Expected (as per rubric):

```
[ROSTEST]-----------------------------------------------------------------------
SUMMARY
 * RESULT: FAIL
 * TESTS: 1
 * ERRORS: 1
 * FAILURES: 0
```

*(In `rostest`, some mismatches show up as **errors** rather than JUnit “failures” depending on timing and assertions; either is acceptable for the failing run in the rubric.)*

---

## Git (required by the checkpoint)

I keep my work on branch **`main`** and commit meaningful changes as I go:

```bash
cd ~/simulation_ws/src/ros1_testing
git add .
git commit -m "feat: add ROS1 waypoint action server and rostest tests"
```

Push if you've set a remote:

```bash
git push -u origin main
```

---

## Troubleshooting

- **No `/odom`?** Ensure the Gazebo sim is running and the robot is spawned.
- **Action server not found?** Confirm Terminal 2 is running `tortoisebot_action_server.py` and that the action name is `/tortoisebot_as`.
- **Robot doesn't move?** Check that `/cmd_vel` is published (`rostopic echo /cmd_vel`) and nothing else is overriding it.
- **Interpreter mismatch?** Use `#!/usr/bin/env python3` in Python scripts and make them executable.
- **Gazebo stuck?** Kill lingering `gzserver`/`gzclient` (see above).

---

## License

Unless otherwise stated, this repository's contents are provided as-is for educational purposes.
