# Copilot instructions — dragon package

Purpose: short, actionable guidance so AI coding agents can be productive immediately in this package.

## Big picture (what to know first)
- This is a ROS (ROS1 / catkin) package named `dragon` that provides robot models, controllers, navigation and sensor integration for the DRAGON aerial robot. Key C++ targets live in `src/` and headers in `include/dragon/`.
- Runtime architecture: navigation + controllers publish commands for the onboard middleware `spinal` (topics like `four_axes/command` and `desire_coordinate`). Hardware-specific code lives in `src/sensor` and `config/quad/Servo.yaml`.
- Control plugins (pluginlib) are declared in `plugins/*.xml` (e.g., `plugins/flight_control_plugins.xml`) and loaded via pluginlib using class names like `aerial_robot_control/dragon_full_vectoring`.
- Simulation support exists (MuJoCo and Gazebo helpers). MuJoCo conversion is triggered by CMake macro using `config/mujoco_model.yaml`.

## How to build & run (explicit commands)
- Build (from workspace root):
  - catkin_make: `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo` (or `catkin build` if you use catkin_tools).
  - After build: `source devel/setup.bash`.
- Bringup (examples):
  - Real machine: `roslaunch dragon bringup.launch`
  - Simulation: `roslaunch dragon bringup.launch real_machine:=false simulation:=true headless:=false`
- Basic demos:
  - Transformation demos: `rosrun dragon transformation_demo.py _mode:=0` (see `scripts/transformation_demo.py` for available modes and parameters).
- Tests:
  - Unit/integration via rostest: `rostest dragon dragon_control.test` and `rostest dragon dragon_jacobian.test`
  - You can also run `catkin_make run_tests` / `catkin_test_results` depending on your build tool.

## Project-specific conventions & patterns
- C++ standard: **C++17** (see `CMakeLists.txt` add_compile_options). Expect RelWithDebInfo build. Keep `-O3 -g -DNDEBUG` behaviour in mind when troubleshooting optimizations.
- Controller pattern: controllers derive from `PoseLinearController` (see `include/dragon/control/*_control.h`) and are exposed to runtime via `plugins/*.xml`. If adding a new controller, add a class + register in the plugin XML and update `CMakeLists.txt`.
- Spinal integration: many nodes publish/subscribe to `spinal` messages. Do not change the topic shapes without checking `spinal` package definitions. Key topics to look for:
  - `four_axes/command` (flight command for spinal)
  - `desire_coordinate` (navigation -> spinal)
  - `/servo/states` (servo status)
- Servo safety and calibration:
  - Servo parameters are loaded from `launch/bringup.launch` (`<rosparam file="$(arg config_dir)/Servo.yaml"/>`). Use `spinal`'s `servo_rough_calib.py` for rough calibration (see `launch/includes/euclid_201709/sensors.launch.xml`).
  - Always check servo angles before commanding real hardware: `rostopic echo -c /servo/states` (the README notes a range check e.g. `5 -> 4095` mapping depending on config).

## Integration points & external dependencies
- Dependencies declared in `CMakeLists.txt` / `package.xml` include: `aerial_robot_control`, `aerial_robot_model`, `aerial_robot_msgs`, `hydrus`, `mujoco_ros_control`, `pluginlib`, `roscpp`, `Eigen3`, `NLopt`, and `spinal`.
- Simulation connections are provided via `mujoco` and `robots/*` xacro/urdf files. If touching models, review `config/mujoco_model.yaml` and `mujoco_model_convert` invocation in `CMakeLists.txt`.

## Debugging & observability tips
- Use ROS tools: `rostopic echo`, `rqt_graph`, `rosnode list`, `rosparam get/set`.
- For control debugging, inspect published topics from controllers (`debug/pose/pid`, `estimate_external_wrench`), and the `spinal` topics used to send commands to the flight controller.
- For SMACH-based tests (`test/flight_check.py`) enable the SMACH introspection server to inspect state transitions.

## Files to look at when making changes
- High-level: `CMakeLists.txt`, `package.xml`, `launch/bringup.launch`.
- Controllers & models: `include/dragon/`, `src/control/`, `src/model/`.
- Plugins (runtime registration): `plugins/*.xml`.
- Hardware/config: `config/quad/Servo.yaml`, `config/mujoco_model.yaml`, `launch/includes/euclid_201709/sensors.launch.xml`.
- Tests & CI: `test/*` (rostest `.test` files and Python/CPP tests).

## Example edits to get started
- Implement a new controller:
  1. Create controller class in `include/dragon/control/` + `src/control/` and expose methods consistent with existing controllers.
  2. Add target to `CMakeLists.txt` (library or link to `dragon_aerial_robot_controllib`) and add dependency on `aerial_robot_msgs` if necessary.
  3. Register the class in `plugins/flight_control_plugins.xml` with a unique `name` used by runtime config.
  4. Add ROS params under a controller namespace and read them during `rosParamInit()` (see pattern in `full_vectoring_control.h/cpp`).

---

If any specific sections look incomplete or you want extra examples (e.g., a short code snippet showing how controllers publish spinal messages), tell me which area to expand and I will iterate.✅
