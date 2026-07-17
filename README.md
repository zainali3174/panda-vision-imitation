# multipanda_ros2
<img src="docs/images/single_sim.png" alt="" height="250">

This project implements most features from the original `franka_ros` repository in ROS2 Humble, specifically for the Franka Emika Robot (Panda).
This project significantly expands upon the original `franka_ros2` from the company, who dropped the support for the Pandas.

**The current version relies on a [fork of the repository](https://github.com/tenfoldpaper/multipanda_ros2)**


## Working features
More thorough information is available in the documentation.

* Real robot:
    * FrankaState broadcaster
    * All control interfaces (torque, position, velocity, Cartesian).
    * Example controllers for all interfaces
    * Controllers are swappable using rqt_controller_manager
    * Runtime franka::ControlException error recovery via `~/service_server/error_recovery`
        * Upon recovery, the previously executed control loop will be executed again, so no reloading necessary.
    * Runtime internal parameter setter services much like what is offered in the updated `franka_ros2`
* Sim robot:
    * Same as the real robot, except Cartesian command interface is not available, and there is no plan to implement this for now.
    * Gripper server with identical interface to the real gripper (i.e. action servers)
    * Example controllers for the real single-arm listed above, that correspond to those interfaces, work out of the box.
    * FrankaState implements the basics: torque, joint position/velocity, `O_T_EE` and `O_F_ext_hat`.
    * Model provides all the existing functions: `pose`, `zeroJacobian`, `bodyJacobian`, `mass`, `gravity`, `coriolis`.
        * Gravity for now just returns the corresponding `qfrc_gravcomp` force from mujoco.
        * Coriolis = `qfrc_bias - qfrc_gravcomp`
    * Camera is available as part of `mujoco_ros_pkg`'s features. You can simply add a `<camera>` object in your mujoco XML file, and the package will handle them.
    * With the forked repository's `mujoco_ros2_control_system` package, you can easily add components with additional degrees of freedom to your robot. Take a look at `garmi_packages/garmi_description/robots/*.ros2_control.xacro` for an example on how to do this.


## Installation with One-Click Installer
1. Clone the repository recursively to include **mujoco_ros_pkgs**:
```
git clone --recursive https://github.com/tenfoldpaper/multipanda_ros2.git
```
2. Then cd into the cloned repository:
```
cd multipanda_ros2
```
3. Build the docker image by running one command (takes some time):
```
./tools/setup_env
```
4. Once the image is built, start the development container:
```
./run
```
Default config allows for communication in the network, gpu access, display forwarding for GUI applications, hardware devices, etc. By default the script opens a bash shell inside the container as a developer user (password can be modified in Dockerfile) in ros2 workspace under ~/multipanda_ws.

5. Build ROS2 packages with:
```
colcon build
```
* In case there are some problems with missing packages, try running the following commands inside the container before `colcon build`:
    ```
    sudo apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r
    ```
6. To verify that the installation was successful, run:
```
source ~/multipanda_ws/install/setup.bash && \
ros2 launch franka_bringup franka_sim.launch.py
```
* This should open up MuJoCo simulation with one Franka Panda arm
* To open the docker container in additional terminal, use *docker exec* command:
    ```
    docker exec -it --user developer multipanda-container bash
    ```
7. (OPTIONAL) Check that the connection to the robot, RT kernel and screen are all working fine (make sure that FCI feature is activated). 
    - You can check if docker is properly linked to your screen by running:
        - `ros2 run rviz2 rviz2` or
        - `~/Libraries/mujoco/bin/simulate`
    - For RT kernel and robot connection, run
        - `~/Libraries/libfranka/bin/communication_test <robot-ip>`

## FYP Additions (this fork)

This fork extends the base `multipanda_ros2` project with additional packages developed as part of a Final Year Project:

- **`panda_cartesian_control`** — provides Cartesian (XYZ/HTM-based) motion control and pick-and-place orchestration for the Franka arm on top of MoveIt, exposing custom ROS 2 actions (`htm_motion`, `pick_place`) instead of requiring manual RViz interaction.
- **`panda_cartesian_control_msgs`** — custom action definitions (`HTMMotion`, `PickPlace`) used by the above.

### Entering the container
```bash
docker exec -it --user developer multipanda-container bash
```
Run all commands below inside the container unless noted otherwise.

### Running IK
```bash
# Simulation
ros2 launch franka_moveit_config sim_moveit.launch.py

# Hardware (fake)
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dummy use_fake_hardware:=true

# Hardware (real)
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

### Cartesian IK
**Simulation**
```bash
# Terminal 1
ros2 launch franka_moveit_config sim_moveit.launch.py

# Terminal 2
ros2 run panda_cartesian_control cartesian_moveit_server

# Terminal 3
ros2 action send_goal /cartesian_via_motion panda_motion_generator_msgs/action/CartesianViaMotion "{via_poses: [{position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}], v_scale: 0.2}"
```

**Hardware**
```bash
# Terminal 1
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2

# Terminal 2
ros2 run panda_cartesian_control cartesian_moveit_server

# Terminal 3
ros2 action send_goal /cartesian_via_motion panda_motion_generator_msgs/action/CartesianViaMotion "{via_poses: [{position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}], v_scale: 0.2}"
```

**Checking current end-effector HTM**
```bash
ros2 run tf2_ros tf2_echo panda_link0 panda_link8
```

### HTM Motion
```bash
ros2 action send_goal /htm_motion panda_cartesian_control_msgs/action/HTMMotion "{htm: [1,0,0,0.4, 0,-1,0,-0.2, 0,0,-1,0.4, 0,0,0,1], v_scale: 0.2}"

# Corrected orientation
ros2 action send_goal /htm_motion panda_cartesian_control_msgs/action/HTMMotion "{htm: [0.707,-0.707,0,0.4, -0.707,-0.707,0,-0.2, 0,0,-1,0.4, 0,0,0,1], v_scale: 0.2}"
```

### Gripper
```bash
# Simulation
ros2 action send_goal /panda_gripper_sim_node/move franka_msgs/action/Move "{width: 0.08, speed: 0.1}"

# Hardware
# Terminal 1
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 load_gripper:=true

# Terminal 2
ros2 action send_goal /panda_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.1}"
ros2 action send_goal /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.05, epsilon: {inner: 0.01, outer: 0.01}, speed: 0.05, force: 20.0}"
```

### Pick and Place
**Simulation**
```bash
# Terminal 1
ros2 launch franka_moveit_config sim_moveit.launch.py

# Terminal 2
ros2 run panda_cartesian_control cartesian_moveit_server

# Terminal 3
ros2 run panda_cartesian_control pick_place_server --ros-args -p use_sim:=true

# Terminal 4
ros2 action send_goal /pick_place panda_cartesian_control_msgs/action/PickPlace "{pick_xyz: [0.4, -0.2, 0.3], place_xyz: [0.4, 0.2, 0.3], z_offset: 0.1, grasp_width: 0.05, grasp_force: 20.0}" --feedback
```

**Hardware**
```bash
# Terminal 1
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2

# Terminal 2
ros2 run panda_cartesian_control cartesian_moveit_server

# Terminal 3
ros2 run panda_cartesian_control pick_place_server --ros-args -p use_sim:=false

# Terminal 4
ros2 action send_goal /pick_place panda_cartesian_control_msgs/action/PickPlace "{pick_xyz: [0.4, -0.2, 0.3], place_xyz: [0.4, 0.2, 0.3], z_offset: 0.1, grasp_width: 0.05, grasp_force: 20.0}" --feedback

## Credits
The original version is forked from mcbed's port of franka_ros2 for [humble][mcbed-humble].

## License

All packages of `multipanda_ros2` are licensed under the [Apache 2.0 license][apache-2.0], following `franka_ros2`.

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble
[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html
[mujoco-instructions]: https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source
[humble-instructions]: https://docs.ros.org/en/humble/Installation.html
