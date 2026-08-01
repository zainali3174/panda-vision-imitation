# multipanda_ros2

<img src="docs/images/single_sim.png" alt="" height="250">

This project implements most features from the original `franka_ros` repository in ROS2 Humble, specifically for the Franka Emika Robot (Panda). This project significantly expands upon the original `franka_ros2` from the company, who dropped the support for the Pandas.

**The current version relies on a [fork of the repository](https://github.com/tenfoldpaper/multipanda_ros2)**

## Working Features

More thorough information is available in the documentation.

### Real Robot

- FrankaState broadcaster
- All control interfaces (torque, position, velocity, Cartesian)
- Example controllers for all interfaces
- Controllers are swappable using rqt_controller_manager
- Runtime `franka::ControlException` error recovery via `~/service_server/error_recovery`
  - Upon recovery, the previously executed control loop will be executed again, so no reloading necessary
- Runtime internal parameter setter services much like what is offered in the updated `franka_ros2`

### Sim Robot

- Same as the real robot, except Cartesian command interface is not available, and there is no plan to implement this for now
- Gripper server with identical interface to the real gripper (i.e. action servers)
- Example controllers for the real single-arm listed above, that correspond to those interfaces, work out of the box
- FrankaState implements the basics: torque, joint position/velocity, `O_T_EE` and `O_F_ext_hat`
- Model provides all the existing functions: `pose`, `zeroJacobian`, `bodyJacobian`, `mass`, `gravity`, `coriolis`
  - Gravity for now just returns the corresponding `qfrc_gravcomp` force from mujoco
  - Coriolis = `qfrc_bias - qfrc_gravcomp`
- Camera is available as part of `mujoco_ros_pkg`'s features. You can simply add a `<camera>` object in your mujoco XML file, and the package will handle them.
- With the forked repository's `mujoco_ros2_control_system` package, you can easily add components with additional degrees of freedom to your robot. Take a look at `garmi_packages/garmi_description/robots/*.ros2_control.xacro` for an example on how to do this.

## Installation — Windows 11

Open `cmd` as administrator and paste:

```bash
wsl --install -d Ubuntu
```

### Two cases

* **If it asks for a username and password:** Enter any username and password of your choice. This creates a **new Linux user account**, separate from your Windows account. It may also ask whether to enable metrics collection; either option is fine. Once the setup is complete, **reboot your device**.

* **If it does not ask for a username or password:** Wait for the command to finish, then **reboot your device**. After restarting, the Ubuntu setup may resume automatically. Let it finish, then create a **new Linux username and password** when prompted. If nothing opens automatically after reboot, launch **Ubuntu** app from the Start menu to continue the setup.


Open a **separate** `cmd` window aas administrator and run:

```bash
wsl -l -v
```

Confirm `VERSION` shows `2`, e.g.:

```
NAME      STATE           VERSION
* Ubuntu    Running         2
```

Still in that same `cmd` window (**not** the Ubuntu one), run:

```bash
notepad "%USERPROFILE%\.wslconfig"
```

Press **Yes** to create the file. Paste:

```ini
[wsl2]
networkingMode=mirrored
```

Save, close, then:

```bash
wsl --shutdown
```

Reopen the Ubuntu app from the Start menu.


Install **Docker Desktop for Windows** ([Download from here](https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe?utm_source=docker&utm_medium=webreferral&utm_campaign=docs-driven-download-win-amd64&_gl=1*1pk98sm*_ga*NTY5NjIyMDMxLjE3ODU1NzI2OTk.*_ga_XJWPQMJYHQ*czE3ODU1NzI2OTgkbzEkZzAkdDE3ODU1NzI2OTgkajYwJGwwJGgw)), Run the .exe file and install. Make sure "Use WSL 2 instead of Hyper-V" is checked during install (this is the default, so just install as it is). After install, go to **Docker Desktop App→ Settings → Resources → WSL Integration**:

- "Enable integration with my default WSL distro" — **ON**
- Your Ubuntu distro toggled **ON** individually
- **Apply & Restart**

Back in the Ubuntu WSL terminal:

```bash
docker info
```

If you see "permission denied" — close the Ubuntu window entirely, reopen, and retry.

**(Optional, if you have an NVIDIA GPU):**

```bash
nvidia-smi
```

If this shows your GPU, Docker will auto-detect it later, nothing else needed.

Clone the repository:

```bash
cd ~
mkdir -p UET_BRIDGE
cd UET_BRIDGE
git clone --recursive https://github.com/zainali3174/multipanda_ros2.git
cd multipanda_ros2
```

Set up and enter the container:

```bash
./tools/setup_env
sudo apt update
sudo apt install -y x11-xserver-utils
./run/
```
If asked for a password, enter the password you created for your Linux user account.

Install pinnochio:

```bash
pip install pin
```

You're now **inside** the container at a `developer@docker-desktop` prompt. Still inside the container, build the workspace:

```bash
colcon build
```

Make ROS see your built packages in every future shell:

```bash
source ~/multipanda_ws/install/setup.bash 
echo "source ~/multipanda_ws/install/setup.bash" >> ~/.bashrc
```

To open a **second** terminal into the same running container later (open a new WSL/Ubuntu terminal window, then run):

```bash
docker exec -it --user developer multipanda-container /bin/bash -c "source /home/developer/.bashrc && bash"
```


## Installation — Windows 10 (Not Verified)

Everything above is identical, **except**:

- Mirrored networking (the `.wslconfig` step) may not be supported on older Windows 10 builds. Check with `wsl --version` , if `networkingMode` isn't recognized after `wsl --shutdown`, skip that step and flag it so the `run` script's `--network` flag can be adjusted instead.
- Install **VcXsrv** (search "VcXsrv" on SourceForge), launch it via XLaunch with "Disable access control" checked, and leave it running in the background.
- Add this line to `~/.bashrc` **inside WSL** (not the container) before running `./run`:

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

Then:

```bash
source ~/.bashrc
```

Make sure VcXsrv is already running before `./run`.

## Installation — Ubuntu 22.04

1. Clone the repository recursively to include **mujoco_ros_pkgs**:

```bash
git clone --recursive https://github.com/zainali3174/multipanda_ros2.git
```

2. Change into the cloned repository:

```bash
cd multipanda_ros2
```

3. Build the docker image by running one command (takes some time):

```bash
./tools/setup_env
```

4. Once the image is built, start the development container:

```bash
./run
```
Install pinnochio:

```bash
pip install pin
```

The default config allows for communication in the network, GPU access, display forwarding for GUI applications, hardware devices, etc. By default the script opens a bash shell inside the container as a `developer` user (password can be modified in the Dockerfile) in the ROS2 workspace under `~/multipanda_ws`.

5. Build ROS2 packages with:

```bash
colcon build
```

Make ROS see your built packages in every future shell:

```bash
source ~/multipanda_ws/install/setup.bash 
echo "source ~/multipanda_ws/install/setup.bash" >> ~/.bashrc
```

In case there are problems with missing packages, try running the following commands inside the container before `colcon build`:

```bash
sudo apt update && \
rosdep update && \
rosdep install --from-paths src --ignore-src -y -r
```

6. To verify that the installation was successful, run:

```bash
source ~/multipanda_ws/install/setup.bash 
ros2 launch franka_bringup franka_sim.launch.py
```

This should open up a MuJoCo simulation with one Franka Panda arm.

To open the docker container in an additional terminal, use the `docker exec` command:

```bash
docker exec -it --user developer multipanda-container bash
```

## Usage Commands

This fork extends the base `multipanda_ros2` project with additional packages developed as part of a Final Year Project:

### Opening a terminal inside container 

```bash
docker exec -it --user developer multipanda-container bash
```

Run the commands below inside the container unless noted otherwise.

## Start Mujuco, RViz and related nodes

```bash
# Terminal 1
ros2 launch franka_moveit_config sim_moveit.launch.py

# Terminal 2
ros2 run panda_cartesian_control cartesian_moveit_server

# Terminal 3
ros2 run panda_cartesian_control pick_place_server --ros-args -p use_sim:=true
```

**Checking current end-effector HTM**

```bash
ros2 run tf2_ros tf2_echo panda_link0 panda_link8
```

### HTM Motion

```bash
ros2 action send_goal /htm_motion panda_cartesian_control_msgs/action/HTMMotion "{htm: [0.707,-0.707,0,0.4, -0.707,-0.707,0,-0.2, 0,0,-1,0.4, 0,0,0,1], v_scale: 0.2}"
```

### Gripper

```bash
# Simulation
ros2 action send_goal /panda_gripper_sim_node/move franka_msgs/action/Move "{width: 0.08, speed: 0.1}"
ros2 action send_goal /panda_gripper_sim_node/grasp franka_msgs/action/Grasp "{width: 0.05, epsilon: {inner: 0.01, outer: 0.01}, speed: 0.05, force: 20.0}"
```

### Pick and Place

**Simulation**

```bash
ros2 action send_goal /pick_place panda_cartesian_control_msgs/action/PickPlace "{pick_xyz: [0.28, -0.18, 0.18], place_xyz: [0.28, 0.18, 0.18], z_offset: 0.1, grasp_width: 0.05, grasp_force: 20.0}" --feedback
```
### Adding and Removing obstacle in RViz

Use these commands to add and remove obstacle in RViz. These are not permanent and wont be there if you restart your setup. These obstacles will also not be visible in Mujuco.

**Adding the obstacle**
```bash
python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
rclpy.init()
node = Node('add_obstacle')
pub = node.create_publisher(PlanningScene, '/planning_scene', 10)
import time
time.sleep(1.0)  # let publisher connect
obj = CollisionObject()
obj.header.frame_id = 'panda_link0'
obj.id = 'human_obstacle'
obj.operation = CollisionObject.ADD
primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = [0.06, 0.34, 0.2]
pose = Pose()
pose.position.x = 0.0
pose.position.y = -0.3
pose.position.z = 0.15
pose.orientation.w = 1.0
obj.primitives.append(primitive)
obj.primitive_poses.append(pose)
scene = PlanningScene()
scene.world.collision_objects.append(obj)
scene.is_diff = True
for _ in range(3):
    pub.publish(scene)
    time.sleep(0.5)
print("Published collision object")
node.destroy_node()
rclpy.shutdown()
PYEOF
```

**Removing/ the obstacle**
```bash
python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
import time

rclpy.init()
node = Node('remove_obstacle')
pub = node.create_publisher(PlanningScene, '/planning_scene', 10)

time.sleep(1.0)

obj = CollisionObject()
obj.header.frame_id = 'panda_link0'
obj.id = 'human_obstacle'
obj.operation = CollisionObject.REMOVE

scene = PlanningScene()
scene.world.collision_objects.append(obj)
scene.is_diff = True

for _ in range(3):
    pub.publish(scene)
    time.sleep(0.5)

print("Removed collision object")
node.destroy_node()
rclpy.shutdown()
PYEOF
```


## License

All packages of `multipanda_ros2` are licensed under the [Apache 2.0 license][apache-2.0], following `franka_ros2`.

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble
[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html
[mujoco-instructions]: https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source
[humble-instructions]: https://docs.ros.org/en/humble/Installation.html
