# Leo Drive Gatevcu Driver

This is a ROS 2 driver for the gatevcu.

## How to build

The driver consists of multiple ROS 2 packages. You need to create a workspace,
pull the repo and install dependencies.

```bash
# Create a workspace
mkdir -p gatevcu_ws/src

# Go into the source directory
cd gatevcu_ws/src

# Clone the repo
git clone https://github.com/leo-drive/leodrive_gatevcu_driver.git
# or (depends on your GitHub authentication method)
git clone git@github.com:leo-drive/leodrive_gatevcu_driver.git

# Go to the workspace root directory
cd ..

# Import dependencies
vcs import src < src/leodrive_gatevcu_driver/gatevcu.repos
rosdep install --from-paths src --ignore-src -y -r

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
# or
colbu_rd
```

## How to control via gamepad

Source the workspace and:

```bash
ros2 launch leodrive_gatevcu_driver driver.launch.xml
```

## Dual Shock Controls

![img.png](img.png)

<br />

![img_1.png](img_1.png)