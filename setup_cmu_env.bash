# Source this file to setup the environment for CMU Exploration
# usage: source ~/cmu_exploration_ws/setup_cmu_env.bash

# Reset PATH to system defaults to avoid Conda conflicts
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace
source ~/cmu_exploration_ws/install/setup.bash

echo "CMU Exploration environment loaded (Conda deactivated)."

