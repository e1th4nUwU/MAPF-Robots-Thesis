#!/usr/bin/env bash

set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
	echo "Run this script as a regular user with sudo access, not as root."
	exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="jazzy"

if [[ -r /etc/os-release ]]; then
	# shellcheck disable=SC1091
	source /etc/os-release
else
	echo "Could not read /etc/os-release."
	exit 1
fi

if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
	echo "This installer supports Ubuntu 24.04 only. Detected: ${PRETTY_NAME:-unknown}."
	exit 1
fi

echo "[1/7] Installing base system packages..."
sudo apt-get update
sudo apt-get install -y \
	software-properties-common \
	curl \
	ca-certificates \
	gnupg \
	lsb-release \
	git

echo "[2/7] Enabling Ubuntu universe repository..."
sudo add-apt-repository universe -y

echo "[3/7] Configuring ROS 2 apt repository..."
sudo install -d -m 0755 /etc/apt/keyrings
if [[ ! -f /etc/apt/keyrings/ros-archive-keyring.gpg ]]; then
	curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
		| sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
	| sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "[4/7] Installing ROS 2, Gazebo, and workspace dependencies..."
sudo apt-get update
sudo apt-get install -y \
	ros-${ROS_DISTRO}-desktop \
	ros-dev-tools \
	python3-colcon-common-extensions \
	python3-rosdep \
	python3-vcstool \
	python3-tk \
	python3-rtree \
	sumo \
	sumo-tools \
	ros-${ROS_DISTRO}-ros-gz \
	ros-${ROS_DISTRO}-gz-ros2-control \
	ros-${ROS_DISTRO}-moveit \
	ros-${ROS_DISTRO}-control-toolbox \
	ros-${ROS_DISTRO}-slam-toolbox \
	ros-${ROS_DISTRO}-nav2-amcl \
	ros-${ROS_DISTRO}-nav2-map-server \
	ros-${ROS_DISTRO}-nav2-util \
	ros-${ROS_DISTRO}-joint-trajectory-controller \
	ros-${ROS_DISTRO}-joint-state-broadcaster \
	ros-${ROS_DISTRO}-diff-drive-controller \
	ros-${ROS_DISTRO}-controller-manager

echo "[5/7] Initializing rosdep..."
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
	sudo rosdep init
fi
rosdep update

echo "[6/7] Installing package dependencies from ros2_ws/src..."
source /opt/ros/${ROS_DISTRO}/setup.bash
cd "${REPO_ROOT}/ros2_ws"
rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

echo "[7/7] Building workspace..."
colcon build

if ! grep -Fq "source /opt/ros/${ROS_DISTRO}/setup.bash" "${HOME}/.bashrc"; then
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${HOME}/.bashrc"
fi

if ! grep -Fq "source ${REPO_ROOT}/ros2_ws/install/setup.bash" "${HOME}/.bashrc"; then
	echo "source ${REPO_ROOT}/ros2_ws/install/setup.bash" >> "${HOME}/.bashrc"
fi

echo
echo "Installation finished. Open a new terminal or run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${REPO_ROOT}/ros2_ws/install/setup.bash"
