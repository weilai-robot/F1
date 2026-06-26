# ════════════════════════════════════════════════════════════════════
# F1 Navigation Sim — Dockerfile (Ubuntu 22.04 Server)
#
# 构建命令:
#   docker build -t f1-nav-sim .
#
# 运行命令:
#   # 有显示器的服务器 (X11 forwarding):
#   docker run --rm -it --network=host \
#       -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#       -v $(pwd)/reports:/workspace/reports \
#       f1-nav-sim
#
#   # 无头服务器 (Xvfb, 自动启用):
#   docker run --rm -it --network=host \
#       -v $(pwd)/reports:/workspace/reports \
#       f1-nav-sim
#
#   # 交互模式 (自己选启动脚本):
#   docker run --rm -it --network=host \
#       -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#       -v $(pwd)/reports:/workspace/reports \
#       f1-nav-sim bash
# ════════════════════════════════════════════════════════════════════

# ── Stage 1: Builder ────────────────────────────────────────────────
FROM ros:humble AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV AMENT_PREFIX_PATH=/opt/ros/humble

# --- System deps ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gcc-13 \
    g++-13 \
    libprotobuf-dev \
    protobuf-compiler \
    libeigen3-dev \
    libglfw3-dev \
    libdart-external-lodepng-dev \
    libsdl2-dev \
    jstest-gtk \
    libyaml-cpp-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-nav2-bringup \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-collision-monitor \
    ros-humble-octomap-server \
    ros-humble-tf2-tools \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-velocity-smoother \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    tmux \
    xvfb \
    mesa-utils \
    libgl1-mesa-glx \
    libglew-dev \
    && rm -rf /var/lib/apt/lists/*

# --- gcc-13 as default ---
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 130 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-13 \
    --slave /usr/bin/gcc-ar gcc-ar /usr/bin/gcc-ar-13 \
    --slave /usr/bin/gcc-nm gcc-nm /usr/bin/gcc-nm-13 \
    --slave /usr/bin/gcc-ranlib gcc-ranlib /usr/bin/gcc-ranlib-13

# --- Python deps ---
RUN pip3 install --no-cache-dir \
    "numpy>=1.20.0" \
    "scipy" \
    "matplotlib" \
    "pynput"

# --- Copy source ---
WORKDIR /workspace
COPY . /workspace/src

# --- Build motion_control (AimRT FetchContent takes ~15min, cached in layer) ---
WORKDIR /workspace/src
RUN source /opt/ros/humble/setup.bash && \
    cmake -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=./build/install \
        -DXYBER_X1_INFER_BUILD_TESTS=OFF \
        -DXYBER_X1_INFER_SIMULATION=ON \
        -DCMAKE_C_COMPILER=gcc-13 \
        -DCMAKE_CXX_COMPILER=g++-13 \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    cmake --build build --config Release --target install --parallel $(nproc)

# --- Build navigation (colcon) ---
WORKDIR /workspace/src/navigation
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
        --packages-select \
            livox_ros_driver2 \
            fast_lio \
            open3d_loc \
            ros2_livox_simulation \
            humanoid_sim

# --- Install MuJoCo-LiDAR (pip) ---
WORKDIR /workspace/src/navigation
RUN pip3 install --no-cache-dir -e MuJoCo-LiDAR


# ── Stage 2: Runtime ────────────────────────────────────────────────
FROM ros:humble AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# --- Runtime deps (no build tools) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    tmux \
    xvfb \
    mesa-utils \
    libgl1-mesa-glx \
    libglfw3 \
    libglew2.2 \
    libsdl2-2.0-0 \
    libyaml-cpp0.7 \
    libprotobuf23 \
    python3-pip \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# --- ROS2 runtime packages ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-bringup \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-collision-monitor \
    ros-humble-octomap-server \
    ros-humble-tf2-tools \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# --- Python runtime deps ---
RUN pip3 install --no-cache-dir \
    "numpy>=1.20.0" \
    "scipy" \
    "matplotlib"

# --- Copy built motion_control ---
WORKDIR /workspace
COPY --from=builder /workspace/src/build/ ./build/

# --- Copy built navigation workspace ---
COPY --from=builder /workspace/src/navigation/install/ ./navigation/install/

# --- Copy MuJoCo-LiDAR Python package ---
COPY --from=builder /usr/local/lib/python3.10/dist-packages/ /usr/local/lib/python3.10/dist-packages/
COPY --from=builder /workspace/src/navigation/MuJoCo-LiDAR/ ./navigation/MuJoCo-LiDAR/

# --- Copy scripts and configs ---
COPY --from=builder /workspace/src/scripts/ ./scripts/

# --- Copy source (for colcon symlink-install to work at runtime) ---
COPY --from=builder /workspace/src/navigation/humanoid_sim/ ./navigation/humanoid_sim/
COPY --from=builder /workspace/src/navigation/fast_lio2/ ./navigation/fast_lio2/
COPY --from=builder /workspace/src/navigation/MuJoCo-LiDAR/ ./navigation/MuJoCo-LiDAR/

# --- Environment ---
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV MUJOCO_LIDAR_SRC=/workspace/navigation/MuJoCo-LiDAR/src
ENV DISPLAY=:0

# --- Entrypoint ---
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
