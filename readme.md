## DCBF_MPC_Planner_ROS2

### 一、WSL2 + Ubuntu 22.04 环境准备

```bash
# 在 Windows PowerShell（管理员）中安装 WSL2 和 Ubuntu 22.04
wsl --install -d Ubuntu-22.04
```

进入 Ubuntu 22.04 后：

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础工具
sudo apt install -y locales curl gnupg lsb-release software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

### 二、安装 ROS2 Humble

```bash
# 添加 ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加 ROS2 软件源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 更新并安装 ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools

# 设置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安装构建工具
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# 配置 rosdep（国内推荐清华镜像）
sudo mkdir -p /etc/ros/rosdep/sources.list.d
sudo tee /etc/ros/rosdep/sources.list.d/20-default.list > /dev/null << 'EOF'
yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/osx-homebrew.yaml osx
yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/base.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/python.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/ruby.yaml
EOF

# 修改 rosdistro 默认 URL 为清华镜像
sudo find /usr/lib/python3* -path "*/rosdistro/__init__.py" -exec sed -i 's|https://raw.githubusercontent.com/ros/rosdistro/master|https://mirrors.tuna.tsinghua.edu.cn/rosdistro|g' {} + 2>/dev/null || true

rosdep update
```

---

### 三、创建工作空间并克隆仓库

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆仓库
git clone https://github.com/zm-3171/DCBF_MPC_Planner_ROS2.git

# 进入仓库目录
cd DCBF_MPC_Planner_ROS2

# 初始化并更新子模块（turtlebot3_velodyne_Gazebo 等）
git submodule update --init --recursive
```

---

### 四、安装项目依赖

```bash
# 回到工作空间根目录
cd ~/ros2_ws

# 安装 rosdep 能解析的依赖
rosdep install --from-paths src --ignore-src -y --skip-keys turtlebot3_velodyne_Gazebo

# 手动安装已知的额外 ROS2 依赖
sudo apt install -y \
    ros-humble-rviz-visual-tools \
    ros-humble-pcl-ros \
    ros-humble-grid-map \
    ros-humble-grid-map-octomap \
    ros-humble-grid-map-core \
    ros-humble-grid-map-ros \
    ros-humble-grid-map-cv \
    ros-humble-grid-map-filters \
    ros-humble-grid-map-loader \
    ros-humble-grid-map-msgs \
    ros-humble-grid-map-pcl \
    ros-humble-grid-map-rviz-plugin \
    ros-humble-grid-map-sdf \
    ros-humble-grid-map-visualization \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-nav2-bringup

# 安装系统级依赖（CGAL、PCL 等）
sudo apt install -y \
    libcgal-dev \
    libcgal-qt5-dev \
    libpcl-dev \
    liboctomap-dev \
    libeigen3-dev

# 求解器
pip install casadi
```
---

### 五、编译项目

```bash
cd ~/ros2_ws

colcon build

# 如果只想编译特定包（可选）
# colcon build --packages-select local_planner samplelaunch
```

---

### 六、设置环境变量

```bash
# 设置 TurtleBot3 模型（通常用 burger）
export TURTLEBOT3_MODEL=burger
```

---

### 七、Launch 运行

```bash
source install/setbu.bash

# 启动sim环境
ros2 launch scene sim.launch.py 

# 启动路径规划器
ros2 launch local_planner local_planner_launch.py 
```


