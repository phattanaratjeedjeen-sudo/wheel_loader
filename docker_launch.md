# ROS2 Humble on Ubuntu 24.04 with Gazebo Classic

This instruction is created for launch ros2 package that required envrion which doesn't match the native setup

| Environment | Package requirements | Native setup |
| :--- | :--- | :---|
| Ubuntu | 22.04 | 24.04 |
| ROS2 | Humble | Jazzy |
| Gazebo | Classic 11.10.2 | Harmonic |

## Steps

Allow Screen Access (Once per reboot)
```bash
xhost +local:root
```

Navigate to your ROS 2 Workspace root folder
```bash
cd ~/path/to/your/workspace_root_folder
```

Run the Docker Container
```bash
docker run -it \
  --name ros2_humble \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev/dri:/dev/dri" \
  --volume="$(pwd):/workspace:rw" \
  --workdir="/workspace" \
  osrf/ros:humble-desktop-full \
  /bin/bash
```

Install Dependencies and Build (Inside the Container)
```bash
apt update
apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control -y
```

```bash
colcon build && source install/setup.bash
```

Opening multiple terminals
```bash
docker exec -it ros2_humble /bin/bash
```

## Enable RTX for Container
Install the NVIDIA Container Toolkit
```bash
# 1. Add the official NVIDIA GPG key and repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Update your package list and install the toolkit
sudo apt update
sudo apt install -y nvidia-container-toolkit

# 3. Configure Docker to use the NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker

# 4. Restart the Docker engine to apply the changes
sudo systemctl restart docker
```


## Alternative Ways (Docker Compose)
Create the `Dockerfile` at your workspace's directory
```bash
FROM osrf/ros:humble-desktop-full

# Automatically install Gazebo and ROS control dependencies during build
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*
```

Create `docker-compose.yml`
```bash
services:
  ros2:
    build: .  
    container_name: ros2_humble
    network_mode: host
    stdin_open: true 
    tty: true        
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - .:/workspace:rw  
    devices:
      - /dev/dri:/dev/dri 
    working_dir: /workspace
    command: /bin/bash
```

Spin up the new permanent environment
```bash
# 1. Stop and clear out the old container instance
docker compose down

# 2. Build the Dockerfile and start the container in the background
docker compose up -d --build
```

Allow Screen Access (Once per reboot)
```bash
xhost +local:root
```

Enter and Launch
```bash
docker exec -it ros2_humble bash
```
```bash
source install/setup.bash
ros2 launch loader_sim_pkg loader_sim.launch.py
```

Stop container (work done for a moment)
```bash
docker compose stop
```

Start container
```bash
docker compose start
```

## Tips

### Editing Code (On your Host)
You can open VS Code or any text editor natively on your Ubuntu 24.04 desktop and modify the files inside your workspace folder. Because of the `--volume="$(pwd):/workspace:rw"` flag, every change you save on your host machine instantly updates inside the container.

### Installing & Building (Inside Docker)
Whenever you need to install a new package, run a launch file, or compile your code, you must do it through the container's terminal:

- To **install dependencies**: Add a new `apt install` requirement to your Dockerfile. Run `docker compose down` and then   `docker compose up -d --build`.
- To **compile**: Run `colcon build` inside the container.
- To **run**: Run `ros2 launch ...` inside the container.