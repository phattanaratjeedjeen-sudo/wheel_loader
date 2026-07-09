#  Static Load Estimation for Wheel Loader

```
cd ~
mkdir -p ros2_ws/src
```

```bash
cd ros2_ws
git clone https://github.com/phattanaratjeedjeen-sudo/wheel_loader.git
```

```bash
colcon build && source install/setup.bash
```

```
ros2 run load_estimation load_estimate.py
```

```
ros2 service call /vehicle/load_estimate std_srvs/srv/Trigger
```