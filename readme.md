#  Static Load Estimation for Wheel Loader
## System architecture

<img width="847" height="484" alt="Screenshot from 2026-07-09 23-24-00" src="https://github.com/user-attachments/assets/6812c6cc-a254-4a12-8021-539b674c7c5d" />

## Use this package
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
