#  Static Load Estimation for Wheel Loader
## System architecture

<img width="847" height="484" alt="Screenshot from 2026-07-09 23-30-46" src="https://github.com/user-attachments/assets/a58aecf4-6259-45f8-b908-5bf50e8e842b" />

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
