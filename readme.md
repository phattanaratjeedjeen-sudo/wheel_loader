#  Static Load Estimation for Wheel Loader
> Note:
> - This estimator can be use with any boom angle (theta_g)
> - Bucket should be at maximum turn up position
> - Used with XCMG958
<img width="863" height="630" alt="image" src="https://github.com/user-attachments/assets/ab5b843b-f936-4a64-841b-068586d9df62" />


## Validation
<img width="1200" height="949" alt="summary" src="https://github.com/user-attachments/assets/892901f0-1a6f-448f-b678-967b14a556b4" />


## System architecture
<img width="847" height="484" alt="Screenshot from 2026-07-09 23-30-46" src="https://github.com/user-attachments/assets/a58aecf4-6259-45f8-b908-5bf50e8e842b" />


## Use this package
```
cd ~
mkdir -p ros2_ws/src
```
```bash
cd ros2_ws/src
git clone https://github.com/phattanaratjeedjeen-sudo/wheel_loader.git
```
```bash
cd ros2_ws
colcon build && source install/setup.bash
```
```
ros2 run load_estimation load_estimate.py
```
```
ros2 service call /vehicle/load_estimate std_srvs/srv/Trigger
```
