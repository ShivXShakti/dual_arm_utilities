# dual_arm_utilities
```bash
ros2 launch dual_arm_utilities get_status.launch.py \
--ros-args -p position:=true -p velocity:=false -p torque:=false -p ft:=false \
-p sampling_frequency:=100.0 \
-p data_at_sampling_frequency:=true \
-p base_path:= '/home/cstar/Documents/dual_arm_ws/src/dual_arm_utilities/data'
-p file_name:= 'exp1'
```
```bash
ros2 run dual_arm_utilities get_status --ros-args -p position:=true -p velocity:=false -p torque:=false -p ft:=false
```
