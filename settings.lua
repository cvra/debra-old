-- This file is run once at robot boot

print("Running lua settings...")

trajectory_set_acc(1000, 20)
trajectory_set_speed(SPEED_NORMAL_D, SPEED_NORMAL_A)

bd_set_threshold("distance", 3600)
bd_set_threshold("angle", 3000)

pid_set_gains(angle_pid, 161, 0, 51.2)
pid_set_gains(distance_pid, 0.6*200, 0, 0.8*1000)

-- Wheel diameter correction factor
rs_set_factor(0.00098906680210644)


-- Arm regulator settings
--

shoulder_pid(10, 0, 0)
--hand_pid(350, 0, 100)
hand_pid(60, 0, 100)
z_axis_pid(900, 0, 0)
elbow_pid(10, 0, 0)

print("Lua settings done...")
