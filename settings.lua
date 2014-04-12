-- This file is run once at robot boot

print("Running lua settings...")

trajectory_set_acc(1300, 10)
trajectory_set_speed(SPEED_NORMAL_D, SPEED_NORMAL_A)

bd_set_threshold("distance", 3600)
bd_set_threshold("angle", 3000)

pid_set_gains(angle_pid, 0.8*400, 0, 0.8*2000)
pid_set_gains(distance_pid, 0.8*200, 0, 0.8*1000)

-- Wheel diameter correction factor
rs_set_factor(0.00098906680210644)


-- Arm regulator settings
--

shoulder_pid(10, 0, 0)
hand_pid(60, 0, 0)
z_axis_pid(900, 0, 0)
elbow_pid(10, 0, 0)

print("Lua settings done...")
