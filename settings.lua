-- This file is run once at robot boot
--

print("Running lua settings...")

trajectory_set_acc(1300, 10)
trajectory_set_speed(SPEED_NORMAL_D, SPEED_NORMAL_A)

bd_set_threshold("distance", 3600)
bd_set_threshold("angle", 3000)
print("Lua settings done...")
