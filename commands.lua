-- This file is run before each TCP shell is handled to a user and can be
-- used to define new commands.

SPEED_NORMAL_D = 800
SPEED_CALAGE_D = 100

SPEED_NORMAL_A = 20
SPEED_CALAGE_A = 1.

-- Standard flags to be used with wait_traj_end
TRAJ_FLAGS_SHORT_DISTANCE = END_TRAJ + END_ERROR + END_TIMER + END_BLOCKING
TRAJ_FLAGS_STD = TRAJ_FLAGS_SHORT_DISTANCE + END_OBSTACLE

function pos()
    x,y,a = position_get()
    print("("..x..";"..y..";"..a..")")
end

function wait_traj_end(why)
    repeat
        val = test_traj_end(why)
        print(val)
    until val ~= 0
    print("Got "..val)
    return val
end

function calage()
    bd_set_threshold("distance", 6000)
    bd_set_threshold("distance", 6000)
    trajectory_set_speed(SPEED_CALAGE_D, SPEED_CALAGE_A)
    mode("distance")
    forward(-2000)
    wait_traj_end(END_BLOCKING)
    mode("all")
end

function calibrate_wheels(count)
    trajectory_set_acc(160, 1.94)
    calage()
    

    start_angle = rs_get_angle()
    start_distance = rs_get_distance()

    forward(50)
    wait_traj_end(END_TRAJ)

    while count > 0 do
        forward(1150)
        wait_traj_end(END_TRAJ)

        turn(180)
        wait_traj_end(END_TRAJ)

        forward(1150)
        wait_traj_end(END_TRAJ)

        turn(-180)
        wait_traj_end(END_TRAJ)
        count = count-1
    end

    forward(-25)
    wait_traj_end(TRAJ_FLAGS_STD)

    calage()

    delta_a = start_angle - rs_get_angle()
    delta_d = start_distance - rs_get_distance()

    factor = delta_a / delta_d

    print("Correction factor : "..factor)
    rs_set_factor(factor)
end

function shoulder_pid(p, i, d)
    if p == nil or i == nil or d == nil then
        return
    end
    pid_set_gains(right_shoulder_pid, p, i, d)
    pid_set_gains(left_shoulder_pid, p, i, d)
end

function z_axis_pid(p, i, d)
    if p == nil or i == nil or d == nil then
        return
    end
    pid_set_gains(right_z_axis_pid, p, i, d)
    pid_set_gains(left_z_axis_pid, p, i, d)
end

function elbow_pid(p, i, d)
    if p == nil or i == nil or d == nil then
        return
    end
    pid_set_gains(right_elbow_pid, p, i, d)
    pid_set_gains(left_elbow_pid, p, i, d)
end

function hand_pid(p, i, d)
    if p == nil or i == nil or d == nil then
        return
    end
    pid_set_gains(right_hand_pid, p, i, d)
    pid_set_gains(left_hand_pid, p, i, d)
end

function arm()
    x,y,z = arm_get_position("left")
    print("left arm : ("..x..";"..y..";"..z..")")
    x,y,z = arm_get_position("right")
    print("right arm : ("..x..";"..y..";"..z..")")
end


function arm_demo()
    -- Shutdown the motors to be able to move the robot by hand
    mode("off")

    x,y,z = arm_get_position("right")

    duration = 5

    if z < 100 then
        calibrate()
    end

    x,y,z = arm_get_position("right")
    points = {
        {x=0, y=200, z=z, angle=90, type=COORDINATE_TABLE, duration=0.5},
    }

    arm_move("right", points)
end

function hand()

    x,y,z = arm_get_position("right")
    points = {
        {x= x, y=y, z=z, angle=0,  type=COORDINATE_ARM, duration=0.5},
        {x= x, y=y, z=z, angle=90, type=COORDINATE_ARM, duration=0.2},
--        {x= x, y=y, z=z, angle=0,  type=COORDINATE_ARM, duration=1.},
    }
    arm_move("right", points)

    x,y,z = arm_get_position("left")
    points = {
        {x= x, y=y, z=z, angle=0,  type=COORDINATE_ARM, duration=0.5},
        {x= x, y=y, z=z, angle=90, type=COORDINATE_ARM, duration=0.2},
--        {x= x, y=y, z=z, angle=0,  type=COORDINATE_ARM, duration=1.},
    }
    arm_move("left", points)
end

function reset_arms()
    x,y,z = arm_get_position("right")
    points = {{x= x, y=y, z=192,  type=COORDINATE_ARM, duration=0.5}}
    arm_move("right", points)

    x,y,z = arm_get_position("left")
    points = {{x= x, y=y, z=192,  type=COORDINATE_ARM, duration=0.5}}
    arm_move("left", points)
end

function arm_move(arm, point_list)

    if arm == nil or point_list == nil then
        return
    end

    if #point_list == 0 then
        return
    end

    traj = arm_traj_create()

    x,y,z = arm_get_position(arm)
    
    -- Duration not used
    arm_traj_append(traj, x, y, z, COORDINATE_ARM, 1.)


    for i=1, #point_list do 
        arm_traj_append(traj,
                        point_list[i].x,
                        point_list[i].y,
                        point_list[i].z,
                        point_list[i].type,
                        point_list[i].duration) 

        if point_list[i].angle ~= nil then
            arm_traj_set_hand_angle(traj, point_list[i].angle)
        end
    end

    arm_do_trajectory(arm, traj)

    arm_traj_delete(traj)
end

function pump(p, v)
    v = v * 500 -- speed
    if p == "left_top" then
        pwm(hexmotor, 3, v)
    end

    if p == "left_bottom" then
        pwm(hexmotor, 6, v)
    end

    if p == "right_top" then
        pwm(hexmotor, 7, v)
    end

    if p == "right_bottom" then
        pwm(hexmotor, 4, v)
    end
end

function dump_coders()
    print("HEXMOTOR")
    for i = 1,7 do
        print(encoder_get(hexmotor, i))
    end

    print("ARMMOTOR")
    for i = 1,7 do
        print(encoder_get(armmotor, i))
    end
end

function prepare_start(color)
    if color == nil then
        print("Please specify color : red or yellow.")
    end

    strat_autopos(color, 139.49, 541, 42.75)
end

function off()
    for i=0,7 do
        pwm(hexmotor, i, 0)
        pwm(armmotor, i, 0)
    end
end

function r()
    prepare_start("red")
    trajectory_set_acc(1000, 20)
    trajectory_set_speed(SPEED_NORMAL_D, SPEED_NORMAL_A)
end

function y()
    prepare_start("yellow")
--    trajectory_set_acc(1000, 20)
--    trajectory_set_speed(SPEED_NORMAL_D, SPEED_NORMAL_A)
end



-- Finally greet the user if running in interactive mode
function greet()
    print("Wilkommen bei ReichOS 1.0 !")
    print("Die offizielle Scriptsprache des vierten Reichs !")
    print("---------------------------------------------------------------------------")
end


if __conn then
    greet()
end


