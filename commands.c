#include <platform.h>
#include <stdio.h>
#include <uptime.h>
#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"
#include "cvra_cs.h"
#include "strat_utils.h"
#include "arm_trajectories.h"
#include "arm_init.h"
#include <2wheels/trajectory_manager_utils.h>

int cmd_pio_read(lua_State *l)
{
    lua_pushnumber(l, IORD(PIO_BASE, 0));
    return 1;
}

int cmd_uptime_get(lua_State *l)
{
    lua_pushnumber(l, uptime_get());
    return 1;
}

int cmd_bluetooth_send(lua_State *l)
{
    if (lua_gettop(l) < 1)
        return 0;

    FILE *f = fopen("/dev/comDEBUG", "w");

    fprintf(f, "%s\n", lua_tostring(l, -1));
    fclose(f);

    return 0;
}

int cmd_mode(lua_State *l)
{
    char *mode;

    if (lua_gettop(l) < 1)
        return 0;

    mode = lua_tostring(l, -1);

    if(!strcmp("angle", mode))
        robot.mode = BOARD_MODE_ANGLE_ONLY;
    if(!strcmp("distance", mode))
        robot.mode = BOARD_MODE_DISTANCE_ONLY;
    if(!strcmp("off", mode))
        robot.mode = BOARD_MODE_FREE;
    if(!strcmp("all", mode))
        robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    trajectory_hardstop(&robot.traj);

    return 0;
}

int cmd_position_get(lua_State *l)
{
    lua_pushnumber(l, position_get_x_float(&robot.pos));
    lua_pushnumber(l, position_get_y_float(&robot.pos));
    lua_pushnumber(l, position_get_a_rad_float(&robot.pos));
    return 3;
}

int cmd_encoders_get(lua_State *l)
{
    int32_t *adress;
    int channel;

    if (lua_gettop(l) < 2)
        return 0;

    adress = lua_touserdata(l, -2);
    channel = lua_tointeger(l, -1);

    lua_pushnumber(l, cvra_dc_get_encoder(adress, channel));

    return 1;
}

int cmd_pwm(lua_State *l)
{
    int32_t *adress;
    int channel, value;

    if (lua_gettop(l) < 3)
        return 0;

    adress = lua_touserdata(l, -3);
    channel = lua_tointeger(l, -2);
    value = lua_tointeger(l, -1);

    cvra_dc_set_pwm(adress, channel, value);

    return 0;
}

int cmd_forward(lua_State *l)
{
    int distance;
    if (lua_gettop(l) < 1)
        return 0;

    distance = lua_tointeger(l, -1);

    trajectory_d_rel(&robot.traj, distance);

    return 0;
}

int cmd_turn(lua_State *l)
{
    int distance;
    if (lua_gettop(l) < 1)
        return 0;

    distance = lua_tointeger(l, -1);

    trajectory_a_rel(&robot.traj, distance);

    return 0;
}

int cmd_set_bd_params(lua_State *l)
{

    int thresold;

    if (lua_gettop(l) < 2)
        return 0;

    thresold = lua_tointeger(l,-1);

    if (!strcmp(lua_tostring(l, -2), "angle"))
        bd_set_thresholds(&robot.angle_bd, thresold, 1);
    else
        bd_set_thresholds(&robot.distance_bd, thresold, 1);

    return 0;
}

int cmd_bd_reset(lua_State *l)
{
    bd_reset(&robot.angle_bd);
    bd_reset(&robot.angle_bd);
    return 0;
}

int cmd_set_traj_speed(lua_State *l)
{
    float angle, distance;

    if (lua_gettop(l) < 2)
        return 0;

    distance = lua_tonumber(l, -2);
    angle = lua_tonumber(l, -1);

    trajectory_set_speed(&robot.traj,
            speed_mm2imp(&robot.traj, distance),
            speed_rd2imp(&robot.traj, angle));
    return 0;
}

int cmd_set_traj_acc(lua_State *l)
{
    float angle, distance;

    if (lua_gettop(l) < 2)
        return 0;

    distance = lua_tonumber(l, -2);
    angle = lua_tonumber(l, -1);

    trajectory_set_acc(&robot.traj,
            acc_mm2imp(&robot.traj, distance),
            acc_rd2imp(&robot.traj, angle));
    return 0;
}

int cmd_test_traj_end(lua_State *l)
{
    int why;
    if (lua_gettop(l) < 1) {
        lua_pushnumber(l, END_ERROR);
        return 1;
    }

    why = lua_tointeger(l, -1);
    lua_pushinteger(l, test_traj_end(why));

    return 1;
}

int cmd_rs_get_angle(lua_State *l)
{
    lua_pushinteger(l, rs_get_angle(&robot.rs));
    return 1;
}

int cmd_rs_get_distance(lua_State *l)
{
    lua_pushinteger(l, rs_get_distance(&robot.rs));
    return 1;
}

int cmd_set_wheel_correction_factor(lua_State *l)
{

    float factor, left_gain, right_gain;
    if (lua_gettop(l) < 1)
        return 0;

    factor = lua_tonumber(l, -1);

    left_gain = (1. + factor) * robot.rs.left_ext_gain;
    right_gain = (1. - factor) * robot.rs.right_ext_gain;

    robot.rs.left_ext_gain = left_gain;
    robot.rs.right_ext_gain = right_gain;

    return 0;
}

int cmd_traj_goto(lua_State *l)
{
    int x,y;
    if (lua_gettop(l) < 2)
        return 0;

    x = lua_tointeger(l, -2);
    y = lua_tointeger(l, -2);



}

int cmd_set_pid_gains(lua_State *l)
{

   struct pid_filter *pid; 
   int p, i, d;

    if (lua_gettop(l) < 4)
        return 0;


   p = lua_tointeger(l, -3);
   i = lua_tointeger(l, -2);
   d = lua_tointeger(l, -1);
   pid = lua_touserdata(l, -4);

   if (pid) {
       pid_set_gains(pid, p, i, d);
   }
   return 0;
}

int cmd_angle_calibrate(lua_State *l)
{
    int32_t start_angle, delta_angle;
    float factor;

    int count;
    if (lua_gettop(l) < 1)
        count = 1;
    else
        count = lua_tointeger(l, -1);

    robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    start_angle = rs_get_angle(&robot.rs);

    trajectory_d_rel(&robot.traj, 100);
    while(!trajectory_finished(&robot.traj));

    trajectory_a_rel(&robot.traj, count*360);
    while(!trajectory_finished(&robot.traj));


    trajectory_d_rel(&robot.traj, -50);
    while(!trajectory_finished(&robot.traj));

    robot.mode = BOARD_MODE_DISTANCE_ONLY;
    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);
    while(!bd_get(&robot.distance_bd));
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    delta_angle = rs_get_angle(&robot.rs) - start_angle;
    delta_angle -= pos_rd2imp(&robot.traj, RAD(360*count));

    // if factor > 0, then the robot turns too much
    factor = (float)delta_angle / (float)(pos_rd2imp(&robot.traj,RAD(360*count)));
    factor = (1.+factor)*robot.pos.phys.track_mm;

    printf("Suggested track : %.8f [mm]\n", factor);
    printf("Old track : %.8f [mm]\n", robot.pos.phys.track_mm);

    robot.pos.phys.track_mm = factor;

    lua_pushnumber(l, factor);
    return 1;
}

int cmd_shoulder_move(lua_State *l)
{

    float a;

    if (lua_gettop(l) < 1)
        return 0;

    a = lua_tonumber(l, -1);

    a = a * 3.14 / 180.;

    cs_set_consign(&robot.right_arm.shoulder.manager, a * robot.right_arm.shoulder_imp_per_rad);

    return 0;
}

int cmd_elbow_move(lua_State *l)
{

    float a;

    if (lua_gettop(l) < 1)
        return 0;

    a = lua_tonumber(l, -1);

    a = a * 3.14 / 180.;

    cs_set_consign(&robot.right_arm.elbow.manager, a * robot.right_arm.shoulder_imp_per_rad);

    return 0;
}

int cmd_arm_traj_test(lua_State *l)
{

    float x, y, z;
    float sx, sy, sz;

    arm_trajectory_t traj;
    arm_trajectory_init(&traj);
    if (lua_gettop(l) < 3)
        return 0;

    x = lua_tonumber(l, -3);
    y = lua_tonumber(l, -2);
    z = lua_tonumber(l, -1);


    arm_get_position(&robot.left_arm, &sx, &sy, &sz);

    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, x, y, z, COORDINATE_ARM, .5);

    arm_do_trajectory(&robot.left_arm, &traj);

    arm_trajectory_delete(&traj);
}

int cmd_hand_test(lua_State *l)
{
    float sx, sy, sz;


    arm_trajectory_t traj;
    arm_trajectory_init(&traj);

    arm_get_position(&robot.left_arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, -100, sz, COORDINATE_ARM, .5);
    arm_trajectory_append_point(&traj, 200, -100, 50, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, 200, -100, 31, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, -100, 120, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 0);
    arm_trajectory_append_point(&traj, 200, -100, 220, COORDINATE_ARM, 1.);

    arm_do_trajectory(&robot.left_arm, &traj);

    arm_trajectory_delete(&traj);
    return 0;
}

int cmd_get_hand_pos(lua_State *l)
{
    float sx, sy, sz;



}

int cmd_calibrate(lua_State *l)
{
    arm_calibrate();
    return 0;
}

void commands_register(lua_State *l)
{
    lua_pushcfunction(l, cmd_pio_read);
    lua_setglobal(l, "pio_get");

    lua_pushcfunction(l, cmd_uptime_get);
    lua_setglobal(l, "uptime_get");

    lua_pushcfunction(l, cmd_position_get);
    lua_setglobal(l, "position_get");

    lua_pushcfunction(l, cmd_bluetooth_send);
    lua_setglobal(l, "log");

    lua_pushcfunction(l, cmd_mode);
    lua_setglobal(l, "mode");

    lua_pushcfunction(l, cmd_forward);
    lua_setglobal(l, "forward");

    lua_pushcfunction(l, cmd_turn);
    lua_setglobal(l, "turn");

    lua_pushcfunction(l, cmd_encoders_get);
    lua_setglobal(l, "encoder_get");

    lua_pushcfunction(l, cmd_pwm);
    lua_setglobal(l, "pwm");

    lua_pushcfunction(l, cmd_set_bd_params);
    lua_setglobal(l, "bd_set_threshold");

    lua_pushcfunction(l, cmd_bd_reset);
    lua_setglobal(l, "bd_reset");

    lua_pushcfunction(l, cmd_set_traj_speed);
    lua_setglobal(l, "trajectory_set_speed");

    lua_pushcfunction(l, cmd_set_traj_acc);
    lua_setglobal(l, "trajectory_set_acc");

    lua_pushcfunction(l, cmd_test_traj_end);
    lua_setglobal(l, "test_traj_end");

    lua_pushcfunction(l, cmd_rs_get_angle);
    lua_setglobal(l, "rs_get_angle");

    lua_pushcfunction(l, cmd_rs_get_distance);
    lua_setglobal(l, "rs_get_distance");

    lua_pushcfunction(l, cmd_set_wheel_correction_factor);
    lua_setglobal(l, "rs_set_factor");

    lua_pushcfunction(l, cmd_set_pid_gains);
    lua_setglobal(l, "pid_set_gains");

    lua_pushcfunction(l, cmd_angle_calibrate);
    lua_setglobal(l, "angle_calibrate");

    lua_pushcfunction(l, cmd_shoulder_move);
    lua_setglobal(l, "shoulder_move");

    lua_pushcfunction(l, cmd_elbow_move);
    lua_setglobal(l, "elbow_move");

    lua_pushcfunction(l, cmd_arm_traj_test);
    lua_setglobal(l, "arm_traj");

    lua_pushcfunction(l, cmd_hand_test);
    lua_setglobal(l, "hand_traj");

    lua_pushcfunction(l, cmd_calibrate);
    lua_setglobal(l, "calibrate");

    lua_pushinteger(l, END_TRAJ);
    lua_setglobal(l, "END_TRAJ");

    lua_pushinteger(l, END_BLOCKING);
    lua_setglobal(l, "END_BLOCKING");

    lua_pushinteger(l, END_NEAR);
    lua_setglobal(l, "END_NEAR");

    lua_pushinteger(l, END_OBSTACLE);
    lua_setglobal(l, "END_OBSTACLE");

    lua_pushinteger(l, END_ERROR);
    lua_setglobal(l, "END_ERROR");

    lua_pushinteger(l, END_TIMER);
    lua_setglobal(l, "END_TIMER");

    lua_pushlightuserdata(l, HEXMOTORCONTROLLER_BASE);
    lua_setglobal(l, "hexmotor");

    lua_pushlightuserdata(l, ARMSMOTORCONTROLLER_BASE);
    lua_setglobal(l, "armmotor");

    lua_pushlightuserdata(l, &robot.angle_pid);
    lua_setglobal(l, "angle_pid");

    lua_pushlightuserdata(l, &robot.distance_pid);
    lua_setglobal(l, "distance_pid");

    lua_pushlightuserdata(l, &robot.right_arm.shoulder.pid);
    lua_setglobal(l, "right_shoulder_pid");

    lua_pushlightuserdata(l, &robot.right_arm.elbow.pid);
    lua_setglobal(l, "right_elbow_pid");

    lua_pushlightuserdata(l, &robot.right_arm.hand.pid);
    lua_setglobal(l, "right_hand_pid");

    lua_pushlightuserdata(l, &robot.right_arm.z_axis.pid);
    lua_setglobal(l, "right_z_axis_pid");

    lua_pushlightuserdata(l, &robot.left_arm.shoulder.pid);
    lua_setglobal(l, "left_shoulder_pid");

    lua_pushlightuserdata(l, &robot.left_arm.elbow.pid);
    lua_setglobal(l, "left_elbow_pid");

    lua_pushlightuserdata(l, &robot.left_arm.hand.pid);
    lua_setglobal(l, "left_hand_pid");

    lua_pushlightuserdata(l, &robot.left_arm.z_axis.pid);
    lua_setglobal(l, "left_z_axis_pid");
}

