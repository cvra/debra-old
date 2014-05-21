#include <stdio.h>
#include <fcntl.h>

#include <cvra_dc.h>
#include <aversive/error.h>
#include <cvra_beacon.h>

#include "strat.h"
#include "strat_job.h"
#include "arm.h"
#include "arm_trajectories.h"
#include "strat_utils.h"
#include "2wheels/trajectory_manager.h"

#include "cvra_cs.h"

#define FIRE_HEIGHT 30

/** Switches from table coordinate to arm coordinate. */
void strat_block_hand(arm_t *arm, int angle)
{
    float sx, sy, sz;
    arm_trajectory_t traj;

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, angle);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);
}

void pump_left_bottom(int v)
{
    cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE,6, v*500);
}

void pump_left_top(int v)
{
    cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE,3, v*500);
}

void pump_right_bottom(int v)
{
    cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE, 4, v*500);
}

void pump_right_top(int v)
{
    cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE,7, v*500);
}

arm_t *strat_opposite_arm(arm_t *arm)
{
    if (arm == &robot.left_arm)
        return &robot.right_arm;
    else
        return &robot.left_arm;

}
void setup_arm_pos(void)
{
    float sx, sy, sz;

    arm_trajectory_t traj;
    arm_t *arm;


    if (strat.color == YELLOW)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 105, COORDINATE_ARM, .5);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);
    while (!arm_trajectory_finished(&arm->trajectory));
    arm_shutdown(arm);
}

void arm_position_navigation(arm_t *arm)
{
    arm_trajectory_t traj;
    float sx, sy, sz;
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    if (arm == &robot.left_arm)
        arm_trajectory_append_point(&traj, 100, 40, 145, COORDINATE_ARM, .5);
    else
        arm_trajectory_append_point(&traj, 100, -40, 145, COORDINATE_ARM, .5);

    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);
}

void do_first_fire(void)
{
    arm_trajectory_t traj;
    arm_t *arm;
    float sx, sy, sz;
    if (strat.color == YELLOW)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;


    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
//    arm_trajectory_append_point(&traj, -40, COLOR_Y(800), sz, COORDINATE_TABLE, .5);
    arm_trajectory_append_point(&traj, -40, COLOR_Y(800), 85, COORDINATE_TABLE, .5);
    arm_trajectory_append_point(&traj, 40, COLOR_Y(800), 85, COORDINATE_TABLE, .2);

    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    while (!arm_trajectory_finished(&arm->trajectory));
    arm_position_navigation(arm);
}

void grab_stack(void)
{
    // takes a fire stack zu mitnehmen
    arm_trajectory_t traj;
    arm_t *arm;
    float sx, sy, sz;
    if (strat.color == RED)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);

    if (strat.color == YELLOW)
        arm_trajectory_append_point(&traj, 5, -310, 1, COORDINATE_ROBOT, 1.);
    else
        arm_trajectory_append_point(&traj, 5, 310, 1, COORDINATE_ROBOT, 1.);

    arm_trajectory_set_hand_angle(&traj, 90);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);
}

void empty_fire_pit(int stack_x, int stack_y)
{
    arm_trajectory_t traj;
    arm_t *arm;
    float sx, sy, sz;
    int i;
    int stack_height = 124;

    if (strat.color == RED)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);

    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 90);
    arm_trajectory_append_point(&traj, sx, sy, 200, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 0);

    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    while (!arm_trajectory_finished(&arm->trajectory));

    //arm->shoulder_mode = SHOULDER_FRONT;

    trajectory_only_a_rel(&robot.traj, COLOR_A(-45));
    wait_traj_end(TRAJ_FLAGS_STD);


    pump_right_bottom(1);
    pump_right_top(1);
    pump_left_bottom(1);
    pump_left_top(1);

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);

    /* First triangle */
    arm_trajectory_append_point(&traj, stack_x, stack_y, 180, COORDINATE_TABLE, 1.);
    arm_trajectory_set_hand_angle(&traj, 0);
    arm_trajectory_append_point(&traj, stack_x, stack_y, stack_height, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, stack_x, stack_y, 180, COORDINATE_TABLE, 1.);
    stack_height -= FIRE_HEIGHT;

    /* Second triangle */
    arm_trajectory_append_point(&traj, stack_x, stack_y, 180, COORDINATE_TABLE, 1.);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, stack_x, stack_y, stack_height, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, stack_x, stack_y, 180, COORDINATE_TABLE, 1.);
    stack_height -= FIRE_HEIGHT;

    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);
    while (!arm_trajectory_finished(&arm->trajectory));

    strat_block_hand(arm, 180);



    /* Switches arms for second part of movement. */
    if (arm == &robot.right_arm)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    trajectory_only_a_rel(&robot.traj, COLOR_A(-45));
    wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);

    arm_trajectory_append_point(&traj, stack_x, stack_y, 180, COORDINATE_TABLE, 1.);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, stack_x, stack_y, stack_height, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, stack_x, stack_y, 180, COORDINATE_TABLE, 1.);
    stack_height -= FIRE_HEIGHT;
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    while (!arm_trajectory_finished(&arm->trajectory));
    strat_block_hand(arm, 180);
}

void do_dropzone_corner(void)
{
    arm_trajectory_t traj;
    arm_t *arm;
    float sx, sy, sz;

    if (strat.color == RED)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;


    trajectory_turnto_xy(&robot.traj, 0, COLOR_Y(2000));
    wait_traj_end(END_TRAJ);
    trajectory_d_rel(&robot.traj, 40);

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 80, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 0);
    arm_trajectory_append_point(&traj, 120, COLOR_Y(2000-50), 80, COORDINATE_TABLE, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    arm = strat_opposite_arm(arm);
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, sx, sy, 80, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 50, COLOR_Y(2000-120), 80, COORDINATE_TABLE, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    while ((!arm_trajectory_finished(&robot.left_arm.trajectory)) ||
          (!arm_trajectory_finished(&robot.right_arm.trajectory)));


    pump_left_bottom(0);
    pump_right_bottom(0);
    pump_right_top(0);
    pump_left_top(0);


}

void strat_begin(void)
{
    bd_set_thresholds(&robot.distance_bd, 6000, 1);

    setup_arm_pos();


    /* Starts the game timer. */
    cvra_wait_starter_pull(PIO_BASE);
    strat_timer_reset();

    do_first_fire();

    trajectory_goto_forward_xy_abs(&robot.traj, 690, COLOR_Y(1050));
    while (position_get_x_float(&robot.pos) < 500);
    grab_stack();
    wait_traj_end(END_TRAJ);

    strat_set_speed(SLOW);

    trajectory_only_a_abs(&robot.traj, COLOR_A(80));
    wait_traj_end(END_TRAJ);

    trajectory_goto_forward_xy_abs(&robot.traj, 300, COLOR_Y(1700));
    wait_traj_end(END_TRAJ);


    strat_set_speed(FUCKING_SLOW);
    empty_fire_pit(430, COLOR_Y(2000 - 160));
    do_dropzone_corner();

    strat_set_speed(FAST);
}


