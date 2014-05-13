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

void reset_arm_pos(void)
{
    float sx, sy, sz;

    arm_trajectory_t traj;

    arm_trajectory_init(&traj);
    arm_get_position(&robot.right_arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 191, COORDINATE_ARM, 1.);
    arm_do_trajectory(&robot.right_arm, &traj);
    arm_trajectory_delete(&traj);

    arm_trajectory_init(&traj);
    arm_get_position(&robot.left_arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, 0, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, 0, 191, COORDINATE_ARM, 1.);
    arm_do_trajectory(&robot.left_arm, &traj);
    arm_trajectory_delete(&traj);
}

void setup_arm_pos(void)
{
    float sx, sy, sz;

    arm_trajectory_t traj;

    arm_trajectory_init(&traj);
    arm_get_position(&robot.right_arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 130, 0, sz, COORDINATE_ROBOT, .5);
    arm_trajectory_append_point(&traj, 130, 0, 95, COORDINATE_ROBOT, .5);
    arm_do_trajectory(&robot.right_arm, &traj);
    arm_trajectory_delete(&traj);
}

void do_first_fire(void)
{
    pump_left_bottom(1);
    pump_left_top(1);
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
    arm_trajectory_append_point(&traj, -40, COLOR_Y(800), sz, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, -40, COLOR_Y(800), 85, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, 125, COLOR_Y(800), 85, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, 125, COLOR_Y(800), 32, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, 125, COLOR_Y(800), 150, COORDINATE_TABLE, 1.);

    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    strat_wait_ms(2000);
    trajectory_d_rel(&robot.traj, 120);
    while(!arm_trajectory_finished(&arm->trajectory));
}

void strat_begin(void) {


    /*trajectory_set_acc(&robot.traj,
            acc_mm2imp(&robot.traj, 1300),
            acc_rd2imp(&robot.traj, 10));


    trajectory_set_speed(&robot.traj,
            speed_mm2imp(&robot.traj, 800),
            speed_rd2imp(&robot.traj, 4.85));*/

    bd_set_thresholds(&robot.distance_bd, 6000, 1);


    /* Starts the game timer. */
    cvra_wait_starter_pull(PIO_BASE);
    strat_timer_reset();
    //setup_arm_pos();
//    reset_arm_pos();
//    trajectory_goto_forward_xy_abs(&robot.traj, 1500, COLOR_Y(690));
//
    do_first_fire();
    strat_wait_ms(3000);
    reset_arm_pos();
}


