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

void test_take(void)
{
    float sx, sy, sz;

    arm_trajectory_t traj;

    arm_t *arm;

    pump_left_top(1);
    pump_left_bottom(1);
    pump_right_top(1);
    pump_right_bottom(1);


    if (strat.color == RED) {
        arm = &robot.left_arm;
    }
    else {
        arm = &robot.right_arm;
    }


    arm_trajectory_init(&traj);

    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 900, COLOR_Y(1100), sz, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, 900, COLOR_Y(1100), 126, COORDINATE_TABLE, 1.);


    if (arm == &robot.left_arm) {
        arm_trajectory_append_point(&traj, 100, 50, 220, COORDINATE_ROBOT, 1.);
        arm_trajectory_append_point(&traj, 100, 50, 120, COORDINATE_ROBOT, 1.);
    } else {
        arm_trajectory_append_point(&traj, 100, -50, 220, COORDINATE_ROBOT, 1.);
        arm_trajectory_append_point(&traj, 100, -50, 120, COORDINATE_ROBOT, 1.);
    }

    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    while (!arm_trajectory_finished(&arm->trajectory));
}

void test_drop(void)
{
    pump_right_top(0);
    pump_right_bottom(0);

    pump_left_bottom(0);
    pump_left_top(0);
}

void take_1st_fire(void)
{
    float sx, sy, sz;

    arm_trajectory_t traj;

    arm_t *arm;


    if (strat.color == YELLOW) {
        arm = &robot.left_arm;
    }
    else {
        arm = &robot.right_arm;
    }

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 400, COLOR_Y(1020), 100, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, 400, COLOR_Y(1160), 100, COORDINATE_TABLE, .5);
    arm_trajectory_append_point(&traj, 400, COLOR_Y(1160), 30, COORDINATE_TABLE, .5);

    if (arm == &robot.left_arm) {
        arm_trajectory_append_point(&traj, 140, 70, 220, COORDINATE_ROBOT, 1.);
        arm_trajectory_append_point(&traj, 140, 70, 160, COORDINATE_ROBOT, 1.);
    } else {
        arm_trajectory_append_point(&traj, 140, -70, 220, COORDINATE_ROBOT, 1.);
        arm_trajectory_append_point(&traj, 140, -70, 160, COORDINATE_ROBOT, 1.);
    }

    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);
    while (!arm_trajectory_finished(&arm->trajectory));
}

void take_2nd_fire(void)
{
    float sx, sy, sz;

    arm_trajectory_t traj;

    arm_t *arm;
    if (strat.color == RED) {
        arm = &robot.left_arm;
    }
    else {
        arm = &robot.right_arm;
    }


    arm_trajectory_init(&traj);

    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);

    arm_trajectory_append_point(&traj, 900, COLOR_Y(1600), 100, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, 1050, COLOR_Y(1600), 100, COORDINATE_TABLE, .5);
    arm_trajectory_append_point(&traj, 1050, COLOR_Y(1600), 30, COORDINATE_TABLE, .5);

    arm_trajectory_append_point(&traj, 200, 0, 120, COORDINATE_ARM, .5);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);
    while (!arm_trajectory_finished(&arm->trajectory));
}

void reset_arm_pos(void)
{
    float sx, sy, sz;

    arm_trajectory_t traj;

    arm_trajectory_init(&traj);
    arm_get_position(&robot.right_arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 220, COORDINATE_ARM, 1.);
    arm_do_trajectory(&robot.right_arm, &traj);
    arm_trajectory_delete(&traj);

    arm_trajectory_init(&traj);
    arm_get_position(&robot.left_arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, 0, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, 0, 220, COORDINATE_ARM, 1.);
    arm_do_trajectory(&robot.left_arm, &traj);
    arm_trajectory_delete(&traj);
}


void strat_begin(void) {


    trajectory_set_acc(&robot.traj,
            acc_mm2imp(&robot.traj, 1300),
            acc_rd2imp(&robot.traj, 10));


    trajectory_set_acc(&robot.traj,
            acc_mm2imp(&robot.traj, 800),
            acc_rd2imp(&robot.traj, 4.85));

    /* Starts the game timer. */
    cvra_wait_starter_pull(PIO_BASE);
    strat_timer_reset();

    int ret;

    do {
        //trajectory_goto_xy_abs(&robot.traj, 615, COLOR_Y(1175));
        trajectory_goto_xy_abs(&robot.traj, 615, COLOR_Y(1000));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret))
            strat_wait_ms(2000);
    } while (!TRAJ_SUCCESS(ret));

    trajectory_a_abs(&robot.traj, COLOR_Y(90));

    wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);

    take_1st_fire();

    do {
        trajectory_goto_xy_abs(&robot.traj, 615, COLOR_Y(1175));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret))
            strat_wait_ms(2000);
    } while (!TRAJ_SUCCESS(ret));


    test_take();


    do {
        trajectory_goto_xy_abs(&robot.traj, 272, COLOR_Y(2000-272));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret))
            strat_wait_ms(2000);

    } while (!TRAJ_SUCCESS(ret));

    trajectory_turnto_xy(&robot.traj, 0, COLOR_Y(2000));
    ret = wait_traj_end(TRAJ_FLAGS_STD);


    test_drop();


    trajectory_goto_xy_abs(&robot.traj, 900, COLOR_Y(1500));

    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if (!TRAJ_SUCCESS(ret))
        goto end;

    trajectory_a_abs(&robot.traj, COLOR_Y(0));
    wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);

    //take_2nd_fire();


    trajectory_goto_xy_abs(&robot.traj, 1250, COLOR_Y(1050));

    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if (!TRAJ_SUCCESS(ret))
        goto end;

    trajectory_a_abs(&robot.traj, COLOR_Y(-90));
    wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);

    /* After the game, shutdown the pumps. */

end:
    pump_right_top(0);
    pump_right_bottom(0);

    pump_left_top(0);
    pump_left_bottom(0);

    reset_arm_pos();
}


