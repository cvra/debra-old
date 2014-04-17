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

    int ret;
    do {
        trajectory_goto_forward_xy_abs(&robot.traj, 1000, COLOR_Y(600));
        ret = wait_traj_end(TRAJ_FLAGS_STD |END_NEAR);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    /*
    do {
        trajectory_goto_forward_xy_abs(&robot.traj, 1600, COLOR_Y(600));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    trajectory_goto_backward_xy_abs(&robot.traj, 1350, COLOR_Y(230));
    ret = wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);

    trajectory_goto_backward_xy_abs(&robot.traj, 1350, COLOR_Y(0));
    ret = wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);
    */

    do {
        trajectory_goto_xy_abs(&robot.traj, 1500, COLOR_Y(600));
        ret = wait_traj_end(TRAJ_FLAGS_STD |TRAJ_FLAGS_NEAR);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    do {
        trajectory_goto_xy_abs(&robot.traj, 1200, COLOR_Y(600));
        ret = wait_traj_end(TRAJ_FLAGS_STD |TRAJ_FLAGS_NEAR);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    do {
        trajectory_goto_xy_abs(&robot.traj, 1200, COLOR_Y(1000));
        ret = wait_traj_end(TRAJ_FLAGS_STD |TRAJ_FLAGS_NEAR);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    do {
        trajectory_goto_forward_xy_abs(&robot.traj, 1200, COLOR_Y(1600));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    do {
        trajectory_goto_forward_xy_abs(&robot.traj, 900, COLOR_Y(1600));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    do {
        trajectory_goto_xy_abs(&robot.traj, 1200, COLOR_Y(1000));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    do {
        trajectory_goto_xy_abs(&robot.traj, 400, COLOR_Y(600));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    do {
        trajectory_goto_forward_xy_abs(&robot.traj, 400, COLOR_Y(1050));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
        if (ret == END_TIMER)
            return;

        if (!TRAJ_SUCCESS(ret)) {
            strat_wait_ms(2000);
            printf("RET=%d\n", ret);
        }
    } while (!TRAJ_SUCCESS(ret));

    trajectory_d_rel(&robot.traj, 200);


end:
    pump_right_top(0);
    pump_right_bottom(0);

    pump_left_top(0);
    pump_left_bottom(0);

//    reset_arm_pos();
}


