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

    pump_right_top(1);
    pump_right_bottom(1);
    arm_trajectory_init(&traj);

    arm_get_position(&robot.right_arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 900, COLOR_Y(1100), sz, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, 900, COLOR_Y(1100), 126, COORDINATE_TABLE, 1.);
    arm_trajectory_append_point(&traj, 170, 0, 220, COORDINATE_ROBOT, 1.);
    arm_trajectory_append_point(&traj, 170, 0, 150, COORDINATE_ROBOT, 1.);

    arm_do_trajectory(&robot.right_arm, &traj);

    printf("%.1f;%.1f;%.1f\n", sx, sy, sz); 

    while (!arm_trajectory_finished(&robot.right_arm.trajectory));

    arm_trajectory_delete(&traj);
}


void strat_begin(void) {

    /* Starts the game timer. */
    strat_timer_reset();

    /* XXX 2014 code goes there. */

    trajectory_goto_forward_xy_abs(&robot.traj, 615, COLOR_Y(1175));
    wait_traj_end(TRAJ_FLAGS_STD);
    test_take();
    trajectory_goto_forward_xy_abs(&robot.traj, 272, COLOR_Y(2000-272));
    wait_traj_end(TRAJ_FLAGS_STD);
    trajectory_turnto_xy(&robot.traj, 0, COLOR_Y(2000));
    wait_traj_end(TRAJ_FLAGS_STD);

    /* After the game, shutdown the pumps. */

    pump_right_top(0);
    pump_right_bottom(0);
}


