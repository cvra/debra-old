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
    cvra_dc_set_pwm((int *)HEXMOTORCONTROLLER_BASE,3, v*500);
}

void pump_left_top(int v)
{
    cvra_dc_set_pwm((int *)HEXMOTORCONTROLLER_BASE,6, v*500);
}

void pump_right_bottom(int v)
{
    cvra_dc_set_pwm((int *)HEXMOTORCONTROLLER_BASE, 4, v*500);
}

void pump_right_top(int v)
{
    cvra_dc_set_pwm((int *)HEXMOTORCONTROLLER_BASE,7, v*500);
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
        arm_trajectory_append_point(&traj, 130, 40, 80, COORDINATE_ARM, .5);
    else
        arm_trajectory_append_point(&traj, 130, -40, 80, COORDINATE_ARM, .5);

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
    arm_trajectory_append_point(&traj, stack_x, stack_y, stack_height-2, COORDINATE_TABLE, 1.);
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
    trajectory_d_rel(&robot.traj, 50);

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 80, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 0);
    arm_trajectory_append_point(&traj, 100, COLOR_Y(2000-30), 80, COORDINATE_TABLE, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    arm = strat_opposite_arm(arm);
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, sx, sy, 80, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 30, COLOR_Y(2000-100), 80, COORDINATE_TABLE, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    while ((!arm_trajectory_finished(&robot.left_arm.trajectory)) ||
          (!arm_trajectory_finished(&robot.right_arm.trajectory)));

    if (strat.color == RED) {
        pump_left_bottom(0);
        pump_right_top(0);
    } else {
        pump_left_top(0);
        pump_right_bottom(0);
    }
}


/** Pass a fire from src arm to dest.
 *
 * @warning Calling function should make sure the arm is in a safe operating
 * position.
 *
 * @note This function sends the fire from bottom to bottom.
 */
void strat_pass_fire(arm_t *dest, arm_t *src)
{
    const int delta_z = 110;
    const int src_z = 15;
    arm_trajectory_t traj;
    float sx, sy, sz;
    float dx, dy, dz;

    if (dest == src) {
        return;
    }

    arm_trajectory_init(&traj);
    arm_get_position(src, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, .5);

    arm_trajectory_append_point(&traj, sx, sy, 150, COORDINATE_ARM, .5);

    arm_trajectory_append_point(&traj, sx, sy, 150, COORDINATE_ARM, 2.);
    arm_trajectory_set_hand_angle(&traj, 180);

    arm_trajectory_append_point(&traj, 180, 0, 150, COORDINATE_ROBOT, 2.);
    arm_trajectory_append_point(&traj, 180, 0, src_z, COORDINATE_ROBOT, .5);
    arm_do_trajectory(src, &traj);
    while (!arm_trajectory_finished(&src->trajectory)) {}
    arm_trajectory_delete(&traj);

    if (dest == &robot.left_arm) {
        pump_left_bottom(1);
    } else {
        pump_right_bottom(1);
    }

    arm_trajectory_init(&traj);
    arm_get_position(dest, &dx, &dy, &dz);
    arm_trajectory_append_point(&traj, dx, dy, dz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, dx, dy, src_z + delta_z + 15, COORDINATE_ARM, .5);
    arm_trajectory_append_point(&traj, 180, 0, src_z + delta_z + 15, COORDINATE_ROBOT, 2.);
    arm_trajectory_append_point(&traj, 180, 0, src_z + delta_z, COORDINATE_ROBOT, .5);
    arm_do_trajectory(dest, &traj);
    while (!arm_trajectory_finished(&dest->trajectory)) {}
    arm_trajectory_delete(&traj);

    if (src == &robot.left_arm) {
        pump_left_bottom(0);
    } else {
        pump_right_bottom(0);
    }

    strat_wait_ms(500);

    arm_trajectory_init(&traj);
    arm_get_position(dest, &dx, &dy, &dz);
    arm_trajectory_append_point(&traj, dx, dy, dz + 30, COORDINATE_ARM, 1.);
    arm_do_trajectory(dest, &traj);

    while (!arm_trajectory_finished(&dest->trajectory)) {}
    arm_trajectory_delete(&traj);


    arm_trajectory_init(&traj);
    arm_get_position(src, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, .5);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, .5);
    arm_trajectory_set_hand_angle(&traj, 0);
    arm_do_trajectory(src, &traj);
    while (!arm_trajectory_finished(&src->trajectory)) {}
    arm_trajectory_delete(&traj);
}

void exchange_fires(void)
{
    arm_trajectory_t traj;
    arm_t *arm;
    float sx, sy, sz;
    int ret;

    if (strat.color == RED)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    do {
        trajectory_d_rel(&robot.traj, -300);
        ret = wait_traj_end(TRAJ_FLAGS_STD);
    } while (!TRAJ_SUCCESS(ret));

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 5, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, 0, 5, COORDINATE_ROBOT, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    arm = strat_opposite_arm(arm);

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 180);
    arm_trajectory_append_point(&traj, sx, sy, 135, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 135, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 0);

    arm_trajectory_append_point(&traj, 200, 0, 135, COORDINATE_ROBOT, 1.);

    // XXX Fix for weird mechanical issues
    if (strat.color == RED) {
        arm_trajectory_append_point(&traj, 200, 0, 117, COORDINATE_ROBOT, 1.);
    } else {
        arm_trajectory_append_point(&traj, 200, 0, 110, COORDINATE_ROBOT, 1.);
    }
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    if (strat.color == RED)
        pump_right_bottom(1);
    else
        pump_left_bottom(1);

    while ((!arm_trajectory_finished(&robot.left_arm.trajectory)) ||
          (!arm_trajectory_finished(&robot.right_arm.trajectory)));

    if (strat.color == RED)
        pump_left_top(0);
    else
        pump_right_top(0);

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 135, COORDINATE_ARM, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    arm = strat_opposite_arm(arm);
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, 0, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 200, 0, 160, COORDINATE_ARM, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    while (!arm_trajectory_finished(&arm->trajectory));

    arm = strat_opposite_arm(arm);
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, sx, sy, 30+30+15, COORDINATE_ARM, .5);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);
    while (!arm_trajectory_finished(&arm->trajectory));
    trajectory_d_rel( &robot.traj, 300);
    wait_traj_end(END_TRAJ);

    pump_left_bottom(0);
    pump_right_bottom(0);
    strat_wait_ms(400);

    trajectory_d_rel( &robot.traj, -100);
    wait_traj_end(TRAJ_FLAGS_STD);
}

void do_fire_middle_table(void)
{
    arm_trajectory_t traj;
    arm_t *arm;
    float sx, sy, sz;

    const int arm_x = 135;
    int ret;



    do {
        trajectory_goto_backward_xy_abs(&robot.traj, 900, COLOR_Y(1400));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
    } while (!TRAJ_SUCCESS(ret));

    if (strat.color == RED) {
        arm = &robot.left_arm;
    } else {
        arm = &robot.right_arm;
    }

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 1000, COLOR_Y(1600), 150, COORDINATE_TABLE, .5);
    arm_trajectory_append_point(&traj, 1000, COLOR_Y(1600), 70, COORDINATE_TABLE, .5);
    arm_trajectory_append_point(&traj, 800, COLOR_Y(1600), 70, COORDINATE_TABLE, .5);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    while (!arm_trajectory_finished(&arm->trajectory));


    do {
        trajectory_goto_xy_abs(&robot.traj, 1500, COLOR_Y(1600));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
    } while (!TRAJ_SUCCESS(ret));

    if (strat.color == RED) {
        pump_left_bottom(1);
        pump_left_top(1);
    } else {
        pump_right_bottom(1);
        pump_right_top(1);
    }

    arm = &robot.left_arm;
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, -arm_x, 200, sz, COORDINATE_ROBOT, .5);
    arm_trajectory_set_hand_angle(&traj, 90);
    arm_trajectory_append_point(&traj, -arm_x, 200, 20, COORDINATE_ROBOT, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    arm = &robot.right_arm;
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, -arm_x, -200, sz, COORDINATE_ROBOT, .5);
    arm_trajectory_set_hand_angle(&traj, 90);
    arm_trajectory_append_point(&traj, -arm_x, -200, 20, COORDINATE_ROBOT, 1.);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    do {
        trajectory_turnto_xy_behind(&robot.traj, 1500, COLOR_Y(2000));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
    } while (!TRAJ_SUCCESS(ret));

    trajectory_goto_backward_xy_abs(&robot.traj, 1500, COLOR_Y(2000-arm_x-38));
    wait_traj_end(END_TRAJ|END_BLOCKING);

    strat_wait_ms(500);

    arm = &robot.left_arm;
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 90);
    arm_trajectory_append_point(&traj, sx, sy, sz+10, COORDINATE_ARM, .1);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    arm = &robot.left_arm;
    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, 1.);
    arm_trajectory_set_hand_angle(&traj, 90);
    arm_trajectory_append_point(&traj, sx, sy, sz+10, COORDINATE_ARM, .1);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    trajectory_d_rel(&robot.traj, 200);
    wait_traj_end(TRAJ_FLAGS_STD);

    if (strat.color == RED)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    arm_trajectory_init(&traj);
    arm_get_position(arm, &sx, &sy, &sz);
    arm_trajectory_append_point(&traj, sx, sy, sz, COORDINATE_ARM, .5);
    arm_trajectory_set_hand_angle(&traj, 90);
    arm_trajectory_append_point(&traj, sx, sy, 75, COORDINATE_ARM, .5);
    arm_trajectory_set_hand_angle(&traj, 0);
    arm_trajectory_append_point(&traj, 120, 0, 75, COORDINATE_ARM, .5);
    arm_trajectory_append_point(&traj, 200, 0, 75, COORDINATE_ROBOT, .5);
    arm_do_trajectory(arm, &traj);
    arm_trajectory_delete(&traj);

    do {
        trajectory_goto_forward_xy_abs(&robot.traj, 1500,COLOR_Y(1000+150+200));
        wait_traj_end(TRAJ_FLAGS_STD);
    } while (ret != END_BLOCKING && ret != END_TRAJ);

    while (!arm_trajectory_finished(&arm->trajectory)) {}

    OSTimeDlyHMSM(0,0,1,0);

    pump_left_bottom(0);
    pump_right_bottom(0);
    pump_left_top(0);
    pump_right_top(0);
}

void strat_begin(void)
{
    int stack_grabbed = 0;
    int ret;
    bd_set_thresholds(&robot.distance_bd, 6000, 1);

    setup_arm_pos();

    /* Starts the game timer. */
    cvra_wait_starter_pull(PIO_BASE);
    strat_timer_reset();

    strat_set_speed(FAST);

    do_first_fire();

    trajectory_goto_forward_xy_abs(&robot.traj, 690, COLOR_Y(1050));
    while (position_get_x_float(&robot.pos) < 500) {
        test_traj_end(TRAJ_FLAGS_STD);
        trajectory_goto_forward_xy_abs(&robot.traj, 690, COLOR_Y(1050));
    }

    grab_stack();

    do {
        trajectory_goto_forward_xy_abs(&robot.traj, 690, COLOR_Y(1050));
        ret = wait_traj_end(TRAJ_FLAGS_STD);
    } while (!TRAJ_SUCCESS(ret));

    strat_set_speed(SLOW);

    trajectory_only_a_abs(&robot.traj, COLOR_A(80));
    wait_traj_end(TRAJ_FLAGS_STD);

    do {
        trajectory_goto_forward_xy_abs(&robot.traj, 300, COLOR_Y(1700));
        ret = wait_traj_end(END_TRAJ |END_OBSTACLE);
    } while (!TRAJ_SUCCESS(ret));

    strat_set_speed(FUCKING_SLOW);
    empty_fire_pit(430, COLOR_Y(2000 - 160));
    do_dropzone_corner();

    strat_set_speed(FAST);
    exchange_fires();

    do_fire_middle_table();

    arm_position_navigation(&robot.left_arm);
    arm_position_navigation(&robot.right_arm);

    while (!arm_trajectory_finished(&robot.left_arm.trajectory));

    trajectory_only_a_rel(&robot.traj, 180);
    wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);

    pump_left_bottom(0);
    pump_right_bottom(0);
    pump_left_top(0);
    pump_right_top(0);
}


