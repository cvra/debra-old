#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <2wheels/trajectory_manager_utils.h>
#include <scheduler.h>
#include <aversive/error.h>
#include "arm.h"
#include "arm_interpolators.h"
#include <stdio.h>
#include "strat_job.h"
#include <cvra_beacon.h>
#include "strat_utils.h"

struct strat_info strat;


/** Increments the match timer, called every second. */
static void increment_timer(__attribute__((unused))void *data) {
    printf("%d\n", strat.time);
    strat.time++;
}

void left_arm_take_glass(int glass_index) {
    arm_trajectory_t traj;
    float x, y, z;

    WARNING(0, "taking glass %d at (%.1f;%.1f)", glass_index, strat.glasses[glass_index].pos.x, strat.glasses[glass_index].pos.y); 

    arm_get_position(&robot.left_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 

    arm_interpolator_append_point(&traj, -28, -66, z, COORDINATE_ARM, 3.);
    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y-40), 197, COORDINATE_TABLE, 1., 135, 160);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y-40), 15, COORDINATE_TABLE, 2., 135, 160);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x, 
                                        COLOR_Y(strat.glasses[glass_index].pos.y+40), 15, COORDINATE_TABLE, 2., 135, 100);

    arm_interpolator_append_point(&traj, 150, -150, 210, COORDINATE_ARM, 2.);
    arm_interpolator_append_point(&traj, -28, -66, 210, COORDINATE_ARM, 2.);


    arm_execute_movement(&robot.left_arm, &traj);
}


void right_arm_take_glass(int glass_index) {
    arm_trajectory_t traj;
    float x, y, z;

    arm_get_position(&robot.right_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 

    arm_interpolator_append_point(&traj, -28, 66, z, COORDINATE_ARM, 3.);
    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y+40), 197, COORDINATE_TABLE, 1., 135, 160);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y+40), 15, COORDINATE_TABLE, 2., 135, 160);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x, 
                                        COLOR_Y(strat.glasses[glass_index].pos.y-40), 15, COORDINATE_TABLE, 2., 135, 100);

    
    arm_interpolator_append_point(&traj, 150, 150, 210, COORDINATE_ARM, 2.);
    arm_interpolator_append_point(&traj, -28, 66, 210, COORDINATE_ARM, 2.);

    arm_execute_movement(&robot.right_arm, &traj);
}

void strat_degage_bras(void) {
    arm_trajectory_t traj;
    float x, y, z;

    arm_get_position(&robot.right_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used     
    arm_interpolator_append_point(&traj, x+20, y, z, COORDINATE_ARM, .3); // duration not used     
    arm_execute_movement(&robot.right_arm, &traj);

    arm_get_position(&robot.left_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used     
    arm_interpolator_append_point(&traj, x+20, y, z, COORDINATE_ARM, .3); // duration not used     
    arm_execute_movement(&robot.left_arm, &traj);
    
    while(!arm_trajectory_finished(&robot.left_arm) || !arm_trajectory_finished(&robot.right_arm));
}

int strat_do_near_glasses(void) {
    int ret;
    robot.left_arm.shoulder_mode = SHOULDER_BACK;
    robot.right_arm.shoulder_mode = SHOULDER_BACK;

    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[1].pos.x+95, COLOR_Y(strat.glasses[2].pos.y));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret))
        return ret;


    trajectory_a_abs(&robot.traj, 180);
    ret = wait_traj_end(TRAJ_FLAGS_STD);    // TODO : necessary? the variable is not used later on...


    left_pump(1);
    right_pump(1);

   // trajectory_a_abs(&robot.traj, 180);
    //wait_traj_end(TRAJ_FLAGS_STD);

    if(strat.color == BLUE) {
        left_arm_take_glass(0);
        right_arm_take_glass(1);
    }
    else {
        left_arm_take_glass(1);
        right_arm_take_glass(0);
    }

    while(!arm_trajectory_finished(&robot.left_arm) || !arm_trajectory_finished(&robot.right_arm));


    left_pump(-1);
    right_pump(-1);


    strat_degage_bras();

    return END_TRAJ;
}

void strat_wait_ms(int ms) {
    int32_t time = uptime_get();
    while(uptime_get() < time + ms*1000);
}


void strat_do_far_glasses(void) {
    int ret;
    robot.left_arm.shoulder_mode = SHOULDER_BACK;
    robot.right_arm.shoulder_mode = SHOULDER_BACK;

    trajectory_goto_backward_xy_abs(&robot.traj, strat.glasses[3].pos.x+95, COLOR_Y(strat.glasses[2].pos.y));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret))
        return ret;

    trajectory_a_abs(&robot.traj, 180);
    wait_traj_end(TRAJ_FLAGS_STD);


    left_pump(1);
    right_pump(1);


    if(strat.color == BLUE) {
        left_arm_take_glass(3);
        right_arm_take_glass(4);
    }
    else {
        left_arm_take_glass(4);
        right_arm_take_glass(3);
    }

    while(!arm_trajectory_finished(&robot.right_arm) || !arm_trajectory_finished(&robot.left_arm));


    left_pump(-1);
    right_pump(-1);

    strat_degage_bras();

    return END_TRAJ;
}

/** @brief Take the first two glasses.
 *
 * This function takes the first two glasses on the correct side.
 * @todo Test the starting coordinates.
 */
static int strat_do_first_glasses(void) {
    WARNING(E_STRAT, "Doing first glasses."); 

    int ret;

    strat_open_servo(LEFT);
    strat_open_servo(RIGHT);


retry1:
    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[2].pos.x, COLOR_Y(strat.glasses[2].pos.y)-50);

    ret = wait_traj_end(TRAJ_FLAGS_NEAR | END_OBSTACLE);

    if(!(TRAJ_SUCCESS(ret))) {
        if(ret == END_OBSTACLE) {
            goto retry1;
        }
        else
            return ret;
    }



retry2:
    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[5].pos.x, COLOR_Y(strat.glasses[5].pos.y)+50);
    ret = wait_traj_end(TRAJ_FLAGS_NEAR | END_OBSTACLE);


    if(!(TRAJ_SUCCESS(ret))) {
        if(ret == END_OBSTACLE) {
            goto retry2;
        }
        else
            return ret;
    }



    trajectory_d_rel(&robot.traj, 100);
    strat_wait_ms(50);
    strat_close_servo(LEFT);
    strat_close_servo(RIGHT);
    wait_traj_end(TRAJ_FLAGS_STD | END_OBSTACLE);

    return END_TRAJ;
}

void strat_do_candles(void) {
    int i;
    int ret;
    float robot_x, robot_y;
    const float waypoint_radius = 650; // distance from center of cake to robot waypoints
    // we do our candle first

    robot_x = cos(strat.candles[10].angle) * (waypoint_radius) + 1500;
    robot_y = sin(strat.candles[10].angle) * (waypoint_radius);

    // replace with goto_avoid
    trajectory_goto_xy_abs(&robot.traj, robot_x, COLOR_Y(robot_y));
    ret = wait_traj_end(TRAJ_FLAGS_STD);
    trajectory_a_abs(&robot.traj, COLOR_A(DEG(strat.candles[10].angle) - 90)); // XXX check red
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    trajectory_circle_rel(&robot.traj, 1500, COLOR_Y(0), waypoint_radius, 90, TRIGO | FORWARD);
    while(position_get_x_s16(&robot.pos < 2000));
    trajectory_stop(&robot.traj);


/*    for(i=10;i>=1;i--) {
        // we do the candles backward.
        robot_x = cos(strat.candles[i].angle) * waypoint_radius + 1500;
        robot_y = sin(strat.candles[i].angle) * waypoint_radius;

        // replace with goto_avoid
        trajectory_goto_backward_xy_abs(&robot.traj, robot_x, COLOR_Y(robot_y));
        ret = wait_traj_end(END_TRAJ);

        // test moar waypoints
    }

    ret = wait_traj_end(END_TRAJ);
    */



}

void strat_job_do_gift(void *param) {
    int gift_index = (int)param;
    WARNING(0, "Doing gift %d from job pool.", gift_index); 
}

int strat_do_gifts(void *dummy) {
    int ret;
    arm_t *arm;
    float x, y, z;

    arm_trajectory_t traj;

    const float height = 100; // height of attack
    const float depth = 40; // the length to go out of the table
    int i;


    if(strat.color == RED)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    arm->shoulder_mode = SHOULDER_FRONT;

    for(i=0;i<4;i++)
        strat.gifts[i].last_try_time = strat.time;

    
    arm_get_position(arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 1.); // duration not used 
    if(strat.color == BLUE) {
        arm_interpolator_append_point(&traj, x, y+30, z, COORDINATE_ARM, .2); 
        arm_interpolator_append_point(&traj, 150, y+30, z, COORDINATE_ARM, 1); 
    }
    else {
        arm_interpolator_append_point(&traj, x, y-30, z, COORDINATE_ARM, .1); 
        arm_interpolator_append_point(&traj, 150, y-30, z, COORDINATE_ARM, 1); 
    }
    arm_interpolator_append_point_with_length(&traj, 100, 10, z, COORDINATE_ARM, 1., 135, 95); 
    arm_interpolator_append_point_with_length(&traj, 100, 10, height, COORDINATE_ARM, 1., 135, 95); //XXX change for red
    arm_execute_movement(arm, &traj);


    while(!arm_trajectory_finished(arm));

    for(i=0;i<4;i++) {

        trajectory_goto_forward_xy_abs(&robot.traj, strat.gifts[i].x-50, COLOR_Y(2000-250));
        ret = wait_traj_end(TRAJ_FLAGS_STD);        // TODO : is this line really necessary when a copy of it appears 4 lines below?

        if(i==0) {
            trajectory_a_abs(&robot.traj, 0);
            ret = wait_traj_end(TRAJ_FLAGS_STD);
        }

        arm_trajectory_init(&traj);

        arm_interpolator_append_point_with_length(&traj, 100, 10, height, COORDINATE_ARM, 1., 135, 95); 
        arm_interpolator_append_point_with_length(&traj, strat.gifts[i].x-25,
                COLOR_Y(2000+depth), height, COORDINATE_TABLE, 0.4, 135, 85); 
        arm_interpolator_append_point_with_length(&traj, 100, 10, height, COORDINATE_ARM, 0.4, 135, 95); 

        arm_execute_movement(arm, &traj);

        while(!arm_trajectory_finished(arm));
    }

    return 0;
}

void strat_set_objects(void) {
    memset(&strat.glasses, 0, sizeof(glass_t)*12);
    memset(&strat.gifts, 0, sizeof(gift_t)*4);

    /* Init gifts position. */ 
    strat.gifts[0].x = 525; /* middle of the gift. */
    strat.gifts[1].x = 1125;
    strat.gifts[2].x = 1725;
    strat.gifts[3].x = 2325;

    /* Init glasses positions. */
    strat.glasses[0].pos.x = 900; strat.glasses[0].pos.y = (1450);
    strat.glasses[1].pos.x = 900; strat.glasses[1].pos.y = (950);
    strat.glasses[2].pos.x = 1050; strat.glasses[2].pos.y = (1200);

    /*XXX Not sure about coordinates of 3 and 4. */
    strat.glasses[3].pos.x = 1200; strat.glasses[3].pos.y = (1450);
    strat.glasses[4].pos.x = 1200; strat.glasses[4].pos.y = (950);
    strat.glasses[5].pos.x = 1350; strat.glasses[5].pos.y = (1200);
    strat.glasses[6].pos.x = 1650; strat.glasses[6].pos.y = (1300);
    strat.glasses[7].pos.x = 1800; strat.glasses[7].pos.y = (1550);
    strat.glasses[8].pos.x = 1800; strat.glasses[8].pos.y = (1050);
    strat.glasses[9].pos.x = 1950; strat.glasses[9].pos.y = (1300);
    strat.glasses[10].pos.x = 2100; strat.glasses[10].pos.y = (1550);
    strat.glasses[11].pos.x = 2100; strat.glasses[11].pos.y = (1050);

    float alpha = 7.5;

    int i;
    for(i=0;i<12;i++) {
        strat.candles[i].angle = alpha * 3.14 / 180.;
        strat.candles[i].done = 0;
        strat.candles[i].color = CANDLE_OPP;
        alpha = alpha + 15.;
    }

    strat.candles[11].color = CANDLE_OUR;

    /* XXX Change if we reach finals. */
    strat.candles[4].color = CANDLE_WHITE;
    strat.candles[5].color = CANDLE_WHITE;
    strat.candles[6].color = CANDLE_WHITE;
    strat.candles[7].color = CANDLE_WHITE;
}

int strat_drop(void) {
    int ret;
    WARNING(0, "Drop it like it is hot !!!");


retrydrop:

    trajectory_goto_forward_xy_abs(&robot.traj, 400, COLOR_Y(1400));
    ret = wait_traj_end(TRAJ_FLAGS_NEAR);

    trajectory_goto_forward_xy_abs(&robot.traj, 120, COLOR_Y(1400));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!(TRAJ_SUCCESS(ret))) {
        if(ret == END_OBSTACLE) {
            goto retrydrop;
        }
        else
            return ret;
    }



    strat_open_servo(LEFT);
    strat_open_servo(RIGHT);
    
    strat_wait_ms(500);

    trajectory_d_rel(&robot.traj, -300);
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret))
        return ret;

    strat_release_servo(LEFT);
    strat_release_servo(RIGHT);
    left_pump(0);
    right_pump(0);
}

void job1(void *dummy) {
    static int i = 10;
    if(i--)
        return 1;

    return 0;
}

int strat_do_funny_action(void *dummy) {
    if(strat.time < 60)
        return 1;

    printf("doing it, time = %d\n", strat.time);


    robot.mode = BOARD_MODE_FREE;
    IOWR(PIO_BASE, 0, 1 << 9);
    right_pump(1);
    strat_wait_ms(9000);
    right_pump(0);
    return 0;
}

void strat_begin(void) {
    int ret;                                // TODO : unused variable
    /* Starts the game timer. */
    strat.time = 0;

    cvra_beacon_init(&robot.beacon, AVOIDING_BASE, AVOIDING_IRQ);
    scheduler_add_periodical_event(increment_timer, NULL, 1000000/SCHEDULER_UNIT);

    /* Prepares the object DB. */
    strat_set_objects();

    /* Do the two central glasses. */
    strat_do_first_glasses(); 
    strat_do_far_glasses();
    strat_do_near_glasses();
    strat_drop();
    
    strat_schedule_job(strat_do_gifts, NULL);

    strat_schedule_job(strat_do_funny_action, NULL);

    strat_do_jobs();
    
    left_pump(0);
    right_pump(0);
}


