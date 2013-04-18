#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <2wheels/trajectory_manager_utils.h>
#include <scheduler.h>
#include <aversive/error.h>
#include "arm.h"
#include "arm_interpolators.h"
#include "strat_job.h"
#include <cvra_beacon.h>
#include "strat_utils.h"

struct strat_info strat;



void strat_left_take_glass(int glass_index) {

}

/** Increments the match timer, called every second. */
static void increment_timer(__attribute__((unused))void *data) {
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
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 197, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 15, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x, 
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 15, COORDINATE_TABLE, .3, 135, 100);

    arm_interpolator_append_point(&traj, 150, -150, 210, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, -28, -66, 210, COORDINATE_ARM, 3.);
    

    arm_execute_movement(&robot.left_arm, &traj);
}


void right_arm_take_glass(int glass_index) {
    arm_trajectory_t traj;
    float x, y, z;

    arm_get_position(&robot.right_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 

    arm_interpolator_append_point(&traj, -28, 66, z, COORDINATE_ARM, 3.);
    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 197, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 15, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x, 
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 15, COORDINATE_TABLE, .3, 135, 100);

    
    arm_interpolator_append_point(&traj, 150, 150, 197, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, -28, 66, 207, COORDINATE_ARM, 3.);

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


    trajectory_a_abs(&robot.traj, 190);
    ret = wait_traj_end(TRAJ_FLAGS_STD);


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


   // strat_degage_bras();

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

    trajectory_a_abs(&robot.traj, 190);
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

   // strat_degage_bras();

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
    ret = wait_traj_end(TRAJ_FLAGS_STD | END_OBSTACLE);


    if(!(TRAJ_SUCCESS(ret))) {
        if(ret == END_OBSTACLE) {
            goto retry2;
        }
        else
            return ret;
    }


    trajectory_d_rel(&robot.traj, 100);
    wait_traj_end(TRAJ_FLAGS_STD | END_OBSTACLE);
    strat_close_servo(LEFT);
    strat_close_servo(RIGHT);

    return END_TRAJ;
}

void strat_job_do_gift(void *param) {
    int gift_index = (int)param;
    WARNING(0, "Doing gift %d from job pool.", gift_index); 
}

void strat_do_gifts(void *dummy) {
    int ret;
    arm_t *arm;
    float x, y, z;

    arm_trajectory_t traj;

    const float height = 100; // height of attack
    const float depth = 150; // the length to go out of the table
    int i;


    if(strat.color == BLUE)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    for(i=0;i<4;i++)
        strat.gifts[i].last_try_time = strat.time;

    
    arm_get_position(arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 1.); // duration not used 
    if(strat.color == BLUE) {
        arm_interpolator_append_point(&traj, x, y-30, z, COORDINATE_ARM, .2); 
        arm_interpolator_append_point(&traj, 150, y-30, 197, COORDINATE_ARM, 1); 
    }
    else {
        arm_interpolator_append_point(&traj, x, y+30, z, COORDINATE_ARM, .1); 
        arm_interpolator_append_point(&traj, 150, y+30, 197, COORDINATE_ARM, 1); 
    }
    arm_interpolator_append_point_with_length(&traj, 100, 0, 197, COORDINATE_ARM, 1., 135, 115); 
    arm_interpolator_append_point_with_length(&traj, 100, 0, height, COORDINATE_ARM, 1., 135, 115); 
    arm_execute_movement(arm, &traj);


    while(!arm_trajectory_finished(arm));

    
/*
    if(!TRAJ_SUCCESS(ret)) {
        for(i=3;i>=0;i--) {
            strat_schedule_job(strat_job_do_gift, (void *)i);
        } 
        return;
    }
    */


    for(i=0;i<4;i++) {
        trajectory_goto_backward_xy_abs(&robot.traj, strat.gifts[i].x, COLOR_Y(2000-250));
        ret = wait_traj_end(TRAJ_FLAGS_STD);


        trajectory_a_abs(&robot.traj, 180);
        ret = wait_traj_end(TRAJ_FLAGS_STD);

        arm_trajectory_init(&traj);

        arm_interpolator_append_point_with_length(&traj, 100, 0, height, COORDINATE_ARM, 1., 135, 115); 

        arm_interpolator_append_point_with_length(&traj, strat.gifts[i].x,
                COLOR_Y(2000+depth), height, COORDINATE_TABLE, 1., 135, 115); 
        arm_interpolator_append_point_with_length(&traj, 100, 0, height, COORDINATE_ARM, 1., 135, 115); 

        arm_execute_movement(arm, &traj);

        while(!arm_trajectory_finished(arm));
    }

    return;


    /* We do the first gift without moving. */
    while(!arm_trajectory_finished(arm));
    strat.gifts[0].done = 1;

    /* start moving. */
    trajectory_goto_backward_xy_abs(&robot.traj, strat.gifts[3].x, COLOR_Y(2000-300));

    while((ret = test_traj_end(TRAJ_FLAGS_STD)) == 0) {
        for(i=1;i<4;i++) {
            /* XXX test if 200 is enough. */
            if(position_get_x_s16(&robot.pos) > strat.gifts[i].x - 200 && !strat.gifts[i].done) {
                if(!arm_trajectory_finished(arm)) {
                    ERROR(0, "Arm is moving too slowly !!");
                    trajectory_stop(&robot.traj);

                    /* reschedule gifts because we will do them slower. */
                    for(i=3;i>=0;i--) {
                        if(!strat.gifts[i].done)
                            strat_schedule_job(strat_job_do_gift, (void *)i);
                    } 
                    return;
                }

                arm_trajectory_init(&traj);
                arm_interpolator_append_point(&traj, 150, 0, height, COORDINATE_ARM, .5);
                arm_interpolator_append_point(&traj, strat.gifts[i].x, 
                        COLOR_Y(2000+depth), height, COORDINATE_TABLE, .5); 
                arm_interpolator_append_point(&traj, 150, 0, height, COORDINATE_ARM, .5); 
                arm_execute_movement(arm, &traj);
                strat.gifts[i].done = 1;
            }
        }
    }


    /* We schedule all the gifts that we haven't done yet for later. */
    for(i=3;i>=0;i--) {
        if(!strat.gifts[i].done)
            strat_schedule_job(strat_job_do_gift, (void *)i);
    } 

    /* Wait for the arm to be complete. */
    while(!arm_trajectory_finished(arm));
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
}

int strat_drop(void) {
    int ret;
    WARNING(0, "Drop it like it is hot !!!");


retrydrop:
    trajectory_goto_forward_xy_abs(&robot.traj, 300, COLOR_Y(1400));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!(TRAJ_SUCCESS(ret))) {
        if(ret == END_OBSTACLE) {
            goto retrydrop;
        }
        else
            return ret;
    }


    strat_degage_bras();

    strat_open_servo(LEFT);
    strat_open_servo(RIGHT);

    trajectory_d_rel(&robot.traj, -300);
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret))
        return ret;

    strat_release_servo(LEFT);
    strat_release_servo(RIGHT);

    robot.mode = BOARD_MODE_FREE;

}

void strat_begin(void) {
    int ret;
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
    

//    strat_do_gifts(NULL);

    strat_schedule_job(strat_do_gifts, NULL); 
    while(!strat_job_pool_is_empty()) {
        WARNING(0, "Doing job!");
        strat_do_job();
    }
    


    left_pump(0);
    right_pump(0);

    /* Pickup this shit. */
/*    strat_do_far_glasses();
    strat_do_near_glasses(); */

}


