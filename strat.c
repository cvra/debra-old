#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <2wheels/trajectory_manager_utils.h>
#include <scheduler.h>
#include <aversive/error.h>
#include <obstacle_avoidance.h>
#include "arm.h"
#include "arm_interpolators.h"
#include <stdio.h>
#include "strat_job.h"
#include <cvra_beacon.h>
#include "strat_utils.h"
#include <stdio.h>
#include <fcntl.h>

/* Strat config. Comment out to disable specific features. */
#define STRAT_DO_WHITE_CANDLES

struct strat_info strat;

void strat_look_cool(void) {
    float x, y, z;

    arm_trajectory_t traj;
    WARNING(0, "%s()", __FUNCTION__);

    strat_wait_ms(3000);
    arm_calibrate();

    arm_trajectory_init(&traj); 
    arm_get_position(&robot.left_arm, &x, &y, &z);
    arm_interpolator_append_point_with_length(&traj, 200, 200, z, COORDINATE_TABLE, 1., 135.5, 96);
    arm_execute_movement(&robot.left_arm, &traj);

    arm_trajectory_init(&traj); 
    arm_get_position(&robot.right_arm, &x, &y, &z);
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 3.);
    arm_interpolator_append_point_with_length(&traj, 200, -200, z, COORDINATE_TABLE, 1., 135.5, 96);
    arm_execute_movement(&robot.right_arm, &traj);

    robot.mode = BOARD_MODE_FREE; 
}

int strat_get_time(void) {
    return (uptime_get() - strat.time_start) / 1000000;
}

void strat_ask_for_candles(void) {
    WARNING(0, "Asking for candles.");
    FILE *bluetooth = fopen("/dev/comBT1", "w");
    if(bluetooth == NULL) {
        ERROR(0, "cannot open /dev/comBT1, aborting.\n");
    }
    else {
        fprintf(bluetooth, "S\n");
        fclose(bluetooth);
    } 
}

void strat_parse_candle_pos(void) {
    const char path[] = "/dev/comBT1";
    int bluetooth;
    char buffer[6];
    int i;

    NOTICE(0, "Parsing candles.");
    /* Zeroes out the buffer. */
    memset(buffer, 0, 6);
    
    /* Try to open the device. */
    bluetooth = open(path, O_RDONLY | O_NONBLOCK);

    /* In case of failure, log it and return. */
    if(bluetooth < 0) {
        ERROR(0, "Cannot open %s, aborting.", path);
        return;
    }

    /* If we dont get enough char, abort. */
    if(read(bluetooth, buffer, 5) < 5) {
        ERROR(0, "Not enough characters in buffer (got '%s') , aborting", buffer);
        close(bluetooth);
        return;
    }

    /* Log the raw data. */
    NOTICE("Got '%s'", buffer);

    /* Parse candle data. */
    for(i=1;i<6;i++) {
        if(buffer[i-1] == 'r') {
            NOTICE(0, "Candle %d is red!", i);
            strat.candles[11-i].color = RED;
            strat.candles[i].color = BLUE;
        }
        else if(buffer[i-1] == 'b') {
            NOTICE(0, "Candle %d is blue!", i);
            strat.candles[11-i].color = BLUE;
            strat.candles[i].color = RED;
        }
        else {
            ERROR(0, "Got unknown char %d at index %d!!\n", buffer[i], i);
        }
    }

    /* Close the bluetooth device. */
    close(bluetooth);

    /* The candle on our side is always ours. */
    strat.candles[11].color = strat.color;

    /* The candle on opp side is always his. */
    if(strat.color == RED)
        strat.candles[0].color = BLUE;
    else
        strat.candles[0].color = RED;

#ifdef STRAT_DO_WHITE_CANDLES
    /* White candles. */
    strat.candles[4].color = strat.color;
    strat.candles[5].color = strat.color;
    strat.candles[6].color = strat.color;
    strat.candles[7].color = strat.color;
#endif

    NOTICE(0, "Candle colors : ");
    for(i=0;i<12;i++)
        fprintf(stderr, "%c", strat.candles[i].color==RED?'r':'b');
    fprintf(stderr, "\n");
}

void strat_da_rel_to_xy_abs(float a_deg, float distance_mm, int *x_mm, int *y_mm) {
    *x_mm = distance_mm * cos(RAD(a_deg)) + position_get_x_s16(&robot.pos);
    *y_mm = distance_mm * sin(RAD(a_deg)) + position_get_y_s16(&robot.pos);
}

void create_opp_polygon(poly_t *pol, int x, int y) {
    const int width = 600; // half width XXX check this, it should be greater IMHO
    const int height = width;

    oa_poly_set_point(pol, x+width, y+width, 0);
    oa_poly_set_point(pol, x+width, y-width, 1);
    oa_poly_set_point(pol, x-width, y-width, 2);
    oa_poly_set_point(pol, x-width, y+width, 3);
}

int strat_goto_avoid(int x, int y, int flags) {
    int i, len, ret;
    int retry_count;
    int opp_x, opp_y;
    poly_t *pol_opp; 
    point_t *p;
    oa_init();
//    pol_cake = oa_new_poly(12);
    // create polygon for cake
    // ...

    // We retry the trajectory 3 times
    for(retry_count=0;retry_count<3;retry_count++) {
        for(i=0;i<robot.beacon.nb_beacon;i++) {
            pol_opp = oa_new_poly(4);

            strat_da_rel_to_xy_abs(robot.beacon.beacon[i].direction, robot.beacon.beacon[i].distance*10,
                   &opp_x, &opp_y); 

            WARNING(0, "Op is at %d;%d", opp_x, opp_y);
            WARNING(0, "> implying op is not a fag...");

            // create polygon for opp

            create_opp_polygon(pol_opp, 800, 0);

            if(is_point_in_poly(pol_opp, x, y)) {
                WARNING(0, "Destination point is in opponent.");
                return END_ERROR;
            }
        }

        // Set starting and ending point of the path
        oa_start_end_points(position_get_x_s16(&robot.pos), position_get_x_s16(&robot.pos), x, y);

        // Compute the path
        len = oa_process();

        // If we cannot find a path
        if(len == 0) {
            WARNING(0, "Cannot find a suitable path.");
            return END_ERROR;
        }

        // Do all the path
        p = oa_get_path();
        for(i=0;i<len;i++) {
            trajectory_goto_forward_xy_abs(&robot.traj, p->x, p->y);
            ret = wait_traj_end(flags);

            if(ret == END_BLOCKING || ret == END_OBSTACLE) {
                WARNING(0, "Retry");
                break; // we retry once more
            }
            else if(!TRAJ_SUCCESS(ret)) {
                WARNING(0, "Unknown error code : %d", ret);
                return ret;
            }
            p++;
        }

        // If we managed to go to the last point, exit
        if(ret == END_TRAJ) {
            break;
        }
    }
}


void left_arm_take_glass(int glass_index) {
    arm_trajectory_t traj;
    float x, y, z;

    WARNING(0, "taking glass %d at (%.1f;%.1f)", glass_index, strat.glasses[glass_index].pos.x, strat.glasses[glass_index].pos.y); 

    arm_get_position(&robot.left_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 

    arm_interpolator_append_point(&traj, -28, -66, z, COORDINATE_ARM, 3.);
    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y)+66, 197, COORDINATE_TABLE, .5, 135, 160);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y)+66, 17, COORDINATE_TABLE, 1., 135, 160);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x, 
                                        COLOR_Y(strat.glasses[glass_index].pos.y)-46, 17, COORDINATE_TABLE, 2., 135, 100);

    arm_interpolator_append_point(&traj, 150, -150, 202, COORDINATE_ARM, 1.5);
    arm_interpolator_append_point(&traj, -28, -66, 202, COORDINATE_ARM, 1.);


    arm_execute_movement(&robot.left_arm, &traj);
}


void right_arm_take_glass(int glass_index) {
    arm_trajectory_t traj;
    float x, y, z;

    arm_get_position(&robot.right_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 

    arm_interpolator_append_point(&traj, -28, 66, z, COORDINATE_ARM, 3.);
    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y)-46, 197, COORDINATE_TABLE, .5, 135, 160);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y)-46, 17, COORDINATE_TABLE, 1., 135, 160);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x, 
                                        COLOR_Y(strat.glasses[glass_index].pos.y)+66, 17, COORDINATE_TABLE, 2., 135, 100);

    
    arm_interpolator_append_point(&traj, 150, 150, 202, COORDINATE_ARM, 1.5);
    arm_interpolator_append_point(&traj, -28, 66, 202, COORDINATE_ARM, 1.);

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

    WARNING(0, "%s()", __FUNCTION__);

    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[1].pos.x+80, COLOR_Y(strat.glasses[2].pos.y));
    ret = wait_traj_end(END_TRAJ | END_BLOCKING);

    if(!TRAJ_SUCCESS(ret))
        return ret;


    trajectory_a_abs(&robot.traj, 180);
    ret = wait_traj_end(END_TRAJ);    // TODO : necessary? the variable is not used later on...


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


int strat_do_opp_glasses(void) {
    int ret;

    robot.left_arm.shoulder_mode = SHOULDER_BACK;
    robot.right_arm.shoulder_mode = SHOULDER_BACK;

    WARNING(0, "%s() ! opp is faggot !", __FUNCTION__);

    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[7].pos.x-95, COLOR_Y(strat.glasses[2].pos.y));


    strat_wait_ms(100);
    strat_close_servo(LEFT);
    strat_close_servo(RIGHT);
    strat_wait_ms(100);
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret)) {
        return ret;
    }

    left_pump(1);
    right_pump(1);

    if(strat.color == BLUE) {
        left_arm_take_glass(7);
        right_arm_take_glass(8);
    }
    else {
        left_arm_take_glass(8);
        right_arm_take_glass(7);
    }

    while(!arm_trajectory_finished(&robot.right_arm) || !arm_trajectory_finished(&robot.left_arm));

    left_pump(-1);
    right_pump(-1);

    strat_degage_bras();

    return END_TRAJ;
}


void strat_do_far_glasses(void) {
    int ret;
    robot.left_arm.shoulder_mode = SHOULDER_BACK;
    robot.right_arm.shoulder_mode = SHOULDER_BACK;

    WARNING(0, "%s()", __FUNCTION__);

    trajectory_goto_backward_xy_abs(&robot.traj, strat.glasses[3].pos.x+80, COLOR_Y(strat.glasses[2].pos.y));
    ret = wait_traj_end(END_TRAJ | END_BLOCKING);

    if(!TRAJ_SUCCESS(ret))
        return ret;

    trajectory_a_abs(&robot.traj, 180);
    wait_traj_end(END_TRAJ);


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
 * @return Number of glasses taken.
 */
static int strat_do_first_glasses(void) {
    WARNING(E_STRAT, "Doing first glasses."); 

    int ret;
    int retry_count=0;

    strat_open_servo(LEFT);
    strat_open_servo(RIGHT);


    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[2].pos.x, COLOR_Y(strat.glasses[2].pos.y)-50);

    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!(TRAJ_SUCCESS(ret))) {
        return 0;
    }

retry:
    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[5].pos.x, COLOR_Y(strat.glasses[5].pos.y)+50);
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!(TRAJ_SUCCESS(ret))) {
        if(retry_count == 0) {
            strat_wait_ms(2000);
            retry_count++;
            goto retry;
        }
        else {
            strat_close_servo(LEFT);
            strat_close_servo(RIGHT);
            return 1;
        }
    }

    
    trajectory_d_rel(&robot.traj, 100);

    strat_wait_ms(100);
    strat_close_servo(LEFT);
    strat_close_servo(RIGHT);
    strat_wait_ms(100);
    wait_traj_end(TRAJ_FLAGS_STD);

    return 2;
}


const float waypoint_radius = 675; // distance from center of cake to robot waypoints
const float ball_radius = 460;
int strat_do_candle(void *param) {
    int ret;
    float robot_x, robot_y;
    arm_t *arm;

    float x, y, z;
    arm_trajectory_t traj;
    int candle = (int)param;
 

    if(strat.color == BLUE)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;


    robot_x = cos(strat.candles[candle].angle) * (waypoint_radius+200) + 1500;
    robot_y = sin(strat.candles[candle].angle) * (waypoint_radius+200);

    trajectory_goto_forward_xy_abs(&robot.traj, robot_x, COLOR_Y(robot_y));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret)) {
        if(ret==END_TIMER)
            return 0;
        else
            return 1;
    }


    robot_x = cos(strat.candles[candle].angle) * (waypoint_radius) + 1500;
    robot_y = sin(strat.candles[candle].angle) * (waypoint_radius);

    trajectory_goto_forward_xy_abs(&robot.traj, robot_x, COLOR_Y(robot_y));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret)) {
        if(ret==END_TIMER)
            return 0;
        else
            return 1;
    }

    trajectory_a_abs(&robot.traj, COLOR_A(DEG(strat.candles[candle].angle) - 90)); // XXX check red
    ret = wait_traj_end(TRAJ_FLAGS_STD); 
    if(!TRAJ_SUCCESS(ret)) {
        if(ret==END_TIMER)
            return 0;
        else
            return 1;
    }


    arm_get_position(arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 1.); // duration not used 

    arm_interpolator_append_point_with_length(&traj, cos(strat.candles[candle].angle) * ball_radius + 1500, COLOR_Y(sin(strat.candles[candle].angle) * ball_radius), z, COORDINATE_TABLE, .3, 135, 85); 
    arm_interpolator_append_point_with_length(&traj, cos(strat.candles[candle].angle) * ball_radius + 1500, COLOR_Y(sin(strat.candles[candle].angle) * ball_radius), 140., COORDINATE_TABLE, .5, 135, 85); 
    arm_interpolator_append_point_with_length(&traj, cos(strat.candles[candle].angle) * ball_radius + 1500, COLOR_Y(sin(strat.candles[candle].angle) * ball_radius), z, COORDINATE_TABLE, .5, 135, 85); 

    if(strat.color==BLUE)
        arm_interpolator_append_point_with_length(&traj, 100, 20, z, COORDINATE_ARM, .4, 135, 95); 
    else
        arm_interpolator_append_point_with_length(&traj, 100, -20, z, COORDINATE_ARM, .4, 135, 95); 

    arm_execute_movement(arm, &traj);
    while(!arm_trajectory_finished(arm));
    arm_shutdown(arm);

    return 0;
}

int strat_do_candles(void) {
    int i;
    int ret;
    float robot_x, robot_y;

    arm_t *arm;
    float x, y, z;
    arm_trajectory_t traj;

    if(strat.color == BLUE)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;


    robot_x = cos(strat.candles[10].angle) * (waypoint_radius) + 1500;
    robot_y = sin(strat.candles[10].angle) * (waypoint_radius);

    // replace with goto_avoid
    trajectory_goto_forward_xy_abs(&robot.traj, robot_x, COLOR_Y(1000));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret)) {
        if(ret==END_TIMER)
            return 0;
        else
            return 1;
    }

    trajectory_goto_forward_xy_abs(&robot.traj, robot_x-150, COLOR_Y(robot_y));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret)) {
        if(ret==END_TIMER)
            return 0;
        else
            return 1;
    }

    trajectory_goto_forward_xy_abs(&robot.traj, robot_x, COLOR_Y(robot_y));
    ret = wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);
    if(!TRAJ_SUCCESS(ret)) {
        if(ret==END_TIMER)
            return 0;
        else
            return 1;
    }
    trajectory_a_abs(&robot.traj, COLOR_A(DEG(strat.candles[10].angle) - 90)); // XXX check red
    ret = wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE); 
    if(!TRAJ_SUCCESS(ret)) {
        if(ret==END_TIMER)
            return 0;
        else
            return 1;
    }

    arm->shoulder_mode = SHOULDER_BACK;

    // we do the candles backward.
    for(i=11;i>=5;i--) {
        robot_x = cos(strat.candles[i].angle) * waypoint_radius + 1500;
        robot_y = sin(strat.candles[i].angle) * waypoint_radius;

        // replace with goto_avoid

        if(i!=10 && i%2==0) {
            trajectory_goto_forward_xy_abs(&robot.traj, robot_x, COLOR_Y(robot_y));
            ret = wait_traj_end(TRAJ_FLAGS_STD);
            if(!TRAJ_SUCCESS(ret)) {
                if(ret == END_TIMER)
                    return 0;
                else {
                    WARNING(0, "Got a problem !!");
                    for(i=5;i<12;i++) {
                        if(strat.candles[i].color == strat.color && !strat.candles[i].done) {
                            WARNING(0, "Scheduling candle %d\n", i);
                            strat_schedule_job(strat_do_candle, (void *)i);
                        }
                    }
                    break; // jumps out of the first for
                }
            }
        }

        arm_trajectory_init(&traj); 

        if(strat.candles[i].color == strat.color) {
            arm_get_position(arm, &x, &y, &z);
            arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 1.); // duration not used 
            arm_interpolator_append_point_with_length(&traj, cos(strat.candles[i].angle) * ball_radius + 1500, COLOR_Y(sin(strat.candles[i].angle) * ball_radius), z, COORDINATE_TABLE, .5, 135, 85); 
            arm_interpolator_append_point_with_length(&traj, cos(strat.candles[i].angle) * ball_radius + 1500, COLOR_Y(sin(strat.candles[i].angle) * ball_radius), 140., COORDINATE_TABLE, .5, 135, 85); 
            arm_interpolator_append_point_with_length(&traj, cos(strat.candles[i].angle) * ball_radius + 1500, COLOR_Y(sin(strat.candles[i].angle) * ball_radius), z, COORDINATE_TABLE, .5, 135, 85); 
    
            if(strat.color==BLUE)
                arm_interpolator_append_point_with_length(&traj, 100, 20, z, COORDINATE_ARM, .4, 135, 95); 
            else
                arm_interpolator_append_point_with_length(&traj, 100, -20, z, COORDINATE_ARM, .4, 135, 95); 

            arm_execute_movement(arm, &traj);
            while(!arm_trajectory_finished(arm));
            arm_shutdown(arm);
            strat.candles[i].done = 1;
        }
    }

    arm_get_position(arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 1.); // duration not used 
    arm_interpolator_append_point(&traj, x, y, 197, COORDINATE_ARM, .5); 
    arm_execute_movement(arm, &traj);

    return 0;
}

int strat_do_gift(void *param) {
    int gift_index = (int)param;

    int ret;
    arm_t *arm;
    float x, y, z;

    arm_trajectory_t traj;

    const float height = 100; // height of attack
    const float depth = 40; // the length to go out of the table

    if(strat.color == RED)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    WARNING(0, "%s(%d)", __FUNCTION__, gift_index);


    // XXX strat avoid
    trajectory_goto_forward_xy_abs(&robot.traj, strat.gifts[gift_index].x-50, COLOR_Y(2000-230));
    ret = wait_traj_end(TRAJ_FLAGS_STD);        // TODO : is this line really necessary when a copy of it appears 4 lines below?

    if(!TRAJ_SUCCESS(ret)) {
        WARNING(0, "%s() got %d as answer.", __FUNCTION__, ret);
        if(ret == END_TIMER)
            return 0;
        else
            return 1;
    }

    trajectory_a_abs(&robot.traj, 0);
    ret = wait_traj_end(TRAJ_FLAGS_STD);


    if(!TRAJ_SUCCESS(ret)) {
        WARNING(0, "%s() got %d as answer.", __FUNCTION__, ret);
        if(ret == END_TIMER)
            return 0;
        else
            return 1;
    }

    arm_trajectory_init(&traj);

    arm_interpolator_append_point_with_length(&traj, 100, 10, height, COORDINATE_ARM, 1., 135, 95); 
    arm_interpolator_append_point_with_length(&traj, strat.gifts[gift_index].x-25,
            COLOR_Y(2000+depth), height, COORDINATE_TABLE, .4, 135, 82); 
    arm_interpolator_append_point_with_length(&traj, 100, 10, height, COORDINATE_ARM, .4, 135, 95); 

    arm_execute_movement(arm, &traj);

    while(!arm_trajectory_finished(arm));

    return 0;
}

int strat_do_gifts(void *dummy) {
    int ret;
    arm_t *arm;
    float x, y, z;

    arm_trajectory_t traj;

    const float height = 100; // height of attack
    const float depth = 40; // the length to go out of the table
    int i;

    WARNING(0, "%s()", __FUNCTION__);


    if(strat.color == RED)
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;

    arm->shoulder_mode = SHOULDER_FRONT;

    trajectory_goto_forward_xy_abs(&robot.traj, strat.gifts[0].x+70, COLOR_Y(1000));
    ret = wait_traj_end(TRAJ_FLAGS_STD);        // TODO : is this line really necessary when a copy of it appears 4 lines below?
    
    arm_get_position(arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 1.); // duration not used 
    if(strat.color == BLUE) {
        arm_interpolator_append_point(&traj, x, y+30, z, COORDINATE_ARM, .2); 
        arm_interpolator_append_point(&traj, 150, y+30, z, COORDINATE_ARM, .3); 
    }
    else {
        arm_interpolator_append_point(&traj, x, y-30, z, COORDINATE_ARM, .1); 
        arm_interpolator_append_point(&traj, 150, y-30, z, COORDINATE_ARM, .3); 
    }
    arm_interpolator_append_point_with_length(&traj, 100, 10, z, COORDINATE_ARM, 1., 135, 95); 
    arm_interpolator_append_point_with_length(&traj, 100, 10, height, COORDINATE_ARM, 1., 135, 95); //XXX change for red
    arm_execute_movement(arm, &traj);


    while(!arm_trajectory_finished(arm));

    for(i=0;i<4;i++) {

        trajectory_goto_forward_xy_abs(&robot.traj, strat.gifts[i].x-50, COLOR_Y(2000-240));
        ret = wait_traj_end(TRAJ_FLAGS_STD);        // TODO : is this line really necessary when a copy of it appears 4 lines below?

        if(!TRAJ_SUCCESS(ret))
            break;

        if(i==0) {
            trajectory_a_abs(&robot.traj, 0);
            ret = wait_traj_end(TRAJ_FLAGS_STD);
        }

        arm_trajectory_init(&traj);

        arm_interpolator_append_point_with_length(&traj, 100, 10, height, COORDINATE_ARM, 1., 135, 95); 
        arm_interpolator_append_point_with_length(&traj, strat.gifts[i].x-25,
                COLOR_Y(2000+depth), height, COORDINATE_TABLE, 0.4, 135, 82); 
        arm_interpolator_append_point_with_length(&traj, 100, 10, height, COORDINATE_ARM, 0.4, 135, 95); 

        arm_execute_movement(arm, &traj);

        while(!arm_trajectory_finished(arm));
        strat.gifts[i].done = 1;
    }

    for(i=3;i>=0;i--) {
        if(!strat.gifts[i].done) {
            strat_schedule_job(strat_do_gift, (void *)i);
            WARNING(0, "Queuing gift %d", i);
        }
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
        if(strat.color == RED)
            strat.candles[i].color = BLUE;
        else
            strat.candles[i].color = RED;

        alpha = alpha + 15.;
    }

    /* The candle on our side is always our color. */
    strat.candles[11].color = strat.color;

#ifdef STRAT_DO_WHITE_CANDLES
    /* White candles. */
    strat.candles[4].color = strat.color;
    strat.candles[5].color = strat.color;
    strat.candles[6].color = strat.color;
    strat.candles[7].color = strat.color;
#endif
}

int strat_drop(void) {
    int ret;
    int blocked_time;
    int drop_zone = 0; // en partant de l'oppose du gateau
    int traj_flags = TRAJ_FLAGS_STD;

    WARNING(0, "%s()", __FUNCTION__);

    // We try every drop case
    for(drop_zone = 0;drop_zone < 3;drop_zone++) {
        NOTICE(0, "Trying zone %d", drop_zone);
        trajectory_goto_forward_xy_abs(&robot.traj, 400, COLOR_Y(1400 - 400*drop_zone));
        ret = wait_traj_end(TRAJ_FLAGS_STD);

        if(!(TRAJ_SUCCESS(ret))) {
            WARNING(0, "Cannot go to drop zone %d");
            continue; // on passe a la case suivante
        }

        
        if(drop_zone == 0)
            trajectory_goto_forward_xy_abs(&robot.traj, 120, COLOR_Y(1400 - 400*drop_zone));
        else
            trajectory_goto_forward_xy_abs(&robot.traj, 320, COLOR_Y(1400 - 400*drop_zone));

        ret = wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);
        break;
    }

    if(!(TRAJ_SUCCESS(ret))) {
        ERROR(0, "Cannot drop !!!");
        return 1;
    }

    strat_open_servo(LEFT);
    strat_open_servo(RIGHT);
    
    strat_wait_ms(1000);

    blocked_time = strat_get_time();
    do {
        // wait 10s, then bourrine
        if(strat_get_time() < blocked_time + 10) {
            trajectory_goto_backward_xy_abs(&robot.traj, 600, COLOR_Y(1400 - 400*drop_zone));
            ret = wait_traj_end(traj_flags);
        }
        else {
            if(drop_zone == 0) {
                drop_zone++;
            }
            else {
                drop_zone--;
            }
            if(drop_zone == 0)
                trajectory_goto_backward_xy_abs(&robot.traj, 120, COLOR_Y(1400 - 400*drop_zone)); 
            else
                trajectory_goto_backward_xy_abs(&robot.traj, 320, COLOR_Y(1400 - 400*drop_zone)); 
            // we specifically disable obstacle avoidance to gtfo
            ret = wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);
            trajectory_goto_backward_xy_abs(&robot.traj, 600, COLOR_Y(1400 - 400*drop_zone)); 
            ret = wait_traj_end(TRAJ_FLAGS_SHORT_DISTANCE);
        } 
    } while(!TRAJ_SUCCESS(ret) && ret != END_TIMER);

    strat_release_servo(LEFT);
    strat_release_servo(RIGHT);
    left_pump(0);
    right_pump(0);

    return 0;
}

int strat_do_funny_action(void *dummy) {

    if(strat_get_time() < 95)
        return 1;

    
    WARNING(0, "%s()", __FUNCTION__);
    robot.mode = BOARD_MODE_FREE;
    IOWR(PIO_BASE, 0, 1 << 9);
    right_pump(1);
    while(strat_get_time() < 100);
    right_pump(0);
    return 0;
}

// strat configuration

void strat_begin(void) {
    int ret;                                // TODO : unused variable
    int number_of_glasses;
    /* Starts the game timer. */
    strat.time_start = uptime_get();

    // eteinds l'electrovanne
    IOWR(PIO_BASE, 0, 0 << 9);

    /* Prepares the object DB. */
    strat_set_objects();


    /* Ask the computer vision for informations. */
    strat_ask_for_candles();

    /* Do the two central glasses. */ 
    number_of_glasses = strat_do_first_glasses(); 
    if(number_of_glasses == 2)
        strat_do_far_glasses();
    if(number_of_glasses >= 1) {
        strat_do_near_glasses();
        strat_schedule_job(strat_drop, NULL);
    }  

    /* Parse computer vision answer. */
    strat_parse_candle_pos(); // WARNING : blocking
   
   /* 
    strat_schedule_job(strat_do_gifts, NULL); 
    strat_schedule_job(strat_do_candles, NULL);
    strat_schedule_job(strat_do_funny_action, NULL);
    */

    strat_do_jobs();
    
    left_pump(0);
    right_pump(0);
}


