#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <2wheels/trajectory_manager_utils.h>
#include <scheduler.h>
#include <aversive/error.h>
#include "arm.h"
#include "arm_interpolators.h"
#include <cvra_beacon.h>

struct strat_info strat;


void left_pump(int status) {
    if(status > 0)
        cvra_dc_set_pwm4(HEXMOTORCONTROLLER_BASE, 475);
    else if(status < 0)
        cvra_dc_set_pwm4(HEXMOTORCONTROLLER_BASE, -475);
    else
        cvra_dc_set_pwm4(HEXMOTORCONTROLLER_BASE, 0);
}

void right_pump(int status) {
    if(status > 0)
        cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, -475);
    else if(status < 0)
        cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, 475);
    else
        cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, 0);
}

void strat_open_servo(enum servo_e servo) {
    if(servo == RIGHT)
        cvra_servo_set(SERVOS_BASE, 1, 21000);
    else
        cvra_servo_set(SERVOS_BASE, 0, 9000); 
}


void strat_close_servo(enum servo_e servo) {
    if(servo == RIGHT)
        cvra_servo_set(SERVOS_BASE, 1, 17500);
    else
        cvra_servo_set(SERVOS_BASE, 0, 11500); 
}

void strat_release_servo(enum servo_e servo) {
    if(servo == RIGHT)
        cvra_servo_set(SERVOS_BASE, 1, 15000);
    else
        cvra_servo_set(SERVOS_BASE, 0, 15000); 
}

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
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used 
    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 197, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 15, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x, 
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 15, COORDINATE_TABLE, .8, 135, 100);

    arm_interpolator_append_point(&traj, 150, -150, 197, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, -28, -66, 207, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, -28, -66, 192, COORDINATE_ARM, 1.);
    

    arm_execute_movement(&robot.left_arm, &traj);
}


void right_arm_take_glass(int glass_index) {
    arm_trajectory_t traj;
    float x, y, z;

    arm_get_position(&robot.right_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used 
    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 197, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x,
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 15, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[glass_index].pos.x, 
                                        COLOR_Y(strat.glasses[glass_index].pos.y), 15, COORDINATE_TABLE, .8, 135, 100);

    
    arm_interpolator_append_point(&traj, 150, 150, 197, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, -28, 66, 207, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, -28, 66, 192, COORDINATE_ARM, 1.); 

    arm_execute_movement(&robot.right_arm, &traj);
}

void strat_degage_bras(void) {
    arm_trajectory_t traj;
    float x, y, z;

    arm_get_position(&robot.right_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used     
    arm_interpolator_append_point(&traj, 0, 66, 192, COORDINATE_ARM, .5);
    arm_execute_movement(&robot.right_arm, &traj);

    arm_get_position(&robot.left_arm, &x, &y, &z);
    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used     
    arm_interpolator_append_point(&traj, 0, -66, 192, COORDINATE_ARM, .5);
    arm_execute_movement(&robot.left_arm, &traj);
}

int strat_do_near_glasses(void) {
    int ret;
    robot.left_arm.shoulder_mode = SHOULDER_BACK;
    robot.right_arm.shoulder_mode = SHOULDER_BACK;

    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[1].pos.x+95, COLOR_Y(strat.glasses[2].pos.y));
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret))
        return ret;


    left_pump(1);
    right_pump(1);

   // trajectory_a_abs(&robot.traj, 180);
    //wait_traj_end(TRAJ_FLAGS_STD);

    if(strat.color == BLUE) {
     //   left_arm_take_glass(0);
        right_arm_take_glass(1);
    }
    else {
      //  left_arm_take_glass(1);
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

    trajectory_a_abs(&robot.traj, 180);
    wait_traj_end(TRAJ_FLAGS_STD);


    left_pump(1);
    right_pump(1);


    if(strat.color == BLUE) {
      //  left_arm_take_glass(3);
        right_arm_take_glass(4);
    }
    else {
       // left_arm_take_glass(4);
        right_arm_take_glass(3);
    }

    while(!arm_trajectory_finished(&robot.left_arm) || !arm_trajectory_finished(&robot.left_arm));


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
    strat_close_servo(LEFT);
    strat_close_servo(RIGHT);

    return END_TRAJ;
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

    strat_open_servo(LEFT);
    strat_open_servo(RIGHT);

    trajectory_d_rel(&robot.traj, -100);
    ret = wait_traj_end(TRAJ_FLAGS_STD);

    if(!TRAJ_SUCCESS(ret))
        return;

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
    ret = strat_do_first_glasses();

    if(TRAJ_SUCCESS(ret)) {

        strat_do_far_glasses();
        ret = strat_do_near_glasses();
        if(TRAJ_SUCCESS(ret)) 
            strat_drop();
    }

    left_pump(0);
    right_pump(0);

    /* Pickup this shit. */
/*    strat_do_far_glasses();
    strat_do_near_glasses(); */

}

void strat_autopos(int16_t x, int16_t y, int16_t a, int16_t epaisseurRobot) {

	robot.is_aligning = 1;

	// Pour se recaler, on met le robot en regulation angulaire, on reduit la vitesse et l'acceleration
	// On diminue la sensibilite on augmente la constante de temps de detection du bloquage

	bd_set_thresholds(&robot.distance_bd,  2000, 2);

	trajectory_set_speed(&robot.traj, 100, 100);
	robot.mode = BOARD_MODE_DISTANCE_ONLY;

	// On recule jusqu'a  qu'on ait touche un mur
	trajectory_d_rel(&robot.traj, (double) -2000);

	while(!bd_get(&robot.distance_bd));
	trajectory_hardstop(&robot.traj);
	bd_reset(&robot.distance_bd);
	bd_reset(&robot.angle_bd);
	robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    position_set(&robot.pos, epaisseurRobot, 0, 0);

	/* On se mets a la bonne position en x. */
	trajectory_d_rel(&robot.traj, (double) (x - epaisseurRobot));
	while(!trajectory_finished(&robot.traj));

	/* On se tourne face a la paroi en Y. */
	trajectory_only_a_abs(&robot.traj, COLOR_A(90));
	while(!trajectory_finished(&robot.traj));

	/* On recule jusqu'a avoir touche le bord. */
	trajectory_d_rel(&robot.traj, (double) -2000);
	while(!bd_get(&robot.distance_bd));

	bd_reset(&robot.distance_bd);
	bd_reset(&robot.angle_bd);

	/* On reregle la position. */
    /* XXX ze + 100 is a hotfix for 2013. */
	position_set(&robot.pos, position_get_x_s16(&robot.pos), COLOR_Y((epaisseurRobot+100)), COLOR_A(90));

	/* On se met en place a la position demandee. */
	trajectory_d_rel(&robot.traj, (double) (y - epaisseurRobot));
	while(!trajectory_finished(&robot.traj));

	/* Pour finir on s'occuppe de l'angle. */
	trajectory_a_abs(&robot.traj, (double)a);
	while(!trajectory_finished(&robot.traj));

	/* On remet le robot dans son etat initial. */
	robot.mode = BOARD_MODE_ANGLE_DISTANCE;
	robot.is_aligning = 0;
}


int test_traj_end(int why) {

    if((why & END_TRAJ) && trajectory_finished(&robot.traj))
        return END_TRAJ;

	if (why & END_NEAR) {
		int16_t d_near = 100; /* mm */
        /* XXX Change distance depending on speed. */        
		if (trajectory_in_window(&robot.traj, d_near, RAD(5.0)))
			return END_NEAR;
    }

    if((why & END_OBSTACLE) && robot.beacon.nb_edges != 0) {
        trajectory_hardstop(&robot.traj);
        return END_OBSTACLE;
    }

    if((why & END_BLOCKING) && bd_get(&robot.distance_bd)) {
        trajectory_hardstop(&robot.traj);
        return END_BLOCKING;
    }

    if((why & END_BLOCKING) && bd_get(&robot.angle_bd)) {
        trajectory_hardstop(&robot.traj);
        return END_BLOCKING;
    }

    /* XXX Implement END_OBSTACLE when we got our beacons. */

    if((why & END_TIMER) && strat.time >= MATCH_TIME) {
        trajectory_hardstop(&robot.traj);
        return END_TIMER;
    } 

    return 0;	
}

int wait_traj_end(int why) {
    int ret;
    /* Here we could easily insert debugging facilities. */
    do {
        ret = test_traj_end(why);
    } while(ret==0); 

    return ret;
}
