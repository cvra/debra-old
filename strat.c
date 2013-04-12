#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <2wheels/trajectory_manager_utils.h>
#include <scheduler.h>
#include <aversive/error.h>
#include "arm.h"
#include "arm_interpolators.h"

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
    if(status)
        cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, -400);
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

/** Increments the match timer, called every second. */
static void increment_timer(__attribute__((unused))void *data) {
    strat.time++;
}

/** @brief Take the first two glasses.
 *
 * This function takes the first two glasses on the correct side.
 * @todo Test the starting coordinates.
 */
static void strat_do_first_glasses(void) {
    arm_trajectory_t traj;
    float x, y, z;
    WARNING(E_STRAT, "Doing first glasses."); 

    robot.left_arm.shoulder_mode = SHOULDER_BACK;

    strat_open_servo(LEFT);
    strat_open_servo(RIGHT);

    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[1].pos.x-50, COLOR_Y(strat.glasses[2].pos.y));

    arm_get_position(&robot.left_arm, &x, &y, &z);

    arm_trajectory_init(&traj); 
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used 
    arm_interpolator_append_point_with_length(&traj, strat.glasses[1].pos.x,
                                        COLOR_Y(strat.glasses[1].pos.y), 197, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[1].pos.x,
                                        COLOR_Y(strat.glasses[1].pos.y), 15, COORDINATE_TABLE, 2., 135, 155);

    arm_interpolator_append_point_with_length(&traj, strat.glasses[1].pos.x, 
                                        COLOR_Y(strat.glasses[1].pos.y), 15, COORDINATE_TABLE, .1, 135, 100);

    arm_interpolator_append_point(&traj, 150, -150, 197, COORDINATE_ARM, 3.);

    arm_interpolator_append_point(&traj, -28, -66, 207, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, -28, -66, 192, COORDINATE_ARM, 1.);



    left_pump(1);


    arm_execute_movement(&robot.left_arm, &traj);

    wait_traj_end(TRAJ_FLAGS_STD);

    strat_close_servo(LEFT);

    while(!arm_trajectory_finished(&robot.left_arm));
    left_pump(-1); 

    return;


    trajectory_goto_forward_xy_abs(&robot.traj, strat.glasses[5].pos.x, COLOR_Y(strat.glasses[5].pos.y)+50);
    wait_traj_end(TRAJ_FLAGS_NEAR);
    strat_close_servo(RIGHT);

    trajectory_a_abs(&robot.traj, -180);
    wait_traj_end(TRAJ_FLAGS_STD);

    trajectory_d_rel(&robot.traj, 1000);

    wait_traj_end(TRAJ_FLAGS_NEAR);
    strat_open_servo(LEFT);
    strat_open_servo(RIGHT);

    trajectory_d_rel(&robot.traj, -100);

    wait_traj_end(TRAJ_FLAGS_STD);

    strat_release_servo(LEFT);
    strat_release_servo(RIGHT); 


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
    strat.glasses[0].pos.x = 900; strat.glasses[0].pos.y = (1550);
    strat.glasses[1].pos.x = 900; strat.glasses[1].pos.y = (950);
    strat.glasses[2].pos.x = 1050; strat.glasses[2].pos.y = (1200);

    /*XXX Not sure about coordinates of 3 and 4. */
    strat.glasses[3].pos.x = 1200; strat.glasses[3].pos.y = (1550);
    strat.glasses[4].pos.x = 1200; strat.glasses[4].pos.y = (1050);
    strat.glasses[5].pos.x = 1350; strat.glasses[5].pos.y = (1200);
    strat.glasses[6].pos.x = 1650; strat.glasses[6].pos.y = (1300);
    strat.glasses[7].pos.x = 1800; strat.glasses[7].pos.y = (1550);
    strat.glasses[8].pos.x = 1800; strat.glasses[8].pos.y = (1050);
    strat.glasses[9].pos.x = 1950; strat.glasses[9].pos.y = (1300);
    strat.glasses[10].pos.x = 2100; strat.glasses[10].pos.y = (1550);
    strat.glasses[11].pos.x = 2100; strat.glasses[11].pos.y = (1050);
}

void strat_begin(void) {
    /* Starts the game timer. */
    strat.time = 0;
    scheduler_add_periodical_event(increment_timer, NULL, 1000000/SCHEDULER_UNIT);

    /* Prepares the object DB. */
    strat_set_objects();

    /* Do the two central glasses. */
    strat_do_first_glasses();

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

    if((why & END_BLOCKING) && bd_get(&robot.distance_bd)) {
        trajectory_hardstop(&robot.traj);
        return END_TRAJ;
    }

    if((why & END_BLOCKING) && bd_get(&robot.angle_bd)) {
        trajectory_hardstop(&robot.traj);
        return END_TRAJ;
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
