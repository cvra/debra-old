#include <aversive.h>
#include <aversive/error.h>
#include <2wheels/trajectory_manager_utils.h>
#include <cvra_beacon.h>
#include "strat_utils.h"
#include "cvra_cs.h"

void strat_autopos(int16_t x, int16_t y, int16_t a, int16_t epaisseurRobot) {

	robot.is_aligning = 1;

	// Pour se recaler, on met le robot en regulation angulaire, on reduit la vitesse et l'acceleration
	// On diminue la sensibilite on augmente la constante de temps de detection du bloquage

	bd_set_thresholds(&robot.distance_bd,  5000, 2);

	trajectory_set_speed(&robot.traj, 100, 100);
	robot.mode = BOARD_MODE_DISTANCE_ONLY;

	// On recule jusqu'a  qu'on ait touche un mur
	trajectory_d_rel(&robot.traj, (double) -2000);

	while(!bd_get(&robot.distance_bd));
	trajectory_hardstop(&robot.traj);
	bd_reset(&robot.distance_bd);
	bd_reset(&robot.angle_bd);
	robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    position_set(&robot.pos, epaisseurRobot, 0, 0.);
    // XXX delete this

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
    /* XXX ze + 100 is just for 2013. */
	position_set(&robot.pos, position_get_x_s16(&robot.pos), COLOR_Y((epaisseurRobot+100)), COLOR_A(90));

	/* On se met en place a la position demandee. */

	trajectory_set_speed(&robot.traj, speed_mm2imp(&robot.traj, 300), speed_rd2imp(&robot.traj, 2.5) ); /* distance, angle */

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

    /* TODO when rouven has fixed this fucking beacon. */
    /*if((why & END_OBSTACLE) && robot.beacon.nb_edges != 0) {
        trajectory_hardstop(&robot.traj);
        return END_OBSTACLE;
    }*/

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
