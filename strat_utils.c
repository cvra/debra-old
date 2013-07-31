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

    //position_set(&robot.pos, epaisseurRobot, 0, -1.1778+0.254);

    position_set(&robot.pos, epaisseurRobot, 0, COLOR_A(0.));
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
    robot.pos.pos_d.y = COLOR_Y(epaisseurRobot+100); 
    robot.pos.pos_s16.y = COLOR_Y(epaisseurRobot+100);

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

    if((why & END_OBSTACLE)) {
        int i;
        for(i=0;i<robot.beacon.nb_beacon;i++) {
            /* Going forward. */
            if(robot.beacon.beacon[i].distance < 60) { /*cm*/
                if(robot.distance_qr.previous_var > 0) {
                    if(robot.beacon.beacon[i].direction > -30 && robot.beacon.beacon[i].direction < 30) {

                        trajectory_stop(&robot.traj);
                        while(robot.distance_qr.previous_var > 0);
                        trajectory_hardstop(&robot.traj);
                        bd_reset(&robot.distance_bd);
                        return END_OBSTACLE;
                    }
                }
                else if(robot.distance_qr.previous_var < 0) {
                    if(robot.beacon.beacon[i].direction < -30 || robot.beacon.beacon[i].direction > 30) {
                        trajectory_stop(&robot.traj);
                        while(robot.distance_qr.previous_var < 0);
                        trajectory_hardstop(&robot.traj);
                        bd_reset(&robot.distance_bd);
                        return END_OBSTACLE;
                    }
                }
            }
        }
    }

    if((why & END_BLOCKING) && bd_get(&robot.distance_bd)) {
        trajectory_hardstop(&robot.traj);
        WARNING(0, "Erreur choc distance !");
        return END_BLOCKING;
    }

    if((why & END_BLOCKING) && bd_get(&robot.angle_bd)) {
        trajectory_hardstop(&robot.traj);

        WARNING(0, "Erreur choc angle !");
        return END_BLOCKING;
    } 

    if((why & END_TIMER) && strat_get_time() >= MATCH_TIME) {
        trajectory_hardstop(&robot.traj);
        return END_TIMER;
    } 

    return 0;	
}

int wait_traj_end_debug(int why, char *file, int line) {
    int ret;
    do {
        ret = test_traj_end(why);
    } while(ret==0); 

    DEBUG(0, "%s:%d got %d", file, line, ret);

    return ret;
}


void right_pump(int status) {
    if(status > 0)
        cvra_dc_set_pwm4(HEXMOTORCONTROLLER_BASE, 475);
    else if(status < 0)
        cvra_dc_set_pwm4(HEXMOTORCONTROLLER_BASE, -475);
    else
        cvra_dc_set_pwm4(HEXMOTORCONTROLLER_BASE, 0);
}

void left_pump(int status) {
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
