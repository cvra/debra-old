/**
 @file cvra_cs.c
 @author Antoine Albertelli
 @date 19th September 2009
 @brief This files implements the control system loop wrappers.
 
 The file provides all the implementation of the Control System Management (CSM).
 A CSM is made of 3 parts : acceleration / deceleration ramp generator 
 (Quadramp), a position regulator (PID), and I/O interfaces (PWM & quadratures
 encoders).
 The quadramp tells the PID what position the wheel should be, depending on the 
 acceleration and time. This value is then fed as the consign value to the PID 
 regulator, with the encoder value as the measured position. The output of the 
 PID is then applied to the motor via the PWM.
 The others functions computed here are the position manager, the trajectory
 manager and the blocking detection system.
 */

#include <aversive.h>

#include <2wheels/trajectory_manager.h>
#include <2wheels/robot_system.h>
#include <2wheels/position_manager.h>
#include <control_system_manager.h>
#include <pid.h>
#include <quadramp.h>
#include <scheduler.h>

#include <aversive/error.h>
#include "error_numbers.h"
#include "adresses.h"

#include <string.h>
#include <stdio.h>

#include "cvra_cs.h"
#include "hardware.h"
#include "cvra_param_robot.h"


struct _rob robot;


void cvra_cs_init(void) {
	robot.mode = BOARD_MODE_ANGLE_DISTANCE;
	/*--------------------------------------------------------------------------*/
	/*                                Motor                                     */
	/*--------------------------------------------------------------------------*/
    /*XXX TODO */

	/****************************************************************************/
	/*                             Robot system                                 */
	/****************************************************************************/
	rs_init(&robot.rs);
	rs_set_flags(&robot.rs, RS_USE_EXT);

	/*************************f***************************************************/
	/*                         Encoders & PWMs                                  */
	/****************************************************************************/
	/*rs_set_left_pwm(&robot.rs, cvra_bldc_set_pwm, robot.left_motor);
	rs_set_right_pwm(&robot.rs, cvra_bldc_set_pwm_negative, robot.right_motor);
	rs_set_left_ext_encoder(&robot.rs, cvra_bldc_get_encoder, robot.left_motor,
			ROBOT_WHEEL_L_CORR);	// CALIBRATION : Changer ce coefficient a 1 si le codeur va dans le mauvais sens
	rs_set_right_ext_encoder(&robot.rs, cvra_bldc_get_encoder, robot.right_motor,
			ROBOT_WHEEL_R_CORR); // CALIBRATION : idem*/

	/****************************************************************************/
	/*                          Position manager                                */
	/****************************************************************************/

	position_init(&robot.pos);
	/* Links the position manager to the robot system. */
	position_set_related_robot_system(&robot.pos, &robot.rs);
	position_set_physical_params(&robot.pos, ROBOT_ECART_ROUE, // Distance between encoding wheels. // 276
			ROBOT_INC_MM); // imp / mm  //

	/****************************************************************************/
	/*                       Regulation de l'angle                              */
	/****************************************************************************/

	pid_init(&robot.angle_pid);
	pid_set_gains(&robot.angle_pid, ROBOT_PID_ANGL_P, ROBOT_PID_ANGL_I, ROBOT_PID_ANGL_D);		// CALIBRATION : Mettre les gains < 0 si le moteur compense dans le mauvais sens
	pid_set_maximums(&robot.angle_pid, 0, 5000, 30000);
	pid_set_out_shift(&robot.angle_pid, 10);

	quadramp_init(&robot.angle_qr);

	cs_init(&robot.angle_cs); /* Initialise le control system. */
	cs_set_consign_filter(&robot.angle_cs, quadramp_do_filter, &robot.angle_qr); /* Met un filtre en acceleration. */
	cs_set_correct_filter(&robot.angle_cs, pid_do_filter, &robot.angle_pid); /* Met le PID. */
	cs_set_process_in(&robot.angle_cs, rs_set_angle, &robot.rs); /* Met la sortie sur le pwm virtuel de l'angle. */
	cs_set_process_out(&robot.angle_cs, rs_get_ext_angle, &robot.rs); /* lecture codeur virtuel de l'angle */
	cs_set_consign(&robot.angle_cs, 0); /* Met une consigne nulle. */

	/****************************************************************************/
	/*                      Regulation de la distance                           */
	/****************************************************************************/

	pid_init(&robot.distance_pid); /* Initialise le PID. */
	pid_set_gains(&robot.distance_pid, ROBOT_PID_DIST_P, ROBOT_PID_DIST_I, ROBOT_PID_DIST_D); /* Regles les gains du PID. */
	pid_set_maximums(&robot.distance_pid, 0, 5000, 30000); /* pas de max sur l'entree, integral limite a 5000, sortie limitee a 4095 (PWM 12 bits). */
	pid_set_out_shift(&robot.distance_pid, 10); /* Divise la sortie par 1024. */

	quadramp_init(&robot.distance_qr); /* Demarre le filtre en acceleration. *//* set accel. */

	cs_init(&robot.distance_cs); /* Initialise le control system. */
	cs_set_consign_filter(&robot.distance_cs, quadramp_do_filter,
			&robot.distance_qr); /* Met un filtre en acceleration. */
	cs_set_correct_filter(&robot.distance_cs, pid_do_filter, &robot.distance_pid); /* Met le PID. */
	cs_set_process_in(&robot.distance_cs, rs_set_distance, &robot.rs); /* Met la sortie sur le pwm virtuel de l'distance. */
	cs_set_process_out(&robot.distance_cs, rs_get_ext_distance, &robot.rs); /* lecture codeur virtuel de l'distance */
	cs_set_consign(&robot.distance_cs, 0);/* Met une consigne nulle. */

	/****************************************************************************/
	/*                           Trajectory Manager (Trivial)                   */
	/****************************************************************************/
	trajectory_init(&robot.traj, ASSERV_FREQUENCY);
	trajectory_set_cs(&robot.traj, &robot.distance_cs, &robot.angle_cs);
	trajectory_set_robot_params(&robot.traj, &robot.rs, &robot.pos);
	trajectory_set_speed(&robot.traj, 2400, 1200); /* distance, angle */
	trajectory_set_acc(&robot.traj, 40., 30.);
	/* distance window, angle window, angle start */
	trajectory_set_windows(&robot.traj, 30., 1.0, 20.); // Prod

	// Angle BDM
	bd_init(&robot.angle_bd, &robot.angle_cs);
	bd_set_thresholds(&robot.angle_bd, ROBOT_ANGLE_BD, 5);

	// Distance BDM
	bd_init(&robot.distance_bd, &robot.distance_cs);
	bd_set_thresholds(&robot.distance_bd, ROBOT_DIST_BD, 5);

	robot.is_aligning = 0;

	// Initialisation déplacement:
	position_set(&robot.pos, 0, 0, 0);

	/* ajoute la regulation au multitache. ASSERV_FREQUENCY est dans cvra_cs.h */
	scheduler_add_periodical_event_priority(cvra_cs_manage, NULL, (1000000
			/ ASSERV_FREQUENCY) / SCHEDULER_UNIT, 130);
}

/**
 @brief Brings power back on after a blocking.
 
 This functions resets the blocking detection systems, sets the robot on angle
 and distance mode and stops the current trajectory.
 */
static void restart_power(__attribute__((unused)) void * dummy) {
	bd_reset(&robot.angle_bd);
	bd_reset(&robot.distance_bd);
	robot.mode = BOARD_MODE_ANGLE_DISTANCE;
	trajectory_hardstop(&robot.traj);
	robot.is_blocked = 0;
}

/** Logge l'erreur sur les differents regulateurs et l'affiche avec le temps. */
static void dump_error(void) {
	static int time = 0;
	if (robot.error_dump_enabled) {
		if (time % 10)
			fprintf(stderr, "%d;%d;%d\n", time, (int)cs_get_error(&robot.angle_cs), (int)cs_get_error(&robot.distance_cs));
		time++;
	} else {
		time = 0;
	}
}

void cvra_cs_manage(__attribute__((unused)) void * dummy) {
	NOTICE(ERROR_CS, __FUNCTION__);

	/* Gestion de la position. */
	rs_update(&robot.rs);
	position_manage(&robot.pos);

	/* Gestion de l'asservissement. */
	if (robot.mode != BOARD_MODE_SET_PWM) {
		if (robot.mode == BOARD_MODE_ANGLE_DISTANCE || robot.mode == BOARD_MODE_ANGLE_ONLY) {
			cs_manage(&robot.angle_cs);
		} else {
			rs_set_angle(&robot.rs, 0); // Sets a null angle PWM
		}

		if (robot.mode == BOARD_MODE_ANGLE_DISTANCE || robot.mode == BOARD_MODE_DISTANCE_ONLY) {
			cs_manage(&robot.distance_cs);
		} else {
			rs_set_distance(&robot.rs, 0); // Sets a distance angle PWM
		}
	}

	/* Affichage des courbes d'asservissement. */
	dump_error();

#if 1
	/* Gestion du blocage */
	bd_manage(&robot.angle_bd);
	bd_manage(&robot.distance_bd);

	if ((bd_get(&robot.angle_bd) || bd_get(&robot.distance_bd))
			&& !robot.is_aligning) {
		// Stop la bete :)
		printf("Choc roue %i,%i\r", bd_get(&robot.angle_bd),  bd_get(&robot.distance_bd));
		trajectory_hardstop(&robot.traj);
		robot.is_blocked = 1;
		robot.mode = BOARD_MODE_FREE; /* On coupe tout. */
		restart_power(NULL);

	}
#endif
}
