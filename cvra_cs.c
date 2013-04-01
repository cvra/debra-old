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
#include <cvra_dc.h>
#include <2wheels/trajectory_manager_utils.h>

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
#ifdef COMPILE_ON_ROBOT
    int i;
    for(i=0;i<6;i++) {
        cvra_dc_set_encoder(HEXMOTORCONTROLLER_BASE, i, 0);
        cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE, i, 0);
    }
#endif

	/****************************************************************************/
	/*                             Robot system                                 */
	/****************************************************************************/
	rs_init(&robot.rs);
	rs_set_flags(&robot.rs, RS_USE_EXT);

	/*************************f***************************************************/
	/*                         Encoders & PWMs                                  */
	/****************************************************************************/
#ifdef COMPILE_ON_ROBOT
	rs_set_left_pwm(&robot.rs, cvra_dc_set_pwm0, HEXMOTORCONTROLLER_BASE);
	rs_set_right_pwm(&robot.rs, cvra_dc_set_pwm5, HEXMOTORCONTROLLER_BASE);
	rs_set_left_ext_encoder(&robot.rs, cvra_dc_get_encoder0, HEXMOTORCONTROLLER_BASE, 0.999981348555308);
	rs_set_right_ext_encoder(&robot.rs, cvra_dc_get_encoder5, HEXMOTORCONTROLLER_BASE, -1.00001865144469);
#endif

	/****************************************************************************/
	/*                          Position manager                                */
	/****************************************************************************/

	position_init(&robot.pos);
	/* Links the position manager to the robot system. */
	position_set_related_robot_system(&robot.pos, &robot.rs);
	position_set_physical_params(&robot.pos, 178. * 0.99805932908 * 0.99889012208657 * 0.999800039992002 * 1.00087947650296, // Distance between encoding wheels. // 276
			162. * 1.002); // imp / mm  //

	/****************************************************************************/
	/*                       Regulation de l'angle                              */
	/****************************************************************************/

	pid_init(&robot.angle_pid);
	pid_set_gains(&robot.angle_pid, 500, 0, 2000);
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
	pid_set_gains(&robot.distance_pid, 600, 0, 600); /* Regles les gains du PID. */
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
	trajectory_set_speed(&robot.traj, speed_mm2imp(&robot.traj, 700), speed_rd2imp(&robot.traj, 2*M_PI) ); /* distance, angle */
	trajectory_set_acc(&robot.traj, acc_mm2imp(&robot.traj, 800), acc_rd2imp(&robot.traj, 2*M_PI));
	/* distance window, angle window, angle start */
	trajectory_set_windows(&robot.traj, 30., 1.0, 45.); // Prod

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
	//NOTICE(ERROR_CS, __FUNCTION__);

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

	/* Gestion du blocage */
	bd_manage(&robot.angle_bd);
	bd_manage(&robot.distance_bd);
}
