#include "cvra_cs.h"
#include "strat.h"
#include <string.h>
#include <2wheels/trajectory_manager_utils.h>
#include <scheduler.h>
#include <aversive/error.h>

struct strat_info strat;

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
    DEBUG(E_STRAT, "Doing first glasses."); 
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
    strat.glasses[0].pos.x = 900; strat.glasses[0].pos.y = COLOR_Y(1550);
    strat.glasses[1].pos.x = 900; strat.glasses[1].pos.y = COLOR_Y(1050);
    strat.glasses[2].pos.x = 1050; strat.glasses[2].pos.y = COLOR_Y(1300);

    /*XXX Not sure about coordinates of 3 and 4. */
    strat.glasses[3].pos.x = 1200; strat.glasses[3].pos.y = COLOR_Y(1550);
    strat.glasses[4].pos.x = 1200; strat.glasses[4].pos.y = COLOR_Y(1050);
    strat.glasses[5].pos.x = 1350; strat.glasses[5].pos.y = COLOR_Y(1300);
    strat.glasses[6].pos.x = 1650; strat.glasses[6].pos.y = COLOR_Y(1300);
    strat.glasses[7].pos.x = 1800; strat.glasses[7].pos.y = COLOR_Y(1550);
    strat.glasses[8].pos.x = 1800; strat.glasses[8].pos.y = COLOR_Y(1050);
    strat.glasses[9].pos.x = 1950; strat.glasses[9].pos.y = COLOR_Y(1300);
    strat.glasses[10].pos.x = 2100; strat.glasses[10].pos.y = COLOR_Y(1550);
    strat.glasses[11].pos.x = 2100; strat.glasses[11].pos.y = COLOR_Y(1050);
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

	/* On fait un backup des reglages de l'asservissement. */
	struct blocking_detection backup_bd;
	struct quadramp_filter backup_qr;


	memcpy(&backup_bd, &robot.distance_bd, sizeof(backup_bd));
	memcpy(&backup_qr, &robot.distance_qr, sizeof(backup_qr));

	robot.is_aligning = 1;

	// Pour se recaler, on met le robot en regulation angulaire, on reduit la vitesse et l'acceleration
	// On diminue la sensibilite on augmente la constante de temps de detection du bloquage

	bd_set_thresholds(&robot.distance_bd, 2000, 2);

	quadramp_set_1st_order_vars(&robot.distance_qr, 1, 1);

	quadramp_set_2nd_order_vars(&robot.distance_qr, 1, 1);
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

	/* On reregle la position. */
	position_set(&robot.pos, position_get_x_s16(&robot.pos), COLOR_Y(epaisseurRobot), COLOR_A(90));

	/* On se met en place a la position demandee. */
	trajectory_d_rel(&robot.traj, (double) (y - epaisseurRobot));
	while(!trajectory_finished(&robot.traj));

	/* Pour finir on s'occuppe de l'angle. */
	trajectory_a_abs(&robot.traj, (double) COLOR_A(a));
	while(!trajectory_finished(&robot.traj));

	/* On remet le robot dans son etat initial. */
	robot.mode = BOARD_MODE_ANGLE_DISTANCE;

	memcpy(&robot.distance_bd, &backup_bd, sizeof(backup_bd));
	memcpy(&robot.distance_qr, &backup_qr, sizeof(backup_qr));
	bd_reset(&robot.distance_bd);
	bd_reset(&robot.angle_bd);

	robot.is_aligning = 0;

	trajectory_hardstop(&robot.traj);
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
