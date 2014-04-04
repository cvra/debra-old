#include <platform.h>
#include <obstacle_avoidance.h>
#include <aversive/error.h>
#include <2wheels/trajectory_manager_utils.h>
#include <cvra_beacon.h>
#include "strat_utils.h"
#include "cvra_cs.h"


struct strat_info strat;

int strat_get_time(void)
{
    return (uptime_get() - strat.time_start) / 1000000;
}

void strat_wait_ms(int ms)
{
    int32_t time = uptime_get();
    while(uptime_get() < time + ms*1000);
}

void strat_timer_reset(void)
{
    strat.time_start = uptime_get();
}

void strat_autopos(int16_t x, int16_t y, int16_t a, int16_t epaisseurRobot)
{

    robot.is_aligning = 1;

    // Pour se recaler, on met le robot en regulation angulaire, on reduit la vitesse et l'acceleration
    // On diminue la sensibilite on augmente la constante de temps de detection du bloquage

    bd_set_thresholds(&robot.distance_bd,  5000, 2);

    trajectory_set_speed(&robot.traj, 100, 100);
    robot.mode = BOARD_MODE_DISTANCE_ONLY;

    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);
    wait_traj_end(END_BLOCKING);

    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    position_set(&robot.pos, epaisseurRobot, 0, COLOR_A(0.));
    /* On se mets a la bonne position en x. */
    trajectory_d_rel(&robot.traj, (double) (x - epaisseurRobot));
    wait_traj_end(END_TRAJ);

    /* On se tourne face a la paroi en Y. */
    trajectory_only_a_abs(&robot.traj, COLOR_A(90));
    wait_traj_end(END_TRAJ);

    /* On recule jusqu'a avoir touche le bord. */

    trajectory_d_rel(&robot.traj, (double) -2000);
    wait_traj_end(END_BLOCKING);

    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);

    /* On reregle la position. */
    robot.pos.pos_d.y = COLOR_Y(epaisseurRobot);
    robot.pos.pos_s16.y = COLOR_Y(epaisseurRobot);

    /* On se met en place a la position demandee. */
    trajectory_set_speed(&robot.traj, speed_mm2imp(&robot.traj, 300), speed_rd2imp(&robot.traj, 2.5));

    trajectory_d_rel(&robot.traj, (double) (y - epaisseurRobot));
    wait_traj_end(END_TRAJ);

    /* Pour finir on s'occuppe de l'angle. */
    trajectory_a_abs(&robot.traj, (double)a);
    wait_traj_end(END_TRAJ);

    /* On remet le robot dans son etat initial. */
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    robot.is_aligning = 0;
}

/** Converts relative angle/distance coordinates to absolute. */
void strat_da_rel_to_xy_abs(float a_deg, float distance_mm, int *x_mm, int *y_mm)
{
    *x_mm = distance_mm * cos(RAD(a_deg)) + position_get_x_s16(&robot.pos);
    *y_mm = distance_mm * sin(RAD(a_deg)) + position_get_y_s16(&robot.pos);
}

void create_opp_polygon(poly_t *pol, int x, int y)
{
    const int width = 600; // half width XXX check this, it should be greater IMHO
    const int height = width;

    oa_poly_set_point(pol, x+width, y+width, 0);
    oa_poly_set_point(pol, x+width, y-width, 1);
    oa_poly_set_point(pol, x-width, y-width, 2);
    oa_poly_set_point(pol, x-width, y+width, 3);
}

int strat_goto_avoid(int x, int y, int flags)
{
    int i, len, ret;
    int retry_count;
    int opp_x, opp_y;
    poly_t *pol_opp;
    point_t *p;
    oa_init();

    /* The robot will try 3 times before giving up. */
    for(retry_count=0;retry_count<3;retry_count++) {

        /* Creates one polygon for each opponent robot. */
        for(i=0;i<robot.beacon.nb_beacon;i++) {
            pol_opp = oa_new_poly(4);

            strat_da_rel_to_xy_abs(robot.beacon.beacon[i].direction, robot.beacon.beacon[i].distance*10,
                   &opp_x, &opp_y);

            NOTICE(0, "Op is at %d;%d", opp_x, opp_y);
            create_opp_polygon(pol_opp, 800, 0);

            /* Checks if the arrival point is in an opponent. */
            if(is_point_in_poly(pol_opp, x, y)) {
                WARNING(0, "Destination point is in opponent.");
                return END_ERROR;
            }
        }

        /* Sets starting and ending point of the path. */
        oa_start_end_points(position_get_x_s16(&robot.pos), position_get_x_s16(&robot.pos), x, y);

        /* Computes the path */
        len = oa_process();

        /* Checks if a path was found. */
        if(len == 0) {
            WARNING(0, "Cannot find a suitable path.");
            return END_ERROR;
        }

        p = oa_get_path();
        /* For all the points in the path. */
        for(i=0;i<len;i++) {
            /* Goes to the point. */
            trajectory_goto_forward_xy_abs(&robot.traj, p->x, p->y);

            /* Waits for the completion of the trajectory. */
            ret = wait_traj_end(flags);

            /* If we were blocked or met an obstacle, we will retry. */
            if(ret == END_BLOCKING || ret == END_OBSTACLE) {
                WARNING(0, "Retry");
                break; // we retry once more
            } else if(!TRAJ_SUCCESS(ret)) {
                /* If it was an other error, we simply abort and return it. */
                WARNING(0, "Unknown error code : %d", ret);
                return ret;
            }

            /* Increments pointer to load next point. */
            p++;
        }

        /* If we reached last point, no need to retry. */
        if(ret == END_TRAJ)
            return END_TRAJ;
    }

    /* If we reach here, it means 3 try were not enough. */
    return END_ERROR;
}

int test_traj_end(int why)
{

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
            if(robot.beacon.beacon[i].distance < 60) { /*cm*/
                if(robot.distance_qr.previous_var > 0) {
                    /* Going forward. */
                    if(robot.beacon.beacon[i].direction > -30 && robot.beacon.beacon[i].direction < 30) {
                        trajectory_stop(&robot.traj);
                        while(robot.distance_qr.previous_var > 0);
                        trajectory_hardstop(&robot.traj);
                        bd_reset(&robot.distance_bd);
                        return END_OBSTACLE;
                    }
                } else if(robot.distance_qr.previous_var < 0) {
                    /* Going backward */
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
        bd_reset(&robot.distance_bd);
        bd_reset(&robot.angle_bd);
        return END_BLOCKING;
    }

    if((why & END_BLOCKING) && bd_get(&robot.angle_bd)) {
        trajectory_hardstop(&robot.traj);

        bd_reset(&robot.distance_bd);
        bd_reset(&robot.angle_bd);
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

void right_pump(int status){
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
