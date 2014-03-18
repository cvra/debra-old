#include <aversive.h>
#include <uptime.h>
#include <string.h>
#include <circles.h>
#include <vect2.h>
#include <2wheels/position_manager.h>
#include <cvra_dc.h>
#include <aversive/error.h>
#include "cvra_cs.h"
#include <scheduler.h>
#include <aversive/error.h>

#include "adresses.h"

#include "arm.h"
#include <math.h>

/** Computes the inverse cinematics of an arm.
 *
 * The computation is done using the simplest algorithm : The intersection of
 * two circles, one being centered on shoulder, the other being centered on
 * target coordinates.
 *
 * When the intersection is known, computing angles is a simple matter of using
 * atan2 in a correct way.
 *
 * @param [in] arm The arm instance.
 * @param [in] x, y, z The coordinates of the target.
 * @param [out] alpha, beta Pointers where angles will be stored.
 *
 * @return 0 en cas de succes, -1 en cas d'erreur.
 *
 * @todo We need an algorithm to choose between the 2 positions in case there
 * are multiple possibilities.
 */
static int compute_inverse_cinematics(arm_t *arm, float x, float y, float *alpha, float *beta, const float l1, const float l2);

inline float smoothstep(float t)
{
    if(t < 0.0f) return 0.0f;
    if(t > 1.0f) return 1.0f;
    return t*t*t*(t*(6.0f*t-15.0f)+10.0f);
}

float interpolate(float t, float a, float b)
{
    return (1 - t) * a + t * b;
}

static void arm_init_control_loops(arm_t *arm)
{
    cs_init(&arm->z_axis_cs);
    pid_init(&arm->z_axis_pid);
    pid_set_out_shift(&arm->z_axis_pid, 12);
    cs_set_correct_filter(&arm->z_axis_cs, pid_do_filter, &arm->z_axis_pid);

    cs_init(&arm->shoulder_cs);
    pid_init(&arm->shoulder_pid);
    pid_set_out_shift(&arm->shoulder_pid, 6);
    cs_set_correct_filter(&arm->shoulder_cs, pid_do_filter, &arm->shoulder_pid);

    cs_init(&arm->elbow_cs);
    pid_init(&arm->elbow_pid);
    pid_set_out_shift(&arm->elbow_pid, 6);
    cs_set_correct_filter(&arm->elbow_cs, pid_do_filter, &arm->elbow_pid);
}

static void arm_set_physical_parameters(arm_t *arm)
{
    /* Physical constants, not magic numbers. */
    arm->length[0] = 135.5; /* mm */
    arm->length[1] = 136;

    pid_set_gains(&arm->z_axis_pid, 2250, 0, 100);
    pid_set_gains(&arm->elbow_pid, 30, 0, 0);
    pid_set_gains(&arm->shoulder_pid, 30, 0, 0);

    arm->z_axis_imp_per_mm = 655*4;
    arm->shoulder_imp_per_rad = -77785;
    arm->elbow_imp_per_rad = -56571;
}

void arm_highlevel_init(void) {
#ifdef COMPILE_ON_ROBOT
    int i;
    for(i=0;i<6;i++) {
        cvra_dc_set_pwm(ARMSMOTORCONTROLLER_BASE, i, 0);
        cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, i, 0);
    }

    cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, 1, 265 * robot.left_arm.z_axis_imp_per_mm);
    cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, 4, 265 * robot.right_arm.z_axis_imp_per_mm);
#endif
    arm_init(&robot.left_arm);
    arm_init(&robot.right_arm);

    robot.left_arm.offset_xy.x = 0; robot.left_arm.offset_xy.y = 79;
    robot.right_arm.offset_xy.x = 0; robot.right_arm.offset_xy.y = -79;

    robot.left_arm.offset_rotation = M_PI / 2.;
    robot.right_arm.offset_rotation = -M_PI / 2.;


    arm_connect_io(&robot.left_arm,
            /* Z */
            cvra_dc_set_pwm1, ARMSMOTORCONTROLLER_BASE,
            cvra_dc_get_encoder1, ARMSMOTORCONTROLLER_BASE,
            /* Shoulder */
            cvra_dc_set_pwm0, ARMSMOTORCONTROLLER_BASE,
            cvra_dc_get_encoder0, ARMSMOTORCONTROLLER_BASE,
            /* Elbow */
            cvra_dc_set_pwm2, ARMSMOTORCONTROLLER_BASE,
            cvra_dc_get_encoder2, ARMSMOTORCONTROLLER_BASE);

    arm_connect_io(&robot.right_arm,
            /* Z */
            cvra_dc_set_pwm4, ARMSMOTORCONTROLLER_BASE,
            cvra_dc_get_encoder4, ARMSMOTORCONTROLLER_BASE,
            /* Shoulder */
            cvra_dc_set_pwm5, ARMSMOTORCONTROLLER_BASE,
            cvra_dc_get_encoder5, ARMSMOTORCONTROLLER_BASE,
            /* Elbow */
            cvra_dc_set_pwm3, ARMSMOTORCONTROLLER_BASE,
            cvra_dc_get_encoder3, ARMSMOTORCONTROLLER_BASE);
}

void arm_init(arm_t *arm) {
    memset(arm, 0, sizeof(arm_t));

    arm_init_control_loops(arm);
    arm_set_physical_parameters(arm);

    /* Sets last loop run date for lag compensation. */
    arm->last_loop = uptime_get();

    /* Needs to be reworked for UC/OS-II, because we cannot have one task per arm with the same prio */
#if 0
    scheduler_add_periodical_event(arm_manage_cs, (void *)arm, 1000 / SCHEDULER_UNIT);
    scheduler_add_periodical_event(arm_manage, (void *)arm, 10000 / SCHEDULER_UNIT);
#endif

    arm->shoulder_mode = SHOULDER_BACK;
}

void arm_connect_io(arm_t *arm,
                    void (*z_set_pwm)(void *, int32_t), void *z_set_pwm_param,
                    int32_t (*z_get_coder)(void *), void *z_get_coder_param,
                    void (*shoulder_set_pwm)(void *, int32_t), void *shoulder_set_pwm_param,
                    int32_t (*shoulder_get_coder)(void *), void *shoulder_get_coder_param,
                    void (*elbow_set_pwm)(void *, int32_t), void *elbow_set_pwm_param,
                    int32_t (*elbow_get_coder)(void *), void *elbow_get_coder_param) {

    cs_set_process_in(&arm->z_axis_cs, z_set_pwm, z_set_pwm_param);
    cs_set_process_out(&arm->z_axis_cs, z_get_coder, z_get_coder_param);
    cs_set_process_in(&arm->shoulder_cs, shoulder_set_pwm, shoulder_set_pwm_param);
    cs_set_process_out(&arm->shoulder_cs, shoulder_get_coder, shoulder_get_coder_param);
    cs_set_process_in(&arm->elbow_cs, elbow_set_pwm, elbow_set_pwm_param);
    cs_set_process_out(&arm->elbow_cs, elbow_get_coder, elbow_get_coder_param);
}

void arm_execute_movement(arm_t *arm, arm_trajectory_t *traj)
{
    arm_shutdown(arm);

    arm->trajectory.frames = malloc(traj->frame_count * sizeof(arm_keyframe_t));

    if (arm->trajectory.frames == NULL)
        panic();

    arm->trajectory.frame_count = traj->frame_count;
    memcpy(arm->trajectory.frames, traj->frames, sizeof(arm_keyframe_t) * traj->frame_count);
}

/** @brief Interpolates 2 arms frames and outputs result in arm coordinate frame.
 * @param [in] arm The arm to use for interpolation.
 * @param [in] date The time to use for interpolation.
 * @param [out] position The interpolated position.
 * @param [out] length The interpolated length.
 * */
static void arm_interpolate_frames(arm_t *arm, int32_t date, float *position, float *length)
{
    float t; /* interpolation factor, between 0 and 1 */
    int i = 1;
    float previous_z, next_z;

    /* The coordinates of the previous frames in arm coordinates.
     * This allows us to mix different coordinate systems in a single trajectory. */
    float previous_frame_xy[2], next_frame_xy[2];
    float *previous_length, *next_length;

    /* Are we before the first frame ? */
    if (date < arm->trajectory.frames[0].date) {
        arm_change_coordinate_system(arm, arm->trajectory.frames[0].position[0], arm->trajectory.frames[0].position[1],
                arm->trajectory.frames[0].coordinate_type, &position[0], &position[1]);
        position[2] = arm->trajectory.frames[0].position[2];

        length[0] = arm->trajectory.frames[0].length[0];
        length[1] = arm->trajectory.frames[0].length[1];
        return;
    }

    if (date > arm->trajectory.frames[arm->trajectory.frame_count-1].date) {
        int f = arm->trajectory.frame_count-1;
        arm_change_coordinate_system(arm, arm->trajectory.frames[f].position[0], arm->trajectory.frames[f].position[1],
                arm->trajectory.frames[f].coordinate_type, &position[0], &position[1]);
        position[2] = arm->trajectory.frames[f].position[2];

        length[0] = arm->trajectory.frames[f].length[0];
        length[1] = arm->trajectory.frames[f].length[1];
        return;
    }

    /* We are between frame i-1 et i. i > 1 */
    while(arm->trajectory.frames[i].date < date)
        i++;


    /* Changes the coordinate systems to arm coordinates */
    arm_change_coordinate_system(arm, arm->trajectory.frames[i-1].position[0],
            arm->trajectory.frames[i-1].position[1],
            arm->trajectory.frames[i-1].coordinate_type,
            &previous_frame_xy[0], &previous_frame_xy[1]);

    arm_change_coordinate_system(arm, arm->trajectory.frames[i].position[0],
            arm->trajectory.frames[i].position[1],
            arm->trajectory.frames[i].coordinate_type,
            &next_frame_xy[0], &next_frame_xy[1]);

    previous_z = arm->trajectory.frames[i-1].position[2];
    next_z = arm->trajectory.frames[i].position[2];

    previous_length = arm->trajectory.frames[i-1].length;
    next_length = arm->trajectory.frames[i].length;

    /* Smoothstep interpolation between the 2 frames. */
    t = date - arm->trajectory.frames[i-1].date;
    t = t / (float)(arm->trajectory.frames[i].date - arm->trajectory.frames[i-1].date);
    t = smoothstep(t);

    position[0] = interpolate(t, previous_frame_xy[0], next_frame_xy[0]);
    position[1] = interpolate(t, previous_frame_xy[1], next_frame_xy[1]);
    position[2] = interpolate(t, previous_z, next_z);
    length[0] = interpolate(t, previous_length[0], next_length[0]);
    length[1] = interpolate(t, previous_length[1], next_length[1]);
}

void arm_manage(void *a) {

    arm_t *arm = (arm_t *) a;
    int32_t current_date = uptime_get();
    float position[3];

    float length[2];

    float alpha, beta; /* The angles of the arms. */

    /* Lag compensation */
    int32_t compensated_date = 2*current_date - arm->last_loop;

    /* If we dont have a trajectory data, disabled everything. */
    if (arm->trajectory.frame_count == 0) {
        cs_disable(&arm->z_axis_cs);
        cs_disable(&arm->shoulder_cs);
        cs_disable(&arm->elbow_cs);
        arm->last_loop = current_date;
        return;
    }

    arm_interpolate_frames(arm, compensated_date, position, length);

    /* Computes the inverse cinematics and send the consign to the control systems. */
    if (compute_inverse_cinematics(arm, position[0], position[1], &alpha, &beta, length[0], length[1]) < 0) {
        cs_disable(&arm->z_axis_cs);
        cs_disable(&arm->shoulder_cs);
        cs_disable(&arm->elbow_cs);
        arm->last_loop = current_date;
        return;
    }

    cs_set_consign(&arm->z_axis_cs, position[2] * arm->z_axis_imp_per_mm);
    cs_set_consign(&arm->shoulder_cs, alpha * arm->shoulder_imp_per_rad);
    cs_set_consign(&arm->elbow_cs, beta * arm->elbow_imp_per_rad);

    cs_enable(&arm->z_axis_cs);
    cs_enable(&arm->shoulder_cs);
    cs_enable(&arm->elbow_cs);

    arm->last_loop = current_date;
}


void arm_manage_cs(void *a) {
    arm_t *arm = (arm_t *)a;

    cs_manage(&arm->z_axis_cs);
    cs_manage(&arm->shoulder_cs);
    cs_manage(&arm->elbow_cs);
    /* XXX Insert the blocking managers. */
}


void arm_get_position(arm_t *arm, float *x, float *y, float *z) {
    float alpha, beta;
    alpha = (float)(arm->shoulder_cs.process_out(arm->shoulder_cs.process_out_params)) / arm->shoulder_imp_per_rad;

    beta = (float)(arm->elbow_cs.process_out(arm->elbow_cs.process_out_params)) / arm->elbow_imp_per_rad;


    if(x)
        *x = arm->length[0] * cos(alpha) + arm->length[1] * cos(alpha + beta);

    if(y)
        *y = arm->length[0] * sin(alpha) + arm->length[1] * sin(alpha + beta);

    if(z)
        *z = (float)(arm->z_axis_cs.process_out(arm->shoulder_cs.process_out_params)) / arm->z_axis_imp_per_mm;

}

int arm_trajectory_finished(arm_t *arm) {
   if(arm->trajectory.frame_count == 0) {
        return 1;
   }

   if(uptime_get() > arm->trajectory.frames[arm->trajectory.frame_count-1].date) {
        return 1;
   }

   return 0;
}

void arm_shutdown(arm_t *arm)
{
    if (arm->trajectory.frame_count != 0) {
        free(arm->trajectory.frames);
        arm->trajectory.frames = NULL;
        arm->trajectory.frame_count = 0;
    }
}


static int compute_inverse_cinematics(arm_t *arm, float x, float y, float *alpha, float *beta, const float l1, const float l2) {
    circle_t c1, c2;
    point_t p1, p2, chosen;

    c1.x = 0;
    c1.y = 0;
    c1.r = l1;

    c2.x = x;
    c2.y = y;
    c2.r = l2;

    int nbPos = circle_intersect(&c1, &c2, &p1, &p2);

    if(nbPos == 0) {
        /* It means there is no way to find an intersection... */
        return -1;
    }

    /* Checks if one of the two possibilities crosses an obstacle. */
    else if(nbPos == 2) {
        if(x < 0.) {
            if(p1.x > 0.)
                chosen = p1;
            else if(p2.x > 0.)
                chosen = p2;
            else
                chosen = p1;
        }
        else {
            if(arm->offset_rotation > 0.) {
                if(arm->shoulder_mode == SHOULDER_BACK) {
                    /* TODO the left arm is mirrored when compared to the right arm. */
                    if(p2.y > p1.y)
                        chosen = p2;
                    else
                        chosen = p1;
                }
                else {
                    /* TODO the left arm is mirrored when compared to the right arm. */
                    if(p2.y < p1.y)
                        chosen = p2;
                    else
                        chosen = p1;
                }
            }
            else {
                if(arm->shoulder_mode == SHOULDER_FRONT) {
                    /* TODO the left arm is mirrored when compared to the right arm. */
                    if(p2.y > p1.y)
                        chosen = p2;
                    else
                        chosen = p1;
                }
                else {
                    /* TODO the left arm is mirrored when compared to the right arm. */
                    if(p2.y < p1.y)
                        chosen = p2;
                    else
                        chosen = p1;
                }
            }
        }
    }
    else {
        chosen = p1;
    }


    *alpha = atan2f(chosen.y, chosen.x);
    *beta = atan2f(y-chosen.y, x-chosen.x);
    *beta = *beta - *alpha;


    if(*beta < -M_PI) {
        *beta = 2*M_PI + *beta;
    }

    if(*beta > M_PI) {
        *beta = -2*M_PI + *beta;
    }


    return 0;
}

void arm_change_coordinate_system(arm_t *arm, float x, float y,
             arm_coordinate_t system, float *arm_x, float *arm_y) {

    if(system == COORDINATE_ARM) {
        *arm_x = x;
        *arm_y = y;
    }
    else if(system == COORDINATE_ROBOT) {
        vect2_cart target;
        vect2_pol target_pol;
        target.x = x;
        target.y = y;
        vect2_sub_cart(&target, &arm->offset_xy, &target);
        vect2_cart2pol(&target, &target_pol);
        target_pol.theta -= arm->offset_rotation;
        vect2_pol2cart(&target_pol, &target);
        *arm_x = target.x;
        *arm_y = target.y;
    }
    else {
        vect2_cart target;
        vect2_pol target_pol;
        target.x = x - position_get_x_float(&robot.pos);
        target.y = y - position_get_y_float(&robot.pos);
        vect2_cart2pol(&target, &target_pol);
        /* XXX Not sure if it is -= or += here. */
        target_pol.theta -= position_get_a_rad_float(&robot.pos);
        vect2_pol2cart(&target_pol, &target);

        /* Coordinate are now in robot coordinate. */
        arm_change_coordinate_system(arm, target.x, target.y, COORDINATE_ROBOT,
            arm_x, arm_y);
    }
}

void arm_calibrate(void) {
    /* Z axis. */
    cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, 1, 183 * robot.left_arm.z_axis_imp_per_mm);
    cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, 4, 183 * robot.left_arm.z_axis_imp_per_mm);

    /* Shoulders. */
    cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, 0, (M_PI * -53.92 / 180.) * robot.left_arm.shoulder_imp_per_rad);
    cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, 5, (M_PI * 55.42 / 180.) * robot.right_arm.shoulder_imp_per_rad);


    /* Elbows. */
    cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, 2, (M_PI * -157.87 / 180.) * robot.left_arm.elbow_imp_per_rad);
    cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, 3, (M_PI * 157.87/ 180.) * robot.right_arm.elbow_imp_per_rad);

}
