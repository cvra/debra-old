#include <uptime.h> //< pseudo ok
#include <string.h> //< OK

#include "arm.h" // OK
#include "arm_cinematics.h" // Ok
#include <math.h> // ok


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

static void arm_set_physical_parameters(arm_t *arm)
{
#if 0
    /* Physical constants, not magic numbers. */
    arm->length[0] = 135.5; /* mm */
    arm->length[1] = 136;

    pid_set_gains(&arm->z_axis_pid, 2250, 0, 100);
    pid_set_gains(&arm->elbow_pid, 30, 0, 0);
    pid_set_gains(&arm->shoulder_pid, 30, 0, 0);

    pid_set_out_shift(&arm->z_axis_pid, 12);
    pid_set_out_shift(&arm->shoulder_pid, 6);
    pid_set_out_shift(&arm->elbow_pid, 6);

    arm->z_axis_imp_per_mm = 655*4;
    arm->shoulder_imp_per_rad = -77785;
    arm->elbow_imp_per_rad = -56571;
#endif
}

void arm_init(arm_t *arm)
{
    memset(arm, 0, sizeof(arm_t));

    arm_cs_init_loop(&arm->shoulder);
    arm_cs_init_loop(&arm->elbow);
    arm_cs_init_loop(&arm->z_axis);

    /* Sets last loop run date for lag compensation. */
    arm->last_loop = uptime_get();

    arm->shoulder_mode = SHOULDER_BACK;
}

#if 0
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

int arm_trajectory_finished(arm_t *arm) {
   if(arm->trajectory.frame_count == 0) {
        return 1;
   }

   if(uptime_get() > arm->trajectory.frames[arm->trajectory.frame_count-1].date) {
        return 1;
   }

   return 0;
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
#endif
