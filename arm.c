#include <uptime.h> //< pseudo ok
#include <string.h> //< OK

#include "arm.h" // OK
#include "arm_cinematics.h" // Ok
#include "arm_trajectories.h"
#include "arm_utils.h"
#include <math.h> // ok

static arm_keyframe_t arm_convert_keyframe_coordinate(arm_t *arm, arm_keyframe_t key);

void arm_set_physical_parameters(arm_t *arm)
{
    /* Physical constants, not magic numbers. */
    arm->length[0] = 135.5; /* mm */
    arm->length[1] = 136;

    pid_set_gains(&arm->z_axis.pid, 2250, 0, 100);
    pid_set_gains(&arm->elbow.pid, 30, 0, 0);
    pid_set_gains(&arm->shoulder.pid, 30, 0, 0);

    pid_set_out_shift(&arm->z_axis.pid, 12);
    pid_set_out_shift(&arm->shoulder.pid, 6);
    pid_set_out_shift(&arm->elbow.pid, 6);

    arm->z_axis_imp_per_mm = 655*4;
    arm->shoulder_imp_per_rad = -77785;
    arm->elbow_imp_per_rad = -56571;
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

    platform_create_semaphore(&arm->trajectory_semaphore, 1);
}

void arm_do_trajectory(arm_t *arm, arm_trajectory_t *traj)
{
    platform_take_semaphore(&arm->trajectory_semaphore);
    arm_trajectory_copy(&arm->trajectory, traj);
    platform_signal_semaphore(&arm->trajectory_semaphore);
}

void arm_manage(arm_t *arm)
{
    arm_keyframe_t frame;
    point_t target, p1, p2;
    int32_t current_date = uptime_get();
    int position_count;
    float alpha, beta;

    if (arm->trajectory.frame_count == 0) {
        cs_disable(&arm->shoulder.manager);
        cs_disable(&arm->elbow.manager);
        cs_disable(&arm->z_axis.manager);
        arm->last_loop = current_date;
        return;
    }

    frame = arm_position_for_date(arm, uptime_get());
    target.x = frame.position[0];
    target.y = frame.position[1];

    position_count = compute_possible_elbow_positions(target, frame.length[0], frame.length[1], &p1, &p2);

    if (position_count == 0) {
        cs_disable(&arm->shoulder.manager);
        cs_disable(&arm->elbow.manager);
        cs_disable(&arm->z_axis.manager);
        arm->last_loop = current_date;
        return;
    } else if (position_count == 2) {
        shoulder_mode_t mode;
        mode = mode_for_orientation(arm->shoulder_mode, arm->offset_rotation);
        p1 = choose_shoulder_solution(target, p1, p2, mode);
    }

    /* p1 now contains the correct elbow pos. */
    alpha = compute_shoulder_angle(p1, target);
    beta  = compute_elbow_angle(p1, target);

    /* This is due to mecanical construction of the arms. */
    beta = beta - alpha;

    /* The arm cannot make one full turn. */
    if (beta < -M_PI)
        beta = 2 * M_PI + beta;

    if (beta > M_PI)
        beta = 2 * M_PI - beta;

    cs_enable(&arm->shoulder.manager);
    cs_enable(&arm->elbow.manager);
    cs_enable(&arm->z_axis.manager);

    cs_set_consign(&arm->shoulder.manager, alpha * arm->shoulder_imp_per_rad);
    cs_set_consign(&arm->elbow.manager, beta * arm->elbow_imp_per_rad);
    cs_set_consign(&arm->z_axis.manager, frame.position[2] * arm->z_axis_imp_per_mm);

    arm->last_loop = uptime_get();
}

static arm_keyframe_t arm_convert_keyframe_coordinate(arm_t *arm, arm_keyframe_t key)
{
    point_t pos;
    pos.x = key.position[0];
    pos.y = key.position[1];
    if (key.coordinate_type == COORDINATE_TABLE) {
        point_t robot_pos;
        float robot_a_rad;

        robot_pos.x = position_get_x_float(arm->robot_pos);
        robot_pos.y = position_get_x_float(arm->robot_pos);
        robot_a_rad = position_get_a_rad_float(arm->robot_pos);
        pos = arm_coordinate_table2robot(pos, robot_pos, robot_a_rad);
        pos = arm_coordinate_robot2arm(pos, arm->offset_xy, arm->offset_rotation);
    } else if (key.coordinate_type == COORDINATE_ROBOT) {
        pos = arm_coordinate_robot2arm(pos, arm->offset_xy, arm->offset_rotation);
    }

    key.position[0] = pos.x;
    key.position[1] = pos.y;
    return key;
}

arm_keyframe_t arm_position_for_date(arm_t *arm, int32_t date)
{
    int i=0;
    arm_keyframe_t k1, k2;
    point_t pos1, pos2;

    /* If we are past last keyframe, simply return last frame. */
    if (arm->trajectory.frames[arm->trajectory.frame_count-1].date < date) {
        k1 = arm->trajectory.frames[arm->trajectory.frame_count-1];
        return arm_convert_keyframe_coordinate(arm, k1);
    }

    while (arm->trajectory.frames[i].date < date)
        i++;

    k1 = arm_convert_keyframe_coordinate(arm, arm->trajectory.frames[i-1]);
    k2 = arm_convert_keyframe_coordinate(arm, arm->trajectory.frames[i]);

    return arm_trajectory_interpolate_keyframes(k1, k2, date);
}

void arm_set_related_robot_pos(arm_t *arm, struct robot_position *pos)
{
    arm->robot_pos = pos;
}

#if 0
static void arm_interpolate_frames(arm_t *arm, int32_t date, float *position, float *length)
{
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
