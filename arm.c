#include <uptime.h>
#include <string.h>
#include <math.h>

#include "arm.h"
#include "arm_cinematics.h"
#include "arm_trajectories.h"
#include "arm_utils.h"


void arm_set_physical_parameters(arm_t *arm)
{
    /* Physical constants, not magic numbers. */
    arm->length[0] = 135.16; /* mm */
    arm->length[1] = 106.5;

    pid_set_gains(&arm->z_axis.pid, 1200, 0, 0);
    pid_set_gains(&arm->elbow.pid, 10, 0, 0);
    pid_set_gains(&arm->shoulder.pid, 10, 0, 0);
    pid_set_gains(&arm->hand.pid, 10, 0, 0);

    pid_set_out_shift(&arm->z_axis.pid, 12);
    pid_set_out_shift(&arm->shoulder.pid, 6);
    pid_set_out_shift(&arm->hand.pid, 10);
    pid_set_out_shift(&arm->elbow.pid, 6);

    arm->z_axis_imp_per_mm = 655*4;
    arm->shoulder_imp_per_rad = -77785;
    arm->elbow_imp_per_rad = -56571;
}


void arm_get_position(arm_t *arm, float *x, float *y, float *z)
{
    float alpha, beta;

    alpha = cs_get_feedback(&arm->shoulder.manager) / (float)arm->shoulder_imp_per_rad;
    beta = cs_get_feedback(&arm->elbow.manager) / (float)arm->elbow_imp_per_rad;

    beta += alpha;

    if (x)
        *x = arm->length[0] * cos(alpha) + arm->length[1] * cos(beta);

    if (y)
        *y = arm->length[0] * sin(alpha) + arm->length[1] * sin(beta);

    if (z)
        *z = cs_get_feedback(&arm->z_axis.manager) / (float)arm->z_axis_imp_per_mm;
}


void arm_init(arm_t *arm)
{
    memset(arm, 0, sizeof(arm_t));

    arm_cs_init_loop(&arm->shoulder);
    arm_cs_init_loop(&arm->hand);
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

    platform_take_semaphore(&arm->trajectory_semaphore);

    if (arm->trajectory.frame_count == 0) {
        cs_disable(&arm->shoulder.manager);
        cs_disable(&arm->elbow.manager);
        cs_disable(&arm->z_axis.manager);
        cs_disable(&arm->hand.manager);
        arm->last_loop = current_date;
        platform_signal_semaphore(&arm->trajectory_semaphore);
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
        cs_disable(&arm->hand.manager);
        arm->last_loop = current_date;
        platform_signal_semaphore(&arm->trajectory_semaphore);
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
        beta = beta - 2 * M_PI;

    cs_enable(&arm->shoulder.manager);
    cs_enable(&arm->elbow.manager);
    cs_enable(&arm->z_axis.manager);
    cs_enable(&arm->hand.manager);

    cs_set_consign(&arm->shoulder.manager, alpha * arm->shoulder_imp_per_rad);
    cs_set_consign(&arm->elbow.manager, beta * arm->elbow_imp_per_rad);
    cs_set_consign(&arm->z_axis.manager, frame.position[2] * arm->z_axis_imp_per_mm);
    cs_set_consign(&arm->hand.manager, frame.hand_angle * 432);

    arm->last_loop = uptime_get();

    platform_signal_semaphore(&arm->trajectory_semaphore);
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
        robot_pos.y = position_get_y_float(arm->robot_pos);
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

void arm_shutdown(arm_t *arm)
{
    platform_take_semaphore(&arm->trajectory_semaphore);
    arm_trajectory_delete(&arm->trajectory);
    platform_signal_semaphore(&arm->trajectory_semaphore);
}
