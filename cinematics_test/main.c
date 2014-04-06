#include <stdio.h>
#include "../arm.h"
#include "../arm_trajectories.h"
#include "uptime.h"

arm_t arm;
arm_trajectory_t traj;

void setup()
{
    arm_init(&arm);
    arm_set_physical_parameters(&arm);
    arm.offset_rotation = M_PI / 2;
    arm_trajectory_init(&traj);
}

void main(void)
{
    float alpha, beta;
    arm_keyframe_t frame;

    point_t target, p1, p2;
    int position_count;

    uptime_set(10);

    setup();

    arm_trajectory_append_point(&traj, 200, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 100, -10, 10, COORDINATE_ARM, 10.);
    arm_do_trajectory(&arm, &traj);

    while (uptime_get() < traj.frames[traj.frame_count-1].date) {
        arm_manage(&arm);

        alpha = arm.shoulder.manager.consign_value / (float)arm.shoulder_imp_per_rad;
        beta = arm.elbow.manager.consign_value / (float)arm.elbow_imp_per_rad;

        frame = arm_position_for_date(&arm, uptime_get());

        target.x = frame.position[0];
        target.y = frame.position[1];
        position_count = compute_possible_elbow_positions(target, frame.length[0], frame.length[1], &p1, &p2);
        if (position_count == 2) {
            shoulder_mode_t mode;
            mode = mode_for_orientation(arm.shoulder_mode, arm.offset_rotation);
            p1 = choose_shoulder_solution(target, p1, p2, mode);
        }

        printf("%d;", uptime_get());
        printf("%f;%f;", alpha, beta);
        printf("%f;%f;", frame.position[0], frame.position[1]);
        printf("%f;%f;", p1.x, p1.y);
        printf("%d;", position_count);
        printf("\n");

        uptime_set(uptime_get() + 200*1000);
    }
}
