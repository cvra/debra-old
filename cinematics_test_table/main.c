#include <stdio.h>
#include "../arm.h"
#include "../arm_trajectories.h"
#include "2wheels/position_manager.h"
#include "uptime.h"

arm_t arm;
arm_trajectory_t traj;

struct robot_position pos;



void setup()
{
    arm_init(&arm);
    arm_set_physical_parameters(&arm);
    arm.offset_xy.x = 0;
    arm.offset_xy.y = 87.5;
    arm.offset_rotation = M_PI / 2;
    arm_trajectory_init(&traj);
    arm_set_related_robot_pos(&arm, &pos);
}

void main(void)
{
    float alpha, beta;
    arm_keyframe_t frame;

    point_t target, p1, p2;
    int position_count;

    uptime_set(10);

    setup();

//    arm_trajectory_append_point(&traj, 10, 0, 0, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 0 , 0, COORDINATE_TABLE, 1.);
    arm_do_trajectory(&arm, &traj);
    uptime_set(10 * 1000000);

    position_set(&pos, -100, 0, 0);

    while (position_get_x_s16(&pos) < 100) {
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
        printf("%.1f;%.1f;", position_get_x_float(&pos), position_get_y_float(&pos));
        printf("%.1f;%.1f;", frame.position[0], frame.position[1]);
        printf("%.1f;%.1f;", p1.x, p1.y);
        printf("%f;%f;", alpha, beta);
        printf("%d;", position_count);
        printf("\n");

        position_set(&pos,
                position_get_x_s16(&pos) + 1,
                position_get_y_s16(&pos) , 0);

        uptime_set(uptime_get() + 2*1000);
    }
}
