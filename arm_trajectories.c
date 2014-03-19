#include "arm_trajectories.h"
#include <stdlib.h>
#include <string.h>
#include <uptime.h>


void arm_trajectory_init(arm_trajectory_t *traj) {
    memset(traj, 0, sizeof(arm_trajectory_t));
}


void arm_trajectory_append_point(arm_trajectory_t *traj, const float x, const float y, const float z,
                                   arm_coordinate_t system, const float duration)
{

    traj->frame_count += 1;
    traj->frames = realloc(traj->frames, traj->frame_count*sizeof(arm_keyframe_t));

    if (traj->frames == NULL)
        panic();

    traj->frames[traj->frame_count-1].position[0] = x;
    traj->frames[traj->frame_count-1].position[1] = y;
    traj->frames[traj->frame_count-1].position[2] = z;
    traj->frames[traj->frame_count-1].coordinate_type = system;

    if (traj->frame_count == 1)
        traj->frames[0].date = uptime_get();
    else
        traj->frames[traj->frame_count-1].date = traj->frames[traj->frame_count-2].date+1000000*duration;


    traj->frames[traj->frame_count-1].length[0] = 135.5;
    traj->frames[traj->frame_count-1].length[1] = 136;
}

void arm_trajectory_append_point_with_length(arm_trajectory_t *traj, const float x, const float y, const float z,
                                   arm_coordinate_t system, const float duration, const float l1, const float l2) {
    arm_trajectory_append_point(traj, x, y, z, system, duration);

    traj->frames[traj->frame_count-1].length[0] = l1;
    traj->frames[traj->frame_count-1].length[1] = l2;
}


