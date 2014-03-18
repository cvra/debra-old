#include <aversive.h>
#include "arm_interpolators.h"
#include <uptime.h>
#include <stdlib.h>


void arm_trajectory_init(arm_trajectory_t *traj) {
    memset(traj, 0, sizeof(arm_trajectory_t));
}


void arm_interpolator_linear_motion(arm_trajectory_t *traj, const float start[3], const float end[3],
                                    arm_coordinate_t system, const float duration) {

    int32_t time = uptime_get();

    if(traj == NULL)
        return;


    traj->frame_count = 2;
    traj->frames = malloc(2*sizeof(arm_keyframe_t));


    traj->frames[0].coordinate_type = COORDINATE_ARM;
    traj->frames[0].position[0] = start[0];
    traj->frames[0].position[1] = start[1];
    traj->frames[0].position[2] = start[2];

    traj->frames[0].date = time;

    traj->frames[0].length[0] = 135;
    traj->frames[0].length[1] = 100;



    traj->frames[1].coordinate_type = system;
    traj->frames[1].position[0] = end[0];
    traj->frames[1].position[1] = end[1];
    traj->frames[1].position[2] = end[2];

    traj->frames[1].date = time+(int)(duration * 1e6);


    traj->frames[1].length[0] = 135;
    traj->frames[1].length[1] = 100;
}

void arm_interpolator_append_point(arm_trajectory_t *traj, const float x, const float y, const float z,
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
    traj->frames[traj->frame_count-1].date = traj->frames[traj->frame_count-2].date+1000000*duration;

    traj->frames[traj->frame_count-1].length[0] = 135.5;
    traj->frames[traj->frame_count-1].length[1] = 136;
    traj->frames[traj->frame_count-1].angle_offset = 0.;
}

void arm_interpolator_append_point_with_length(arm_trajectory_t *traj, const float x, const float y, const float z,
                                   arm_coordinate_t system, const float duration, const float l1, const float l2) {
    arm_interpolator_append_point(traj, x, y, z, system, duration);

    traj->frames[traj->frame_count-1].length[0] = l1;
    traj->frames[traj->frame_count-1].length[1] = l2;
}


