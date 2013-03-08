#include <aversive.h>
#include "arm_interpolators.h"
#include <uptime.h>
#include <stdlib.h>


void arm_interpolator_linear_motion(arm_trajectory_t *traj, const float start[3], const float end[3], 
                                    const float duration) {

    if(traj == NULL)
        return;

    if(traj->frame_count != 0) {
        free(traj->frames);
    }

    traj->frame_count = 2;
    traj->frames = malloc(2*sizeof(arm_keyframe_t));

    traj->frames[0].position[0] = start[0];
    traj->frames[0].position[1] = start[1];
    traj->frames[0].position[2] = start[2];
    traj->frames[0].date = uptime_get();

    traj->frames[1].position[0] = end[0]; 
    traj->frames[1].position[1] = end[1];
    traj->frames[1].position[2] = end[2];
    traj->frames[1].date = uptime_get() + (int)(duration * 1e6); 
}
