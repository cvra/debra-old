#include <aversive.h>
#include "arm_interpolators.h"
#include <uptime.h>
#include <stdlib.h>


void arm_interpolator_linear_motion(arm_trajectory_t *traj, const float start[3], const float end[3], 
                                    const float duration) {

    int32_t time = uptime_get();

    if(traj == NULL)
        return;


    traj->frame_count = 2;
    traj->frames = malloc(2*sizeof(arm_keyframe_t));


    traj->frames[0].coordinate_type = COORDINATE_TABLE;
    traj->frames[0].position[0] = start[0];
    traj->frames[0].position[1] = start[1];
    traj->frames[0].position[2] = start[2];

    traj->frames[0].date = time; 



    traj->frames[1].coordinate_type = COORDINATE_TABLE;
    traj->frames[1].position[0] = end[0]; 
    traj->frames[1].position[1] = end[1];
    traj->frames[1].position[2] = end[2];

    traj->frames[1].date = time+(int)(duration * 1e6); 
}
