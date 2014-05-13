#include "arm_trajectories.h"
#include <stdlib.h>
#include <string.h>
#include <uptime.h>

static float smoothstep(float t)
{
    if(t < 0.0f) return 0.0f;
    if(t > 1.0f) return 1.0f;
    return t*t*t*(t*(6.0f*t-15.0f)+10.0f);
}

static float interpolate(float t, float a, float b)
{
    return (1 - t) * a + t * b;
}

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


    traj->frames[traj->frame_count-1].length[0] = 135.16;
    traj->frames[traj->frame_count-1].length[1] = 106.5;

    if (traj->frame_count > 1)
        traj->frames[traj->frame_count-1].hand_angle = traj->frames[traj->frame_count-2].hand_angle;
    else
        traj->frames[traj->frame_count-1].hand_angle = 0;
}

void arm_trajectory_append_point_with_length(arm_trajectory_t *traj, const float x, const float y, const float z,
                                   arm_coordinate_t system, const float duration, const float l1, const float l2) {
    arm_trajectory_append_point(traj, x, y, z, system, duration);

    traj->frames[traj->frame_count-1].length[0] = l1;
    traj->frames[traj->frame_count-1].length[1] = l2;
}

void arm_trajectory_delete(arm_trajectory_t *traj)
{
    if (traj->frame_count != 0) {
        free(traj->frames);
        traj->frames = NULL;
        traj->frame_count = 0;
    }
}

void arm_trajectory_copy(arm_trajectory_t *dest, arm_trajectory_t *src)
{
    dest->frame_count = src->frame_count;
    dest->frames = malloc(dest->frame_count * sizeof(arm_keyframe_t));
    memcpy(dest->frames, src->frames, dest->frame_count * sizeof(arm_keyframe_t));
}

int arm_trajectory_finished(arm_trajectory_t *traj)
{
    int last_frame = traj->frame_count - 1;

    if (traj->frame_count == 0)
        return 1;

    if (traj->frames[last_frame].date < uptime_get())
        return 1;

    return 0;
}

void arm_trajectory_set_hand_angle(arm_trajectory_t *traj, float angle)
{
    int last_frame = traj->frame_count - 1;

    if (last_frame == -1)
        return;

    traj->frames[last_frame].hand_angle = angle;
}

arm_keyframe_t arm_trajectory_interpolate_keyframes(arm_keyframe_t k1, arm_keyframe_t k2, int32_t date)
{
    float t;
    arm_keyframe_t result;
    int i;

    result.date = date;

    t = (date - k1.date) / (float)(k2.date - k1.date);
    t = smoothstep(t);

    for (i=0; i<3; i++)
        result.position[i] = interpolate(t, k1.position[i], k2.position[i]);

    for (i=0; i<2; i++)
        result.length[i] = interpolate(t, k1.length[i], k2.length[i]);

    result.hand_angle = interpolate(t, k1.hand_angle, k2.hand_angle);

    return result;
}
