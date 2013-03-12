#include <aversive.h>
#include <string.h>
#include <uptime.h>

#include "arm.h"

void arm_init(arm_t *arm, const char *name) {
    memset(arm, 0, sizeof(arm_t)); 
    strncpy(arm->name, name, 64);

    cs_init(&arm->z_axis_cs);
    pid_init(&arm->z_axis_pid);
    pid_set_maximums(&arm->z_axis_pid, 0, 5000, 30000);
    pid_set_gains(&arm->z_axis_pid, 10, 0, 0); /* FIXME Regler les gains. */
    pid_set_out_shift(&arm->z_axis_pid, 10);
    cs_set_correct_filter(&arm->z_axis_cs, pid_do_filter, &arm->z_axis_pid);

    cs_init(&arm->shoulder_cs);
    pid_init(&arm->shoulder_pid);
    pid_set_maximums(&arm->shoulder_pid,0, 5000, 30000);
    pid_set_gains(&arm->shoulder_pid, 10, 0, 0); /* FIXME Regler les gains. */
    pid_set_out_shift(&arm->shoulder_pid, 10);
    cs_set_correct_filter(&arm->shoulder_cs, pid_do_filter, &arm->shoulder_pid);

    cs_init(&arm->elbow_cs);
    pid_init(&arm->elbow_pid);
    pid_set_gains(&arm->z_axis_pid, 10, 0, 0); /* FIXME Regler les gains. */
    pid_set_maximums(&arm->z_axis_pid,0, 5000, 30000);
    pid_set_out_shift(&arm->elbow_pid, 10);
    cs_set_correct_filter(&arm->elbow_cs, pid_do_filter, &arm->elbow_pid);

    /* FIXME regler tout les parametres physiques */
    arm->length[0] = 100; /* mm */
    arm->length[1] = 200;

    arm->z_axis_imp_per_mm = 10;
    arm->shoulder_imp_per_rad = 20;
    arm->elbow_imp_per_rad = 20;

    arm->last_loop = uptime_get();
}

void arm_connect_io(arm_t *arm, 
                    void (*z_set_pwm)(void *, int32_t), void *z_set_pwm_param,
                    int32_t (*z_get_coder)(void *), void *z_get_coder_param,
                    void (*shoulder_set_pwm)(void *, int32_t), void *shoulder_set_pwm_param,
                    int32_t (*shoulder_get_coder)(void *), void *shoulder_get_coder_param,
                    void (*elbow_set_pwm)(void *, int32_t), void *elbow_set_pwm_param,
                    int32_t (*elbow_get_coder)(void *), void *elbow_get_coder_param) {

    cs_set_process_in(&arm->z_axis_cs, z_set_pwm, z_set_pwm_param);
    cs_set_process_out(&arm->z_axis_cs, z_get_coder, z_get_coder_param);
    cs_set_process_in(&arm->shoulder_cs, shoulder_set_pwm, shoulder_set_pwm_param);
    cs_set_process_out(&arm->shoulder_cs, shoulder_get_coder, shoulder_get_coder_param);
    cs_set_process_in(&arm->elbow_cs, elbow_set_pwm, elbow_set_pwm_param);
    cs_set_process_out(&arm->elbow_cs, elbow_get_coder, elbow_get_coder_param);
}

void arm_execute_movement(arm_t *arm, arm_trajectory_t *traj) {
    /** FIXME @todo Implementer la concatenation de 2 trajectoires. */
    /* Step 1 : If we had a previous trajectory in memory, release it. */
    if(arm->trajectory.frame_count != 0) {
        free(arm->trajectory.frames);
    } 

    /* Step 2 : Allocates requested memory for our copy of the trajectory. */
    arm->trajectory.frames = malloc(traj->frame_count * sizeof(arm_keyframe_t));

    if(arm->trajectory.frames == NULL)
        panic();

    /* Step 3 : Copy the trajectory data. */
    arm->trajectory.frame_count = traj->frame_count;
    memcpy(arm->trajectory.frames, traj->frames, sizeof(arm_keyframe_t) * traj->frame_count);
} 

void arm_manage(void *a) {

    arm_t *arm = (arm_t *) a;
    int32_t current_date = 2*uptime_get() - arm->last_loop;  /* lag compensation. */
    float position[3];

    /* If we dont have a trajectory data, disabled everything. */
    if(arm->trajectory.frame_count != 0) {
        cs_enable(&arm->z_axis_cs);
        cs_enable(&arm->shoulder_cs);
        cs_enable(&arm->elbow_cs);

        /* Are we before the first frame ? */
        if(current_date < arm->trajectory.frames[0].date) { 
           memcpy(position, arm->trajectory.frames[0].position, 3*sizeof(float)); 
        }
        /* Are we past the last frame ? */
        else if(current_date > arm->trajectory.frames[arm->trajectory.frame_count-1].date) {
            memcpy(position, arm->trajectory.frames[arm->trajectory.frame_count-1].position, 
                  3*sizeof(float));  
        }
        else {
            float t; /* interpolation factor, between 0 and 1 */
            int i = 1;
            /* We are between frame i-1 et i. i > 1 */
            while(arm->trajectory.frames[i].date < current_date) i++;
            t = current_date - arm->trajectory.frames[i-1].date;
            t = t / (float)(arm->trajectory.frames[i].date - arm->trajectory.frames[i-1].date);

            /* Linear interpolation between the 2 frames. */
            position[0] = t * arm->trajectory.frames[i].position[0];
            position[1] = t * arm->trajectory.frames[i].position[1];
            position[2] = t * arm->trajectory.frames[i].position[2];
            position[0] += (1 - t) * arm->trajectory.frames[i-1].position[0];
            position[1] += (1 - t) * arm->trajectory.frames[i-1].position[1];
            position[2] += (1 - t) * arm->trajectory.frames[i-1].position[2];
        }
        /** FIXME @todo Implementer la cinematique inverse. */
    }
    else {
        cs_disable(&arm->z_axis_cs);
        cs_disable(&arm->shoulder_cs);
        cs_disable(&arm->elbow_cs);
    }

    arm->last_loop = uptime_get();
}

void arm_shutdown(arm_t *arm) {
    if(arm->trajectory.frame_count != 0) {
        free(arm->trajectory.frames);
        arm->trajectory.frames = 0;
    }
}


