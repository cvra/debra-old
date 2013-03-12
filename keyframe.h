#ifndef _KEYFRAME_H_
#define _KEYFRAME_H_

#include <aversive/types.h>

/** Definition d'un point de la trajectoire d'un bras. */
typedef struct {
    int32_t date;       /**< Temps de validite de la frame, depuis le boot, en us. */
    float position[3];  /**< Position du bras. */
} arm_keyframe_t;


/** Definition d'une trajectoire pour les bras. */
typedef struct {
    arm_keyframe_t *frames; /**< Trajectory keyframes. */
    int frame_count;      /**< Number of frames. */
} arm_trajectory_t;
#endif
