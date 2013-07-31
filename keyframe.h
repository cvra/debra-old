#ifndef _KEYFRAME_H_
#define _KEYFRAME_H_
#include <aversive.h>

/** This typedef stores the type of the coordinate system used for a trajectory. */
typedef enum {
    COORDINATE_ARM=0,  /**< Coordinate relative to the arm shoulder. */
    COORDINATE_ROBOT,/**< Coordinate relative to the robot center. */
    COORDINATE_TABLE /**< Coordinate relative to the table (absolute). */
} arm_coordinate_t;

/** This structure holds the data for a single keyframe of an arm trajectory. */
typedef struct {
    int32_t date;       /**< Keyframe validity date, since the boot of the robot in us. */
    float position[3];  /**< Position of the arm at the data. */
    arm_coordinate_t coordinate_type; /**< The coordinate system of this trajectory. */
    float length[2]; /**< The length of the arm to use. */
    float angle_offset; /* ugly hack because 2013 grippers are ugly. */
} arm_keyframe_t;


/** This structure holds the data of a single arm trajectory. */
typedef struct {
    arm_keyframe_t *frames; /**< Trajectory keyframes. */
    int frame_count;        /**< Number of frames. */
} arm_trajectory_t;
#endif
