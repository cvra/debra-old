/** @file arm.h
 * @author Antoine Albertelli, CVRA
 * @date 2013
 * @brief Toutes les fonctions liees a la nouvelle version des bras pour la coupe 2013.
 * @sa doc/APi Bras.odt
 */
#ifndef _ARM_H_
#define _ARM_H_

#include <platform.h>

#include "arm_cs.h"
#include "arm_cinematics.h"
#include "keyframe.h"
#include "2wheels/position_manager.h"
#include <vect2.h>


typedef struct {
    vect2_cart offset_xy; /**< Offset vector between center of robot and shoulder. */
    float offset_rotation; /**< Rotation between the robot base and shoulder in rad. */

    /* Control systems */
    arm_control_loop_t z_axis;
    arm_control_loop_t shoulder;
    arm_control_loop_t hand;
    arm_control_loop_t elbow;

    /* Physical parameters. */
    int32_t z_axis_imp_per_mm;      /**< Z axis encoder impulsion per mm. */
    int32_t shoulder_imp_per_rad;   /**< Shoulder impulsions per rad. */
    int32_t elbow_imp_per_rad;      /**< Elbow impulsions per rad. */
    float length[2];                  /**< Length of the 2 arms elements. */

    /* Path informations */
    arm_trajectory_t trajectory;    /**< Current trajectory of the arm. */
    semaphore_t trajectory_semaphore;
    int32_t last_loop;              /**< Timestamp of the last loop execution, in us since boot. */
    struct robot_position *robot_pos;

    shoulder_mode_t shoulder_mode;
} arm_t;

void arm_init(arm_t *arm);

void arm_do_trajectory(arm_t *arm, arm_trajectory_t *traj);

void arm_set_physical_parameters(arm_t *arm);

void arm_manage(arm_t *arm);


void arm_get_position(arm_t *arm, float *x, float *y, float *z);

arm_keyframe_t arm_position_for_date(arm_t *arm, int32_t date);

void arm_set_related_robot_pos(arm_t *arm, struct robot_position *pos);

void arm_shutdown(arm_t *arm);

#endif
