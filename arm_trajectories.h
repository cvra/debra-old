#ifndef _ARM_TRAJECTORY_H_

#include "keyframe.h"

/** Adds a point to a given trajectory.
 * @param [in, out] traj The trajectory structure to add the point to.
 * @param [in] x,y,z The point to add.
 * @param [in] system The coordinate system of the point.
 * @param [in] duration The time between this point and the previous one in second.
 *
 * @note This function tests if the given trajectory is empty, and if it is, it assumes the first point
 * date is now.
 */
void arm_trajectory_append_point(arm_trajectory_t *traj, const float x, const float y, const float z,
                                   arm_coordinate_t system, const float duration);

/** Zeroes an arm_trajectory_t structure to avoid problems.
 * @param traj The trajectory to zero.
 */
void arm_trajectory_init(arm_trajectory_t *traj);


/** same as arm_trajectory_append_point but with a custom length. */
void arm_trajectory_append_point_with_length(arm_trajectory_t *traj, const float x, const float y, const float z,
                                   arm_coordinate_t system, const float duration, const float l1, const float l2);


void arm_trajectory_delete(arm_trajectory_t *traj);

void arm_trajectory_copy(arm_trajectory_t *dest, arm_trajectory_t *src);

int arm_trajectory_finished(arm_trajectory_t *traj);

#endif
