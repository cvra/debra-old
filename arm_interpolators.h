#ifndef _ARM_INTERPOLATOR_H_

#include "keyframe.h"
/** Creates a simple linear trajectory with a constant speed.
 * @param [out] traj The trajectory structure to fill.
 * @param [in] start, end The starting and ending point of the trajectory.
 * @param [in] duration The duration of the trajectory in seconds.
 *
 * @note This function checks if the given trajectory structure is empty, and if not, empties it before
 * continuing.
 *
 * @warning The movement created by this function is expected to be launched right away by 
 * arm_execute_movement. If this is not the case, sudden movement can happen.
 */
void arm_interpolator_linear_motion(arm_trajectory_t *traj, const float start[3], const float end[3], arm_coordinate_t system,  const float duration);

/** Adds a point to a given trajectory.
 * @param [in, out] traj The trajectory structure to add the point to.
 * @param [in] x,y,z The point to add.
 * @param [in] system The coordinate system of the point.
 * @param [in] duration The time between this point and the previous one in second.
 *
 * @note This function tests if the given trajectory is empty, and if it is, it assumes the first point
 * date is now.
 */
void arm_interpolater_append_point(arm_trajectory_t *traj, const float x, const float y, const float z,
                                   arm_coordinate_t system, const float duration);

/** Zeroes an arm_trajectory_t structure to avoid problems.
 * @param traj The trajectory to zero.
 */
void arm_trajectory_init(arm_trajectory_t *traj);

#endif
