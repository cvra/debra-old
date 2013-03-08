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
void arm_interpolator_linear_motion(arm_trajectory_t *traj, const float start[3], const float end[3], 
                                    const float duration);

#endif
