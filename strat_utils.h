#ifndef _STRAT_UTILS_H_
#define _STRAT_UTILS_H_

/** Auto positions the robot before the match. 
 *
 * This function positions the robot using the border as references. The
 * color is assumed to be already configured.
 * 
 * @param [in] x, y The starting coordinates, in mm.
 * @param [in] a The starting angle relative to the X-axis, in degrees. 
 * @param epaisseurRobot The distamce between the back of the robot and the wheel axis. 
 */
void strat_autopos(int16_t x, int16_t y, int16_t a, int16_t epaisseurRobot);

/** Tests for end of trajectory.
 *
 * @param [in] why The allowed reasons for this function to return true.
 * @returns An error code indicating the reason of the end of the trajectory.
 */
int test_traj_end(int why);

/** Waits for the end of a trajectory.
 *
 * @param [in] why The allowed reasons to end the trajectory.
 * @returns An error code indicating the reason of the end of the trajectory.
 */
int wait_traj_end(int why);

#endif
