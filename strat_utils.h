#ifndef _STRAT_UTILS_H_
#define _STRAT_UTILS_H_

/** Duration of a match in seconds. */
#define MATCH_TIME 89

/* Return values for trajectories. */
#define END_TRAJ       1 /**< Trajectory successful. */
#define END_BLOCKING   2 /**< Blocking during trajectory. */
#define END_NEAR       4 /**< Arrived near point. */
#define END_OBSTACLE   8 /**< There is an obstacle in front of us */
#define END_ERROR     16 /**< Cannot do the command */
#define END_TIMER     32 /**< End of match timer. */

/** Checks if an return code indicates a succesful trajectory. */
#define TRAJ_SUCCESS(f) ((f) & (END_TRAJ|END_NEAR))

/** Flags for "standard" trajectories. */
#define TRAJ_FLAGS_STD (END_TRAJ|END_BLOCKING|END_TIMER|END_ERROR | END_OBSTACLE)

#define TRAJ_FLAGS_SHORT_DISTANCE (END_TRAJ|END_ERROR|END_TIMER)

/** Flags for "cutting corners" trajectories.
 * @warning Using this type of trajectories lowers the precision.
 */
#define TRAJ_FLAGS_NEAR (TRAJ_FLAGS_STD|END_NEAR)

/** This enum is used for specifying a team color. */
typedef enum {BLUE, RED} strat_color_t;

/** Computes the symmetrical position depending on color. */ 
#define COLOR_Y(x) (strat.color == RED ? (x) : 2000 - (x))

/** Computes the symmetrical angle depending on color. */
#define COLOR_A(x) (strat.color == RED ? (x) : -(x))

#define wait_traj_end(why) wait_traj_end_debug(why, __FILE__, __LINE__)

enum servo_e {
    LEFT,
    RIGHT
}; 

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
int wait_traj_end_debug(int why, char *file, int line);


void left_pump(int status);
void right_pump(int status);

/** Opens the given servo motor. */
void strat_open_servo(enum servo_e servo);

/** Closes the given servo motor. */
void strat_open_servo(enum servo_e servo);

/** Releases a servo pressure. */
void strat_release_servo(enum servo_e servo);

#endif
