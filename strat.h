/** @file strat.h
 * @brief Strategy file
 *
 * This file implements all functions needed to give the "intelligence" of the
 * robot, also known as the strategy.
 *
 * Our strategy's graph can be seen in the following picture :
 * \dotfile doc/strat.dot "Strategy for 2013"
 *
 * The difference between playing in red and playing in blue is a change in the
 * coordinate system : The starting corner (away from the gift) is always at (0, 0)
 * and the X axis points toward the long side of the table. Therefore, since we want
 * our coordinate system to remain direct, the Y axis changes direction. When we play
 * as the red team, the Y axis points in the table and when we play on the blue side,
 * the Y axis points \a outside the table.
 *
 * To avoid error, when you use coordinate and/or angles, you should \a always use
 * the COLOR_Y and COLOR_A macros to specify a coordinate. Those macros will
 * do the correct coordinate change based on the color of the robot.
 */
#ifndef _STRAT_H_
#define _STRAT_H_

#include <aversive.h>
#include <vect_base.h>
#include "strat_utils.h"

/** @brief Starts a match
 *
 * This function starts the match. It will \a not check for the starting cord
 * so the caller should do it.
 */
void strat_begin(void);


int strat_goto_avoid(int x, int y, int flags);
#endif
