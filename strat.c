#include <stdio.h>
#include <fcntl.h>

#include <aversive/error.h>
#include <cvra_beacon.h>

#include "strat.h"
#include "strat_job.h"
#include "strat_utils.h"


void strat_begin(void) {

    /* Starts the game timer. */
    strat_timer_reset();

    /* XXX 2014 code goes there. */

    /* After the game, shutdown the pumps. */
    left_pump(0);
    right_pump(0);
}


