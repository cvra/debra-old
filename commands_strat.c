#include <aversive.h>
#include <parse.h>
#include <parse_string.h>
#include <stdio.h>
#include <string.h>

#include "strat.h"


/* Launches the match. */

struct cmd_start_result {
    fixed_string_t arg0;
};


static void cmd_start_parsed(__attribute__((unused)) void *r, 
                             __attribute__((unused)) void *data) {
    printf("Press a key to start the robot.\n");
    getchar();
    strat_begin();
    printf("Match done. Hope you enjoyed it !\n");
}


char str_start_arg0[] = "start";

parse_token_string_t cmd_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0,
                                                               str_start_arg0); 

char help_start[] = "Starts the match.";
parse_inst_t cmd_start = {
    .f = cmd_start_parsed,
    .data = NULL,
    .help_str = help_start,
    .tokens = {
        (void *)&cmd_start_arg0,
        NULL,
    },
};
