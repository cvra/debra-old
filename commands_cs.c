#include <parse_string.h>
#include <parse_num.h>
#include <stdio.h>
#include <string.h>

#include "cvra_cs.h"


/* Gets robot position. */

struct cmd_position_result {
    fixed_string_t arg0; ///< "position"
    float x; ///< X coordinate if set command 
    float y; ///< y coordinate if set command
    float a; ///< angle if set command
};

static void cmd_position_parsed(void *r, void *data) {
    struct cmd_position_result *result = (struct cmd_position_result *)r;

    if(data == NULL) {
        printf("x=%d, y=%d, angle=%d\n", position_get_x_s16(&robot.pos), position_get_x_s16(&robot.pos),  
                position_get_a_deg_s16(&robot.pos));
    }
    else {
        position_set(&robot.pos, result->x, result->y, result->a);
    }
}

char str_position_arg0[] = "position";
parse_token_string_t cmd_position_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_position_result,
                                                                  arg0, str_position_arg0);

parse_token_num_t cmd_position_argx = TOKEN_NUM_INITIALIZER(struct cmd_position_result, x, FLOAT); 
parse_token_num_t cmd_position_argy = TOKEN_NUM_INITIALIZER(struct cmd_position_result, y, FLOAT); 
parse_token_num_t cmd_position_arga = TOKEN_NUM_INITIALIZER(struct cmd_position_result, a, FLOAT); 

char help_position_get[] = "Prints robot position";

parse_inst_t cmd_position_get = {
    .f = cmd_position_parsed,
    .data = (void *)NULL,
    .help_str = help_position_get, 
    .tokens = {
        (void *)&cmd_position_arg0,
        NULL,
    },
};

char help_position_set[] = "Sets robot position";
parse_inst_t cmd_position_set = {
    .f = cmd_position_parsed,
    .data = (void *)1,
    .help_str = help_position_set, 
    .tokens = {
        (void *)&cmd_position_arg0,
        (void *)&cmd_position_argx,
        (void *)&cmd_position_argy,
        (void *)&cmd_position_arga,
        NULL,
    },
};
