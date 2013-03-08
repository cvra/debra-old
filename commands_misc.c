#include <aversive.h>
#include <parse.h>
#include <parse_string.h>
#include <stdio.h>
#include <string.h>
#include "hardware.h"

/**********************************************************/
/* Resets the board */

/** this structure is filled when cmd_reset is parsed successfully */
struct cmd_reset_result {
	fixed_string_t arg0;
};

/** function called when cmd_reset is parsed successfully */
static void cmd_reset_parsed(__attribute__((unused)) void * parsed_result,__attribute__((unused)) void * data) {
	reset();
}

char str_reset_arg0[] = "reset";
parse_token_string_t cmd_reset_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_reset_result, arg0, str_reset_arg0);

char help_reset[] = "Reset the board";
parse_inst_t cmd_reset = {
	.f = cmd_reset_parsed,                  /* function to call */
	.data = NULL,                           /* 2nd arg of func */
	.help_str = help_reset,                 /* Help string */
	.tokens = {                             /* token list, NULL terminated */
		(void *)&cmd_reset_arg0, 
		NULL,
	},
};



/* Sets the pump on and off. */
struct cmd_pump_result {
    fixed_string_t arg0; ///< "pump"
    fixed_string_t pump; ///< "left" or "right" or "both"
    fixed_string_t direction; ///< "off" or "dir1" or "dir2"
};

static void cmd_pump_parsed(void *parsed_result, __attribute__((unused)) void * data) {
    struct cmd_pump_result *result = (struct cmd_pump_result *)parsed_result;

    if(strcmp(result->pump, "left") == 0 || strcmp(result->pump, "both")==0) {
        if(strcmp(result->direction, "dir1") == 0)
            cvra_pump_left_mode(VENT_BAS);
        if(strcmp(result->direction, "dir2") == 0)
            cvra_pump_left_mode(VENT_LAT);
        if(strcmp(result->direction, "off") == 0)
            cvra_pump_left_mode(OFF);
    }

    if(strcmp(result->pump, "right") == 0 || strcmp(result->pump, "both")==0) {
        if(strcmp(result->direction, "dir1") == 0)
            cvra_pump_right_mode(VENT_BAS);
        if(strcmp(result->direction, "dir2") == 0)
            cvra_pump_right_mode(VENT_LAT);
        if(strcmp(result->direction, "off") == 0)
            cvra_pump_right_mode(OFF);
    }

}

char str_pump_arg0[] = "pump";
char str_pump_pump[] = "left#right#both";
char str_pump_direction[] = "off#dir1#dir2";
parse_token_string_t cmd_pump_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pump_result, arg0, str_pump_arg0);
parse_token_string_t cmd_pump_pump = TOKEN_STRING_INITIALIZER(struct cmd_pump_result, pump, str_pump_pump);
parse_token_string_t cmd_pump_direction = TOKEN_STRING_INITIALIZER(struct cmd_pump_result, direction, str_pump_direction);
char help_pump[] = "Sets the pumps state.";

parse_inst_t cmd_pump = {
    .f = cmd_pump_parsed,
    .data = NULL,
    .help_str = help_pump,
    .tokens = {
        (void *)&cmd_pump_arg0,
        (void *)&cmd_pump_pump,
        (void *)&cmd_pump_direction,
        NULL /* The last element of the list must always be NULL. */
    },
};
