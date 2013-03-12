#include <aversive.h>
#include <parse.h>
#include <parse_string.h>
#include <stdio.h>

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
