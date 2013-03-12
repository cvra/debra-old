#include <aversive.h>
#include <parse.h>
#include <parse_string.h>
#include <stdio.h>
#include <string.h>
#include "arm.h"
#include "cvra_cs.h"

/* Shutdown an arm */

struct cmd_arm_shutdown_result {
    fixed_string_t arg0;
    fixed_string_t arg1;
};

static void cmd_arm_shutdown_parsed(void *r, void *data) {
    struct cmd_arm_shutdown_result *results =(struct cmd_arm_shutdown_result *)r;
    if(data) {
        arm_shutdown(&robot.right_arm);
        arm_shutdown(&robot.left_arm);
    }
    else {
        if(!strcmp(results->arg1, "left")) {
           arm_shutdown(&robot.left_arm); 
        }
        if(!strcmp(results->arg1, "right")) {
            arm_shutdown(&robot.right_arm); 
        }
    }
}

char str_arm_shutdown_arg0[] = "arm_shutdown";
parse_token_string_t cmd_arm_shutdown_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_shutdown_result, 
                                                                      arg0, str_arm_shutdown_arg0);
char str_arm_shutdown_arg1[] = "left#right";
parse_token_string_t cmd_arm_shutdown_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_arm_shutdown_result, 
                                                                      arg1, str_arm_shutdown_arg1);

char help_arm_shutdown[] = "Shutdowns an arm";
parse_inst_t cmd_arm_shutdown= {
	.f = cmd_arm_shutdown_parsed,                  /* function to call */
	.data = (void *)0,                           /* 2nd arg of func */
	.help_str = help_arm_shutdown,                 /* Help string */
	.tokens = {                             /* token list, NULL terminated */
		(void *)&cmd_arm_shutdown_arg0, 
        (void *)&cmd_arm_shutdown_arg1, 
		NULL,
	},
};


char help_arm_shutdown_both[] = "Shutdown both arms";
parse_inst_t cmd_arm_shutdown_both = {
	.f = cmd_arm_shutdown_parsed,                  /* function to call */
	.data = (void *)1,                           /* 2nd arg of func */
	.help_str = help_arm_shutdown_both,                 /* Help string */
	.tokens = {                             /* token list, NULL terminated */
		(void *)&cmd_arm_shutdown_arg0, 
		NULL,
	},
};

