#include <aversive.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>
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


/* Arm goto command. */
struct cmd_arm_goto_result {
    fixed_string_t arg0;
    fixed_string_t arg1;
    float argx;
    float argy;
    float argz;
    float duration;
};


static void cmd_arm_goto_parsed (void *r, __attribute__((unused)) void *data) {
    struct cmd_arm_goto_result *results = (struct cmd_arm_goto_result *)r;

    printf("%s arm going to (%.1f;%.1f;%.1f) in %.1f seconds\n", results->arg1,
            results->argx, results->argy, results->argz, results->duration);
}

char str_arm_goto_arg0[] = "arm_goto";
parse_token_string_t cmd_arm_goto_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_goto_result, 
                                                                      arg0, str_arm_goto_arg0);
char str_arm_goto_arg1[] = "left#right";
parse_token_string_t cmd_arm_goto_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_arm_goto_result, 
                                                                      arg1, str_arm_goto_arg1);

parse_token_num_t cmd_arm_goto_argx = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, argx, FLOAT);
parse_token_num_t cmd_arm_goto_argy = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, argy, FLOAT);
parse_token_num_t cmd_arm_goto_argz = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, argz, FLOAT);
parse_token_num_t cmd_arm_goto_arg_duration = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, duration, FLOAT);

char help_arm_goto[] = "Makes an arm go to a point.";
parse_inst_t cmd_arm_goto= {
	.f = cmd_arm_goto_parsed,                  /* function to call */
	.data = (void *)0,                           /* 2nd arg of func */
	.help_str = help_arm_goto,                 /* Help string */
	.tokens = {                             /* token list, NULL terminated */
		(void *)&cmd_arm_goto_arg0, 
        (void *)&cmd_arm_goto_arg1, 
        (void *)&cmd_arm_goto_argx, 
        (void *)&cmd_arm_goto_argy, 
        (void *)&cmd_arm_goto_argz, 
        (void *)&cmd_arm_goto_arg_duration, 
		NULL,
	},
};


struct cmd_arm_coordinate_system_result {
    fixed_string_t arg0;
    fixed_string_t system;

    float argx;
    float argy;
};

static void cmd_arm_coordinate_system_parsed (void *r, __attribute__((unused)) void *data) {
    struct cmd_arm_coordinate_system_result *result = (struct cmd_arm_coordinate_system_result *)r;
    arm_coordinate_t coordinate = COORDINATE_ARM;
    float resX, resY;

    if(!strcmp(result->system, "robot")) {
        coordinate = COORDINATE_ROBOT;
    }

    if(!strcmp(result->system, "table")) {
        coordinate = COORDINATE_TABLE;
    }

    arm_change_coordinate_system(&robot.left_arm, result->argx, result->argy,
            coordinate, &resX, &resY);

    printf("The results in arm coordinate is (%.1f; %.1f)\n", resX, resY);

}


char help_arm_coordinate_system[] = "Test coordinate system transformation.";

char str_arm_coordinate_system_arg0[] = "arm_coordinate_system";
parse_token_string_t cmd_arm_coordinate_system_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_coordinate_system_result, 
                                                                      arg0, str_arm_coordinate_system_arg0);
char str_arm_coordinate_system_system[] = "arm#robot#table";
parse_token_string_t cmd_arm_coordinate_system_system = TOKEN_STRING_INITIALIZER(struct cmd_arm_coordinate_system_result, 
                                                                      system, str_arm_coordinate_system_system);

parse_token_num_t cmd_arm_coordinate_system_argx = TOKEN_NUM_INITIALIZER(struct cmd_arm_coordinate_system_result, argx, FLOAT);
parse_token_num_t cmd_arm_coordinate_system_argy = TOKEN_NUM_INITIALIZER(struct cmd_arm_coordinate_system_result, argy, FLOAT);

parse_inst_t cmd_arm_coordinate_system = {
    .f = cmd_arm_coordinate_system_parsed,
    .data = NULL,
    .help_str = help_arm_coordinate_system,
    .tokens = {
        (void *)&cmd_arm_coordinate_system_arg0,
        (void *)&cmd_arm_coordinate_system_system,
        (void *)&cmd_arm_coordinate_system_argx,
        (void *)&cmd_arm_coordinate_system_argy,
        NULL,
    },
};
