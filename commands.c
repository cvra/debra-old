/** Registers all the command in a single file. */
#include <parse.h>
#include <stdlib.h>

/* commands_misc.c */
/** Resets the board. */
extern parse_inst_t cmd_reset;


/* commands_arm.c */
/** Shutdowns an arm. */
extern parse_inst_t cmd_arm_shutdown;

/** Shutdowns both arms. */
extern parse_inst_t cmd_arm_shutdown_both;

/** Makes the arm go to a point. */
extern parse_inst_t cmd_arm_goto;

/** Tests the coordinate system transform functions. */
extern parse_inst_t cmd_arm_coordinate_system;

/* commands_cs.c */
/** Prints robot position. */
extern parse_inst_t cmd_position_get;

/** Sets robot position. */
extern parse_inst_t cmd_position_set;

/* All the following commands are declared in commands_misc.c */
extern parse_inst_t cmd_start;


parse_ctx_t commands[] = {
    (parse_ctx_t)&cmd_reset,
    (parse_ctx_t)&cmd_arm_shutdown,
    (parse_ctx_t)&cmd_arm_shutdown_both,
    (parse_ctx_t)&cmd_arm_goto,
    (parse_ctx_t)&cmd_arm_coordinate_system,
    (parse_ctx_t)&cmd_position_get,
    (parse_ctx_t)&cmd_position_set,
	(parse_ctx_t)&cmd_start,
    NULL /* The last element of the list must always be NULL. */
};
 



