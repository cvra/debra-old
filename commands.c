/** Registers all the command in a single file. */
#include <parse.h>
#include <stdlib.h>

/* All the following commands are declared in commands_misc.c */
/** Resets the board. */
extern parse_inst_t cmd_reset;

/** Shutdowns an arm. */
extern parse_inst_t cmd_arm_shutdown;

/** Shutdowns both arms. */
extern parse_inst_t cmd_arm_shutdown_both;

/* All the following commands are declared in commands_misc.c */
extern parse_inst_t cmd_start;

/** Sets PWM. */
extern parse_inst_t cmd_pwm;


parse_ctx_t commands[] = {
    (parse_ctx_t)&cmd_reset,
    (parse_ctx_t)&cmd_arm_shutdown,
    (parse_ctx_t)&cmd_arm_shutdown_both,
    (parse_ctx_t)&cmd_start,
    (parse_ctx_t)&cmd_pwm,
    NULL
};
 



