#include <parse_string.h>
#include <parse_num.h>
#include <stdio.h>
#include <string.h>

#include "cvra_cs.h"

struct cmd_pwm_result {
    fixed_string_t arg0; ///< "pwm"
    fixed_string_t pwm_name;
    int16_t value;
};

static void cmd_pwm_parsed(void *r, __attribute__((unused)) void *data) {
    struct cmd_pwm_result *results = (struct cmd_pwm_result *)r;

    /* TODO Check with Rouven how this is supposed to work. */
    printf("Setting PWM %s to %d", results->pwm_name, results->value);
}

char str_pwm_arg0[] = "pwm";
parse_token_string_t cmd_pwm_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pwm_result, arg0, str_pwm_arg0);
char str_pwm_pwm_name[] = "brushless1#brushless2"; /*XXX Add other PWMs. */
parse_token_string_t cmd_pwm_pwm_name = TOKEN_STRING_INITIALIZER(struct cmd_pwm_result, pwm_name, str_pwm_pwm_name);

parse_token_num_t cmd_pwm_value = TOKEN_NUM_INITIALIZER(struct cmd_pwm_result, value, INT16); 
char str_pwm_help[] = "Set PWM value.";

parse_inst_t cmd_pwm = {
    .f = cmd_pwm_parsed, 
    .data = (void *)NULL,
    .help_str = str_pwm_help,
    .tokens = {
        (void *)&cmd_pwm_arg0,
        (void *)&cmd_pwm_pwm_name,
        (void *)&cmd_pwm_value,
        NULL,
    },
};

