#include "arm_cs.h"

void arm_cs_init_loop(arm_control_loop_t *loop)
{
    cs_init(&loop->manager);
    pid_init(&loop->pid);
    cs_set_correct_filter(&loop->manager, pid_do_filter, &loop->pid);
}

void arm_cs_manage(arm_control_loop_t *loop)
{
    cs_manage(&loop->manager);
}

void arm_cs_connect_motor(arm_control_loop_t *loop,  void (*pwm)(void *, int32_t), void *pwm_param)
{
    cs_set_process_in(&loop->manager, pwm, pwm_param);
}

void arm_cs_connect_encoder(arm_control_loop_t *loop,  int32_t (*encoder)(void *), void *encoder_param)
{
    cs_set_process_out(&loop->manager, encoder, encoder_param);
}

