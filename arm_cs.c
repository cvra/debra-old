#include "arm_cs.h"

void arm_cs_init_loop(arm_control_loop_t *loop)
{
    cs_init(&loop->manager);
    pid_init(&loop->pid);
    cs_set_correct_filter(&loop->manager, pid_do_filter, &loop->pid);
}

void arm_manage_cs(arm_control_loop_t *loop)
{
    cs_manage(&arm->manager);
}
