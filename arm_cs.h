#ifndef _ARM_CS_H_
#define _ARM_CS_H_

#include <pid.h>
#include <control_system_manager.h>

typedef struct {
    struct pid_filter pid;
    struct cs manager;
} arm_control_loop_t;

void arm_cs_init_loop(arm_control_loop_t *loop);

void arm_cs_connect_motor(arm_control_loop_t *loop,  void (*pwm)(void *, int32_t), void *pwm_param);
void arm_cs_connect_encoder(arm_control_loop_t *loop,  int32_t (*encoder)(void *), void *encoder_param);
void arm_cs_manage(arm_control_loop_t *loop);

#endif
