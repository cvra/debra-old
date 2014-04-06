#include <platform.h>
#include <stdlib.h>
#include <cvra_dc.h>
#include "arm.h"
#include "cvra_cs.h"

OS_STK    control_task_stk[2048];
#define   CONTROL_TASK_PRIORITY 24

OS_STK    cinematics_task_stk[2048];
#define   CINEMATICS_TASK_PRIORITY 25

void arm_cinematics_manage_task(void *dummy)
{
    dummy;
    while (1) {
        arm_manage(&robot.right_arm);
        OSTimeDlyHMSM(0, 0, 0, 20);
    }
}

void arm_control_manage_task(__attribute__((unused)) void *dummy)
{
    while (1) {
        arm_cs_manage(&robot.right_arm.z_axis);
        arm_cs_manage(&robot.left_arm.z_axis);
        arm_cs_manage(&robot.right_arm.shoulder);
        arm_cs_manage(&robot.left_arm.shoulder);
        arm_cs_manage(&robot.right_arm.elbow);
        arm_cs_manage(&robot.left_arm.elbow);
        arm_cs_manage(&robot.right_arm.hand);
        arm_cs_manage(&robot.left_arm.hand);
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}

void arm_highlevel_init(void)
{
    int i;

    for (i=0;i<6;i++) {
        cvra_dc_set_pwm(ARMSMOTORCONTROLLER_BASE, i, 0);
        cvra_dc_set_encoder(ARMSMOTORCONTROLLER_BASE, i, 0);

    }
    arm_init(&robot.right_arm);
    arm_init(&robot.left_arm);

    arm_cs_connect_motor(&robot.right_arm.z_axis, cvra_dc_set_pwm5, HEXMOTORCONTROLLER_BASE);
    arm_cs_connect_encoder(&robot.right_arm.z_axis, cvra_dc_get_encoder5, HEXMOTORCONTROLLER_BASE);

    arm_cs_connect_motor(&robot.left_arm.z_axis, cvra_dc_set_pwm0, HEXMOTORCONTROLLER_BASE);
    arm_cs_connect_encoder(&robot.left_arm.z_axis, cvra_dc_get_encoder0, HEXMOTORCONTROLLER_BASE);

    arm_cs_connect_motor(&robot.right_arm.shoulder, cvra_dc_set_pwm4, ARMSMOTORCONTROLLER_BASE);
    arm_cs_connect_encoder(&robot.right_arm.shoulder, cvra_dc_get_encoder4, ARMSMOTORCONTROLLER_BASE);

    arm_cs_connect_motor(&robot.right_arm.elbow, cvra_dc_set_pwm5, ARMSMOTORCONTROLLER_BASE);
    arm_cs_connect_encoder(&robot.right_arm.elbow, cvra_dc_get_encoder5, ARMSMOTORCONTROLLER_BASE);

    arm_cs_connect_motor(&robot.right_arm.hand, cvra_dc_set_pwm3, ARMSMOTORCONTROLLER_BASE);
    arm_cs_connect_encoder(&robot.right_arm.hand, cvra_dc_get_encoder3, ARMSMOTORCONTROLLER_BASE);

    arm_cs_connect_motor(&robot.left_arm.elbow, cvra_dc_set_pwm0, ARMSMOTORCONTROLLER_BASE);
    arm_cs_connect_encoder(&robot.left_arm.elbow, cvra_dc_get_encoder0, ARMSMOTORCONTROLLER_BASE);

    arm_cs_connect_motor(&robot.left_arm.hand, cvra_dc_set_pwm2, ARMSMOTORCONTROLLER_BASE);
    arm_cs_connect_encoder(&robot.left_arm.hand, cvra_dc_get_encoder2, ARMSMOTORCONTROLLER_BASE);

    arm_cs_connect_motor(&robot.left_arm.shoulder, cvra_dc_set_pwm1, ARMSMOTORCONTROLLER_BASE);
    arm_cs_connect_encoder(&robot.left_arm.shoulder, cvra_dc_get_encoder1, ARMSMOTORCONTROLLER_BASE);

    arm_set_physical_parameters(&robot.right_arm);
    arm_set_physical_parameters(&robot.left_arm);

    OSTaskCreateExt(arm_control_manage_task,
                    NULL,
                    &control_task_stk[2047],
                    CONTROL_TASK_PRIORITY,
                    CONTROL_TASK_PRIORITY,
                    &control_task_stk[0],
                    2048, /* stack size */
                    NULL, NULL);


#if 0
    OSTaskCreateExt(arm_cinematics_manage_task, 
                    NULL,
                    &cinematics_task_stk[2047],
                    CINEMATICS_TASK_PRIORITY,
                    CINEMATICS_TASK_PRIORITY,
                    &cinematics_task_stk[0],
                    2048, /* stack size */
                    NULL, NULL);
#endif
}
