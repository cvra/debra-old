#include "CppUTest/TestHarness.h"

extern "C" {
#include "../arm_cs.h"
}

static int pwm_called;
static int get_encoder_called;

static void pwm(void *p, int32_t v)
{
    pwm_called++;
}

static int32_t get_encoder(void *p)
{
    get_encoder_called++;
    return 0;
}

TEST_GROUP(ArmControlTestGroup)
{
    arm_control_loop_t loop;

    void setup(void)
    {
        arm_cs_init_loop(&loop);
    }

};

TEST(ArmControlTestGroup, LoopIsEnabled)
{
    CHECK(loop.manager.enabled);
}


TEST(ArmControlTestGroup, LoopArchitectureIsCorrect)
{
    POINTERS_EQUAL(pid_do_filter, loop.manager.correct_filter);
    POINTERS_EQUAL(&loop.pid, loop.manager.correct_filter_params);
}

TEST(ArmControlTestGroup, ProcessOutCalled)
{
    arm_cs_connect_motor(&loop, pwm, NULL);
    arm_cs_manage(&loop);
    CHECK_EQUAL(1, pwm_called);
}

TEST(ArmControlTestGroup, ProcessInCalled)
{
    arm_cs_connect_encoder(&loop, get_encoder, NULL);
    arm_cs_manage(&loop);
    CHECK_EQUAL(1, get_encoder_called);
}
