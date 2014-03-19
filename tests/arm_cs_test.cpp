#include "CppUTest/TestHarness.h"

extern "C" {
#include "../arm_cs.h"
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
