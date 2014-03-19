#include "CppUTest/TestHarness.h"

extern "C" {
#include "../arm.h"
#include "uptime.h"
}

TEST_GROUP(ArmTestGroup)
{
    arm_t arm;
};

TEST(ArmTestGroup, AllControlSystemInitialized)
{
    arm_init(&arm);
    CHECK_EQUAL(1, arm.shoulder.manager.enabled);
    CHECK_EQUAL(1, arm.elbow.manager.enabled);
    CHECK_EQUAL(1, arm.z_axis.manager.enabled);
}

TEST(ArmTestGroup, LagCompensationIsInitialized)
{

    uptime_set(42);
    arm_init(&arm);
    CHECK_EQUAL(42, arm.last_loop);
    uptime_set(0);
}

TEST(ArmTestGroup, ShoulderModeIsSetToBack)
{
    arm_init(&arm);
    CHECK_EQUAL(SHOULDER_BACK, arm.shoulder_mode);
}
