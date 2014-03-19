#include "CppUTest/TestHarness.h"

extern "C" {
#include "../arm.h"
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
