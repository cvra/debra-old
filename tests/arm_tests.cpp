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

TEST(ArmTestGroup, PhysicalParametersMakeSense)
{
    arm_init(&arm);
    arm_set_physical_parameters(&arm);

    /* Length must be greater than zero. */
    CHECK(arm.length[0] > 0);
    CHECK(arm.length[1] > 0);

    /* We force the PID regulators to be at least of P type. */
    CHECK(pid_get_gain_P(&arm.z_axis.pid) != 0);
    CHECK(pid_get_gain_P(&arm.shoulder.pid) != 0);
    CHECK(pid_get_gain_P(&arm.elbow.pid) != 0);

    /* We check that we have some out shifting performed. */
    CHECK(pid_get_out_shift(&arm.z_axis.pid) != 0);
    CHECK(pid_get_out_shift(&arm.shoulder.pid) != 0);
    CHECK(pid_get_out_shift(&arm.elbow.pid) != 0);

    CHECK(arm.z_axis_imp_per_mm != 0);
    CHECK(arm.shoulder_imp_per_rad != 0);
    CHECK(arm.elbow_imp_per_rad != 0);
}
