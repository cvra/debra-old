#include "CppUTest/TestHarness.h"
#include <cstring>

extern "C" {
#include "../arm.h"
#include "../arm_trajectories.h"
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

TEST(ArmTestGroup, ExecuteTrajectoryCopiesData)
{
    arm_trajectory_t traj;
    arm_init(&arm);
    arm_trajectory_init(&traj);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);

    arm_do_trajectory(&arm, &traj);
    CHECK_EQUAL(traj.frame_count, arm.trajectory.frame_count);
    CHECK(0 == memcmp(traj.frames, arm.trajectory.frames, sizeof(arm_keyframe_t) * traj.frame_count));
}

TEST(ArmTestGroup, ExecuteTrajectoryIsAtomic)
{
    arm_trajectory_t traj;
    arm_init(&arm);
    arm_trajectory_init(&traj);
    arm_do_trajectory(&arm, &traj);

    CHECK_EQUAL(1, arm.trajectory_semaphore.acquired_count);

    /* Checks that the function released semaphore. */
    CHECK_EQUAL(1, arm.trajectory_semaphore.count);
}

TEST(ArmTestGroup, ArmManageEmptyTrajectoryDisablesControl)
{
    arm_init(&arm);
    arm_manage(&arm);
    CHECK_EQUAL(0, arm.shoulder.manager.enabled);
    CHECK_EQUAL(0, arm.elbow.manager.enabled);
    CHECK_EQUAL(0, arm.z_axis.manager.enabled);
}


TEST(ArmTestGroup, ArmManageUpdatesLastLoop)
{
    arm_init(&arm);
    uptime_set(42);
    arm_manage(&arm);
    uptime_set(0);
    CHECK_EQUAL(42, arm.last_loop)
}

TEST(ArmTestGroup, ArmFinishedTrajectoryHasEnabledControl)
{
    arm_trajectory_t traj;
    arm_init(&arm);
    arm_trajectory_init(&traj);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
    arm_do_trajectory(&arm, &traj);

    uptime_set(20 * 1000000);
    arm_manage(&arm);
    CHECK_EQUAL(1, arm.shoulder.manager.enabled);
    CHECK_EQUAL(1, arm.elbow.manager.enabled);
    CHECK_EQUAL(1, arm.z_axis.manager.enabled);
    uptime_set(0);
}

IGNORE_TEST(ArmTestGroup, ArmManageChangesConsign)
{
    arm_trajectory_t traj;
    arm_init(&arm);
    arm_trajectory_init(&traj);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
    arm_do_trajectory(&arm, &traj);

    uptime_set(8 * 1000000);
    arm_manage(&arm);
    CHECK(0 != cs_get_consign(&arm.shoulder.manager));
    CHECK(0 != cs_get_consign(&arm.elbow.manager));
    CHECK(0 != cs_get_consign(&arm.z_axis.manager));
}

