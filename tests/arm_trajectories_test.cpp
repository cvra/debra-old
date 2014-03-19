#include "CppUTest/TestHarness.h"

extern "C" {
#include "../arm_trajectories.h"
}

TEST_GROUP(ArmTrajectoriesBuilderTest)
{
    arm_trajectory_t traj;

    void setup()
    {
        arm_trajectory_init(&traj);
    }

    void teardown()
    {
        free(traj.frames);
    }
};


TEST(ArmTrajectoriesBuilderTest, CanAddOnePoint)
{
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
    CHECK_EQUAL(traj.frame_count, 1);
}

TEST(ArmTrajectoriesBuilderTest, CanAddMultiplePoints)
{
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
    CHECK_EQUAL(traj.frame_count, 2);
}


TEST(ArmTrajectoriesBuilderTest, DateIsCorrectlyComputed)
{
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);

    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 15.);
    CHECK_EQUAL(traj.frames[1].date, 10*1000000);
    CHECK_EQUAL(traj.frames[2].date, 25*1000000);
}

TEST(ArmTrajectoriesBuilderTest, DeleteTrajectory)
{
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 15.);
    arm_trajectory_delete(&traj);
    CHECK_EQUAL(0, traj.frame_count);
}

TEST(ArmTrajectoriesBuilderTest, CopyTrajectory)
{
    arm_trajectory_t copy;
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);

    arm_trajectory_copy(&copy, &traj);
    CHECK_EQUAL(traj.frame_count, copy.frame_count);
    CHECK_EQUAL(traj.frames[0].position[0], copy.frames[0].position[0]);

    /* Check that it is a full copy. */
    CHECK(traj.frames[0].position != copy.frames[0].position);
}
