#include "CppUTest/TestHarness.h"

extern "C" {
#include "../arm_trajectories.h"
#include "uptime.h"
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
        uptime_set(0);
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
    CHECK_EQUAL(traj.frames[1].date, 10000000);
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

TEST(ArmTrajectoriesBuilderTest, EmptyTrajectoryIsFinished)
{
    CHECK_EQUAL(1, arm_trajectory_finished(&traj));
}

TEST(ArmTrajectoriesBuilderTest, TrajectoryWithPointIsNotFinished)
{
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
    CHECK_EQUAL(0, arm_trajectory_finished(&traj));
}

TEST(ArmTrajectoriesBuilderTest, PastTrajectoryIsFinished)
{
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
    uptime_set(20*1000000);
    CHECK_EQUAL(1, arm_trajectory_finished(&traj));
}

TEST(ArmTrajectoriesBuilderTest, KeyframeInterpolation)
{
    const int32_t interpolation_date = 5 * 1000000; // microseconds
    arm_keyframe_t result;
    arm_trajectory_append_point_with_length(&traj, 0, 0, 0, COORDINATE_ARM, 1., 100, 100);
    arm_trajectory_append_point_with_length(&traj, 10, 20, 0, COORDINATE_ARM, 10., 200, 200);

    result = arm_trajectory_interpolate_keyframes(traj.frames[0], traj.frames[1], interpolation_date);
    CHECK_EQUAL(interpolation_date, result.date);

    DOUBLES_EQUAL(5., result.position[0], 0.1);
    DOUBLES_EQUAL(10., result.position[1], 0.1);
    DOUBLES_EQUAL(10., result.position[1], 0.1);

    DOUBLES_EQUAL(150., result.length[0], 0.1);
    DOUBLES_EQUAL(150., result.length[1], 0.1);
}

