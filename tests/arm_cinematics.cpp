#include "CppUTest/TestHarness.h"
#include <cmath>

extern "C" {
#include "../arm_cinematics.h"
#include "../arm.h"
#include "../arm_trajectories.h"
#include "uptime.h"
}

#define RAD(x) ((x/180.)*M_PI)

TEST_GROUP(CinematicsTestGroup)
{
    float alpha, beta;
    point_t p1, p2;
    int status;

    arm_trajectory_t traj;
    arm_t arm;

    void setup()
    {
        arm_init(&arm);
        arm_set_physical_parameters(&arm);
        arm.offset_rotation = M_PI / 2;
        arm_trajectory_init(&traj);
    }

    void teardown()
    {
        uptime_set(0);
        arm_trajectory_delete(&traj);
        arm_trajectory_delete(&arm.trajectory);
    }
};

TEST(CinematicsTestGroup, FindsASolution)
{
    point_t target = {100., 100.};
    status = compute_possible_elbow_positions(target, 100., 100., &p1, &p2);
    CHECK_EQUAL(2, status);
}

IGNORE_TEST(CinematicsTestGroup, FindsSingleSolution)
{
    point_t target = {100., 0.};
    status = compute_possible_elbow_positions(target, 50., 50., &p1, &p2);
    CHECK_EQUAL(1, status);
}

TEST(CinematicsTestGroup, FailsWhenTooFar)
{
    point_t target = {100., 100.};
    status = compute_possible_elbow_positions(target, 10., 10., &p1, &p2);
    CHECK_EQUAL(0, status);
}

TEST(CinematicsTestGroup, ForwardCinematicsTrivialCase)
{
    point_t result;
    float length[] = {100., 100.};
    result = arm_forward_cinematics(0., 0., length);
    DOUBLES_EQUAL(result.x, 200, 1e-2);
    DOUBLES_EQUAL(result.y, 0, 1e-2);
}

TEST(CinematicsTestGroup, ForwardCinematicsTrivialCaseBis)
{
    point_t result;
    float length[] = {100., 100.};
    result = arm_forward_cinematics(M_PI/2, 0., length);

    DOUBLES_EQUAL(result.x, 0, 1e-2);
    DOUBLES_EQUAL(result.y, 200, 1e-2);
}

TEST(CinematicsTestGroup, ForwardCinematicsNegativeAnglesToo)
{
    point_t result;
    float length[] = {100., 100.};
    result = arm_forward_cinematics(-M_PI/2, 0., length);

    DOUBLES_EQUAL(result.x, 0, 1e-2);
    DOUBLES_EQUAL(result.y, -200, 1e-2);
}

TEST(CinematicsTestGroup, DoesNotOscillateAroundZero)
{
    arm_keyframe_t frame;

    point_t target;
    int position_count;

    arm_trajectory_append_point(&traj, 100,  1, 10, COORDINATE_ARM, 1.);
    arm_trajectory_append_point(&traj, 100, -1, 10, COORDINATE_ARM, 10.);
    arm_do_trajectory(&arm, &traj);

    while (uptime_get() < traj.frames[traj.frame_count-1].date) {
        frame = arm_position_for_date(&arm, uptime_get());

        target.x = frame.position[0];
        target.y = frame.position[1];

        position_count = compute_possible_elbow_positions(target,
                                 frame.length[0], frame.length[1],
                                 &p1, &p2);

        CHECK_EQUAL(2, position_count);
        uptime_set(uptime_get() + 2*1000);
    }
}
