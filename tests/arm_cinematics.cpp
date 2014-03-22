#include "CppUTest/TestHarness.h"
#include <cmath>

extern "C" {
#include "../arm_cinematics.h"
}

#define RAD(x) ((x/180.)*M_PI)

TEST_GROUP(CinematicsTestGroup)
{
    float alpha, beta;
    point_t p1, p2;
    int status;
};

TEST(CinematicsTestGroup, FindsASolution)
{
    point_t target = {100., 100.};
    status = compute_possible_elbow_positions(target, 100., 100., &p1, &p2);
    CHECK_EQUAL(2, status);
}

TEST(CinematicsTestGroup, FindsSingleSolution)
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
