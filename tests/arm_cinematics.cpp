#include "CppUTest/TestHarness.h"
#include "math.h"

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
    point_t target = {.x=100., .y=100.};
    status = compute_possible_elbow_positions(target, 100., 100., &p1, &p2);
    CHECK_EQUAL(2, status);
}

TEST(CinematicsTestGroup, FindsSingleSolution)
{
    point_t target = {.x=100., .y=0.};
    status = compute_possible_elbow_positions(target, 50., 50., &p1, &p2);
    CHECK_EQUAL(1, status);
}

TEST(CinematicsTestGroup, FailsWhenTooFar)
{
    point_t target = {.x=100., .y=100.};
    status = compute_possible_elbow_positions(target, 10., 10., &p1, &p2);
    CHECK_EQUAL(0, status);
}
