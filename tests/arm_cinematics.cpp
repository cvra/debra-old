#include "CppUTest/TestHarness.h"
#include "math.h"

extern "C" {
#include "../arm_cinematics.h"
}

#define RAD(x) ((x/180.)*PI)

TEST_GROUP(CinematicsTestGroup)
{
    float alpha, beta;
    point_t p1, p2;
    int status;
};

TEST(CinematicsTestGroup, FindsASolution)
{
    status = compute_possible_elbow_positions(100., 100., 100., 100., &p1, &p2);
    CHECK_EQUAL(2, status);
}

TEST(CinematicsTestGroup, FindsSingleSolution)
{
    status = compute_possible_elbow_positions(100., 0., 50., 50., &p1, &p2);
    CHECK_EQUAL(1, status);
}

TEST(CinematicsTestGroup, FailsWhenTooFar)
{
    status = compute_possible_elbow_positions(100., 100., 10., 10., &p1, &p2);
    CHECK_EQUAL(0, status);
}


