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

TEST(CinematicsTestGroup, SimpleShoulderAngle)
{
    float angle;
    point_t elbow, hand;
    elbow.x = 100;
    elbow.y = 0;
    angle = compute_shoulder_angle(elbow, hand);

    DOUBLES_EQUAL(0, angle, 1e-3);
}

TEST(CinematicsTestGroup, SimpleElbowAngle)
{
    float angle;
    point_t elbow = {.x=10, .y=10};
    point_t hand =  {.x=20, .y=20};

    angle = compute_elbow_angle(elbow, hand);
    DOUBLES_EQUAL(RAD(45), angle, 1e-3);
}

TEST(CinematicsTestGroup, ModeMirrorIdentity)
{
    shoulder_mode_t m = SHOULDER_FRONT;
    float angle = 1;
    CHECK_EQUAL(m, mode_for_orientation(m, angle));
}

TEST(CinematicsTestGroup, ModeMirrorInvert)
{
    shoulder_mode_t m = SHOULDER_FRONT;
    float angle = -1;
    CHECK_EQUAL(SHOULDER_BACK, mode_for_orientation(m, angle));
}

TEST(CinematicsTestGroup, ModeMirrorInvertBack)
{
    shoulder_mode_t m = SHOULDER_BACK;
    float angle = -1;
    CHECK_EQUAL(SHOULDER_FRONT, mode_for_orientation(m, angle));
}
