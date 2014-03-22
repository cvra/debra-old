#include "CppUTest/TestHarness.h"

extern "C" {
#include "../arm_cinematics.h"
}

TEST_GROUP(ChooseElbowPositionTestGroup)
{
};

TEST(ChooseElbowPositionTestGroup, ModeMirrorIdentity)
{
    shoulder_mode_t m = SHOULDER_FRONT;
    float angle = 1;
    CHECK_EQUAL(m, mode_for_orientation(m, angle));
}

TEST(ChooseElbowPositionTestGroup, ModeMirrorInvert)
{
    shoulder_mode_t m = SHOULDER_FRONT;
    float angle = -1;
    CHECK_EQUAL(SHOULDER_BACK, mode_for_orientation(m, angle));
}

TEST(ChooseElbowPositionTestGroup, ModeMirrorInvertBack)
{
    shoulder_mode_t m = SHOULDER_BACK;
    float angle = -1;
    CHECK_EQUAL(SHOULDER_FRONT, mode_for_orientation(m, angle));
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowSolutionOutsideRobot)
{
    // First test case : target_x < 0 ('inside' the robot)
    point_t elbow1 = {10, 10};
    point_t elbow2 = {-10, 10};
    point_t target = {-10, 20};
    point_t chosen;

    chosen = choose_shoulder_solution(target, elbow1, elbow2, SHOULDER_FRONT);
    CHECK_EQUAL(elbow1.x, chosen.x);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowSolutionOutsideRobotBis)
{
    // First test case : target_x < 0 ('inside' the robot)
    point_t elbow1 = {-10, 10};
    point_t elbow2 = {10, 10};
    point_t target = {-10, 20};
    point_t chosen;

    chosen = choose_shoulder_solution(target, elbow1, elbow2, SHOULDER_FRONT);
    CHECK_EQUAL(elbow2.x, chosen.x);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowBackward)
{
    point_t elbow1 = {10, 10};
    point_t elbow2 = {10, -10};
    point_t target = {10, 0};
    point_t chosen;

    chosen = choose_shoulder_solution(target, elbow1, elbow2, SHOULDER_BACK);
    CHECK_EQUAL(elbow1.y, chosen.y);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowBackwardBis)
{
    point_t elbow1 = {10, -10};
    point_t elbow2 = {10, 10};
    point_t target = {10, 0};
    point_t chosen;

    chosen = choose_shoulder_solution(target, elbow1, elbow2, SHOULDER_BACK);
    CHECK_EQUAL(elbow2.y, chosen.y);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowForward)
{
    point_t elbow1 = {10, -10};
    point_t elbow2 = {10, 10};
    point_t target = {10, 0};
    point_t chosen;

    chosen = choose_shoulder_solution(target, elbow1, elbow2, SHOULDER_FRONT);
    CHECK_EQUAL(elbow1.y, chosen.y);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowForwardBis)
{
    point_t elbow1 = {10, 10};
    point_t elbow2 = {10, -10};
    point_t target = {10, 0};
    point_t chosen;

    chosen = choose_shoulder_solution(target, elbow1, elbow2, SHOULDER_FRONT);
    CHECK_EQUAL(elbow2.y, chosen.y);
}
