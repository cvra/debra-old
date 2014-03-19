#include "CppUTest/TestHarness.h"
#include <cmath>

extern "C" {
#include "../arm_utils.h"
}

TEST_GROUP(ArmUtilsTestGroup)
{
};

TEST(ArmUtilsTestGroup, CoordinateRobotToArm)
{
    point_t target = {.x=100,.y=100};
    vect2_cart offset_xy = {.x=0,.y=100};
    float offset_angle = M_PI/2.;
    point_t result;

    result = arm_coordinate_robot2arm(target, offset_xy, offset_angle);
    DOUBLES_EQUAL(0., result.x, 1e-2);
    DOUBLES_EQUAL(-100., result.y, 1e-2);
}
