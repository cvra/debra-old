#include "CppUTest/TestHarness.h"
#include "cstring"

extern "C" {
#include <cvra_dc.h>
}

#define PWM0_REG 0x03
#define DC_PWM_MAX_VALUE 512

TEST_GROUP(CvraDcTestGroup)
{
    int device[21];


    void setup()
    {
        memset(device, 0, sizeof(device));
    }
};

TEST(CvraDcTestGroup, CanSetPWM)
{
    cvra_dc_set_pwm(device, 0, 42);
    CHECK_EQUAL(device[PWM0_REG], 42);
}

TEST(CvraDcTestGroup, PWMIsCapped)
{
    cvra_dc_set_pwm(device, 0, 2000);
    CHECK(device[PWM0_REG] < DC_PWM_MAX_VALUE);
}

TEST(CvraDcTestGroup, ChanIsCapped)
{
    cvra_dc_set_pwm(device, 880, 42);
}
