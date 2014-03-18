#include "CppUTest/TestHarness.h"
#include <cstring>
#include <cmath>

extern "C" {
#include "../hardware.h"
}

#define STARTER_MASK 0x1000

TEST_GROUP(MiscHardwareTestGroup)
{
    int32_t device[6];

    void setup()
    {
        memset(device, 0, 6);
    }
};

TEST(MiscHardwareTestGroup, UARTRatePrecisionTest)
{
    int baudrate = 57600;
    int clock = 1000000;
    int real_baudrate;
    float error;

    cvra_set_uart_speed(device, clock, baudrate);

    real_baudrate = clock / (device[0x04]+1);
    error = fabs(real_baudrate - baudrate);

    error = error / (float)real_baudrate;

    // 5% seems to be max value for UART error rates. Take some margin
    CHECK(error < 4e-2);
}

TEST(MiscHardwareTestGroup, StarterCordAbsent)
{
    /* This should change depending on the wiring of the robot.
     * The starter pulls the pin to ground when present, so a logical one means
     * the starter is absent. */
    device[0] |= STARTER_MASK;
    CHECK_EQUAL(0, cvra_get_starter_cord(device));
}

