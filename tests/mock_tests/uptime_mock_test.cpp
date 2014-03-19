#include "CppUTest/TestHarness.h"

extern "C" {
#include "uptime.h"
}

TEST_GROUP(UptimeMockTestGroup)
{
    void teardown()
    {
        uptime_set(0);
    }
};


TEST(UptimeMockTestGroup, ReturnsZeroWithoutSettings)
{
    CHECK_EQUAL(0, uptime_get());
}

TEST(UptimeMockTestGroup, CanSetTime)
{
    uptime_set(42);
    CHECK_EQUAL(42, uptime_get());

}
