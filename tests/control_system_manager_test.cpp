#include "CppUTest/TestHarness.h"

extern "C" {
#include <control_system_manager.h>
}


TEST_GROUP(ControlSystemTestGroup)
{
    struct cs cs;

    void setup(void)
    {
        cs_init(&cs);
    }
};

TEST(ControlSystemTestGroup, IsEnabledAtStartup)
{
    CHECK(cs.enabled);
}
