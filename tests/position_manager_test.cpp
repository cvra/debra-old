#include "CppUTest/TestHarness.h"

extern "C" {
#include "2wheels/position_manager.h"
#include "2wheels/robot_system.h"
}

TEST_GROUP(PositionManagerTestGroup)
{
    struct robot_system rs;
    struct robot_position pos;

    void setup()
    {
        rs_init(&rs);
        position_init(&pos);
        position_set_related_robot_system(&pos,&rs);
    }
};

TEST(PositionManagerTestGroup, LockAreReleased)
{
    position_manage(&pos);
    CHECK_EQUAL(1, pos.lock.count);
    CHECK_EQUAL(1, pos.lock.acquired_count);
    position_set(&pos, 0, 0, 0);
    CHECK_EQUAL(1, pos.lock.count);
    CHECK_EQUAL(2, pos.lock.acquired_count);
}
