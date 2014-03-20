#include "CppUTest/TestHarness.h"

extern "C" {
#include <platform.h>
}

TEST_GROUP(SempahoresTestGroup)
{
    semaphore_t sem;
};

TEST(SempahoresTestGroup, CanCreateSemaphore)
{
    sem.count = 0xff;
    sem.acquired_count = 0xff;
    platform_create_semaphore(&sem, 0);
    CHECK_EQUAL(0, sem.count);
}

TEST(SempahoresTestGroup, CanSignalSemaphre)
{
    platform_create_semaphore(&sem, 0);
    platform_signal_semaphore(&sem);
    CHECK_EQUAL(1, sem.count);
}

TEST(SempahoresTestGroup, CanTakeSemaphore)
{
    platform_create_semaphore(&sem, 1);
    platform_take_semaphore(&sem);
    CHECK_EQUAL(0, sem.count);
    CHECK_EQUAL(1, sem.acquired_count);
}



