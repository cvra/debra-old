#include "CppUTest/TestHarness.h"

TEST_GROUP(FirstTestGroup)
{

};

TEST(FirstTestGroup, FirstTest)
{
    CHECK(true);
    CHECK_EQUAL(1,1);
    LONGS_EQUAL(1,1);
    DOUBLES_EQUAL(1.000, 1.001, .01);
    STRCMP_EQUAL("hello", "hello");
}
