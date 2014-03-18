#include "CppUTest/TestHarness.h"
#include "math.h"

extern "C" {
#include "../arm_cinematics.h"
}

#define RAD(x) ((x/180.)*PI)

TEST_GROUP(CinematicsTestGroup)
{
    float alpha, beta;

};

TEST(CinematicsTestGroup, FindsASolution)
{
    int status;
    status = compute_inverse_cinematics(100., 100., &alpha, &beta, 100., 100.);
    CHECK_EQUAL(0, status);
}

TEST(CinematicsTestGroup, FailsWhenTooFar)
{
    int status;
    status = compute_inverse_cinematics(100., 100., &alpha, &beta, 10., 10.);
    CHECK(status < 0);
}

TEST(CinematicsTestGroup, TrivialCase)
{
    int status = compute_inverse_cinematics(100., 100., &alpha, &beta, 50., 50.);
    CHECK_EQUAL(0, status);
    DOUBLES_EQUAL(0, beta, 0.01);
    DOUBLES_EQUAL(0, alpha, 0.01);
}

