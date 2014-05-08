#include "CppUTest/TestHarness.h"
#include <cstring>
#include <iostream>

extern "C" {
#include "../obstacle_avoidance_protocol.h"
}

TEST_GROUP(ObstacleAvoidanceProtocolTestGroup) {


};

TEST(ObstacleAvoidanceProtocolTestGroup, CanCreateObstacleAvoidanceRequest)
{
    obstacle_avoidance_request_t r;
    obstacle_avoidance_request_create(&r, 4);
    CHECK(r.obstacles != NULL);
    CHECK_EQUAL(4, r.obstacle_count);
}

TEST(ObstacleAvoidanceProtocolTestGroup, CanFreeObstacles)
{
    obstacle_avoidance_request_t r;
    obstacle_avoidance_request_create(&r, 4);
    obstacle_avoidance_request_delete(&r);
    POINTERS_EQUAL(NULL, r.obstacles);
    CHECK_EQUAL(0, r.obstacle_count);
}

TEST(ObstacleAvoidanceProtocolTestGroup, CanEncodeEmptyRequest)
{
    obstacle_avoidance_request_t r;
    char *json;

    obstacle_avoidance_request_create(&r, 0);
    r.desired_samplerate = 12;
    r.desired_datapoints = 13;
    r.start.x = 42;
    r.start.y = 98;
    json = obstacle_avoidance_request_encode(&r);
    STRCMP_EQUAL("[[42,98,0,0],12,13,[]]", json);
    free(json);
}

TEST(ObstacleAvoidanceProtocolTestGroup, CanEncodeObstacles)
{
    obstacle_avoidance_request_t r;
    char *json;

    obstacle_avoidance_request_create(&r, 1);

    r.obstacles[0].x = 1;
    r.obstacles[0].y = 2;
    r.obstacles[0].vx = 3;
    r.obstacles[0].vy = 4;
    r.obstacles[0].r = 150;

    json = obstacle_avoidance_request_encode(&r);
    STRCMP_EQUAL("[[0,0,0,0],0,0,[[1,2,3,4,150]]]", json);
    free(json);
}

TEST(ObstacleAvoidanceProtocolTestGroup, MultipleObstaclesToo)
{
    obstacle_avoidance_request_t r;
    char *json;
    const int len = 2;

    obstacle_avoidance_request_create(&r, len);

    memset(r.obstacles, 0, len * sizeof(obstacle_avoidance_obstacle_t));

    json = obstacle_avoidance_request_encode(&r);
    STRCMP_EQUAL("[[0,0,0,0],0,0,[[0,0,0,0,0],[0,0,0,0,0]]]", json);

    free(json);
}
