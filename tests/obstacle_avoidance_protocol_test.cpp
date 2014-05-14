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
    r.end.x = 20;
    r.end.y = 21;
    json = obstacle_avoidance_request_encode(&r);
    STRCMP_EQUAL("[[42,98,0,0],[20,21],12,13,[]]", json);
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
    STRCMP_EQUAL("[[0,0,0,0],[0,0],0,0,[[1,2,3,4,150]]]", json);
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
    STRCMP_EQUAL("[[0,0,0,0],[0,0],0,0,[[0,0,0,0,0],[0,0,0,0,0]]]", json);

    free(json);
}

TEST(ObstacleAvoidanceProtocolTestGroup, CanDecodeEmptyPath)
{
    int ret;
    obstacle_avoidance_path_t p;

    ret = obstacle_avoidance_decode_path(&p, "[]");

    CHECK_EQUAL(0, p.len);
    CHECK_EQUAL(0, ret);
    obstacle_avoidance_delete_path(&p);
}

TEST(ObstacleAvoidanceProtocolTestGroup, CanDetectInvalidEncoding)
{
    int ret;
    obstacle_avoidance_path_t p;

    ret = obstacle_avoidance_decode_path(&p, "[1,2");
    CHECK_EQUAL(ERR_VAL, ret);

    ret = obstacle_avoidance_decode_path(&p, "1");
    CHECK_EQUAL(ERR_VAL, ret);
}

TEST(ObstacleAvoidanceProtocolTestGroup, CanParseSimplePath)
{
    obstacle_avoidance_path_t p;

    obstacle_avoidance_decode_path(&p, "[[1,2,3,4, 1000], [10, 11, 12, 13, 1000]]");
    CHECK_EQUAL(2, p.len);

    CHECK_EQUAL(1, p.points[0].x);
    CHECK_EQUAL(2, p.points[0].y);
    CHECK_EQUAL(3, p.points[0].vx);
    CHECK_EQUAL(4, p.points[0].vy);
    CHECK_EQUAL(1000, p.points[0].timestamp);

    CHECK_EQUAL(10, p.points[1].x);
    CHECK_EQUAL(11, p.points[1].y);
    CHECK_EQUAL(12, p.points[1].vx);
    CHECK_EQUAL(13, p.points[1].vy);
    CHECK_EQUAL(1000, p.points[1].timestamp);

    obstacle_avoidance_delete_path(&p);
}

TEST(ObstacleAvoidanceProtocolTestGroup, CanDeletePath)
{
    obstacle_avoidance_path_t p;
    obstacle_avoidance_decode_path(&p, "[[1,2,3,4,5], [10, 11, 12, 13,14]]");

    obstacle_avoidance_delete_path(&p);

    CHECK_EQUAL(0, p.len);
    POINTERS_EQUAL(NULL, p.points);
}
