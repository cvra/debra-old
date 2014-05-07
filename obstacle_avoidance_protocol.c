#include <stdlib.h>
#include <string.h>
#include "obstacle_avoidance_protocol.h"
#include "json.h"


void obstacle_avoidance_request_create(obstacle_avoidance_request_t *r, int obstacle_count)
{
    memset(r, 0, sizeof(obstacle_avoidance_request_t));
    r->obstacles = malloc(sizeof(obstacle_avoidance_obstacle_t) * obstacle_count);
    r->obstacle_count = obstacle_count;
}

void obstacle_avoidance_request_delete(obstacle_avoidance_request_t *r)
{
    free(r->obstacles);
    r->obstacles = NULL;
    r->obstacle_count = 0;
}

char *obstacle_avoidance_request_encode(obstacle_avoidance_request_t *r)
{
    JsonNode *node;
    JsonNode *array_head;
    JsonNode *obstacle_head;
    char *ret;
    int i;
    node = json_mkarray();

    array_head = json_mkarray();

    /* Start coordinates */
    json_append_element(array_head, json_mknumber(r->start.x));
    json_append_element(array_head, json_mknumber(r->start.y));
    json_append_element(array_head, json_mknumber(r->start.vx));
    json_append_element(array_head, json_mknumber(r->start.vy));
    json_append_element(node, array_head);

    json_append_element(node, json_mknumber(r->desired_samplerate));
    json_append_element(node, json_mknumber(r->desired_datapoints));

    array_head = json_mkarray();
    for (i=0;i<r->obstacle_count;i++) {
        obstacle_head = json_mkarray();
        json_append_element(obstacle_head, json_mknumber(r->obstacles[i].x));
        json_append_element(obstacle_head, json_mknumber(r->obstacles[i].y));
        json_append_element(obstacle_head, json_mknumber(r->obstacles[i].vx));
        json_append_element(obstacle_head, json_mknumber(r->obstacles[i].vy));
        json_append_element(obstacle_head, json_mknumber(r->obstacles[i].r));
        json_append_element(array_head, obstacle_head);
    }

    json_append_element(node, array_head);

    ret = json_encode(node);
    json_delete(node);
    return ret;
}
