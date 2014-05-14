#include <stdlib.h>
#include <string.h>
#include <lwip/tcpip.h>
#include <lwip/ip.h>
#include <lwip/sys.h>
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

    array_head = json_mkarray();
    json_append_element(array_head, json_mknumber(r->end.x));
    json_append_element(array_head, json_mknumber(r->end.y));
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

int obstacle_avoidance_decode_path(obstacle_avoidance_path_t *path,const char *json)
{
    JsonNode *node, *n;
    int i;

    node = json_decode(json);

    if (node == NULL || node->tag != JSON_ARRAY)
        return ERR_VAL;

    path->len = json_get_length(node);

    path->points = malloc(sizeof(obstacle_avoidance_point_t) * path->len);

    i = 0;
    json_foreach(n, node) {
        path->points[i].x  = json_find_element(n, 0)->number_;
        path->points[i].y  = json_find_element(n, 1)->number_;
        path->points[i].vx = json_find_element(n, 2)->number_;
        path->points[i].vy = json_find_element(n, 3)->number_;
        path->points[i].timestamp = json_find_element(n, 4)->number_;
        i++;
    }

    return ERR_OK;
}

int obstacle_avoidance_send_request(obstacle_avoidance_request_t *request, struct ip_addr remote_ip, int port, obstacle_avoidance_path_t *path)
{
    /* XXX find a better way to do this. */
    #define MAX_LEN 2048
    char *data;
    static char answer[MAX_LEN];
    void *tmp;
    struct netconn *conn = NULL;

    struct netbuf *buf;
    u16_t len;
    int err;

    data = obstacle_avoidance_request_encode(request);

    conn = netconn_new(NETCONN_TCP);

    err = netconn_connect(conn, &remote_ip, 2048);
    if (err != ERR_OK) {
        free(data);
        return err;
    }

    err = netconn_write(conn, data, strlen(data), NETCONN_COPY);

    if (err != ERR_OK) {
        free(data);
        return err;
    }

    free(data);

    int answer_offset = 0;
    while ((err = netconn_recv(conn, &buf)) == ERR_OK) {
        do {
            /* Copies the buffer data into a string. */
            netbuf_data(buf, &tmp, &len);
            if (answer_offset + len >= MAX_LEN) {   /* >= for null-terminator */
                return ERR_BUF
            }
            memcpy(answer + answer_offset, tmp, len);
            answer_offset += len;
        } while (netbuf_next(buf) >= 0);
        answer[answer_offset] = '\0';
        netbuf_delete(buf);
    }

    err = obstacle_avoidance_decode_path(path, answer);
    netconn_delete(conn);
    return err;
}

void obstacle_avoidance_delete_path(obstacle_avoidance_path_t *path)
{
    path->points = NULL;
    path->len = 0;
    free(path->points);
}
