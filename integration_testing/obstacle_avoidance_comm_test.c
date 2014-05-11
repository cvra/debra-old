#include <stdio.h>
#include <lwip/sys.h>
#include <lwip/tcpip.h>
#include <lwip/ip.h>
#include <netif/tapif.h>
#include <netif/slipif.h>

#include "obstacle_avoidance_protocol.h"

struct netif slipf;
/** Shared semaphore to signal when lwIP init is done. */
sys_sem_t lwip_init_done;

void ipinit_done_cb(void *a)
{
    sys_sem_signal(&lwip_init_done);
}

void ip_stack_init(void)
{
    /* Netif configuration */
    static ip_addr_t ipaddr, netmask, gw;

    /* for some reason 192.168.4.9 fails on my test station. */
    IP4_ADDR(&gw, 10,0,0,1);
    IP4_ADDR(&ipaddr, 10,0,0,2);
    IP4_ADDR(&netmask, 255,255,255,0);

    /* Creates the "Init done" semaphore. */
    sys_sem_new(&lwip_init_done, 0);

    /* We start the init of the IP stack. */
    tcpip_init(ipinit_done_cb, NULL);

    /* We wait for the IP stack to be fully initialized. */
    printf("Waiting for LWIP init...\n");
    sys_sem_wait(&lwip_init_done);

    /* Deletes the init done semaphore. */
    sys_sem_free(&lwip_init_done);
    printf("LWIP init complete\n");

    /* Adds a tap pseudo interface for unix debugging. */
    netif_add(&slipf,&ipaddr, &netmask, &gw, NULL, tapif_init, tcpip_input);

    netif_set_default(&slipf);
    netif_set_up(&slipf);

}

void send_request(void)
{
    struct ip_addr server;
    int port;
    int i;

    IP4_ADDR(&server, 10,0,0,1);
    port = 2048;

    obstacle_avoidance_request_t request;
    obstacle_avoidance_request_create(&request, 0);

    obstacle_avoidance_path_t path;

    request.desired_datapoints = 3;
    request.desired_samplerate = 300;

    int err = obstacle_avoidance_send_request(&request, server, port, &path);

    printf("Request returned %d \n", err);

    for (i=0;i<path.len;i++)
        printf("x = %d, y = %d\n", path.points[i].x, path.points[i].y);

    obstacle_avoidance_delete_path(&path);
    obstacle_avoidance_request_delete(&request);
}


void main(void)
{
    printf("Testing communication\n");
    ip_stack_init();
    send_request();
}
