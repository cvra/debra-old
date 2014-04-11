/** @file main.c
 * @author Antoine Albertelli
 * @author Florian "Flopy" Glardon
 * @date 2011
 * @brief Le fichier principal du programme.
 *
 * Ce fichier contient uniquement les fonctions de bases du code. Il se charge
 * de l'init des differents systemes et d'appeller le scheduling toutes les ms.
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <nios2.h>

#include <platform.h>
#include <aversive/error.h>
#include <fast_math.h>

#include <uptime.h>
#include <cvra_beacon.h>

#include <lwip/sys.h>
#include <lwip/tcpip.h>
#include <lwip/ip.h>
#include <netif/slipif.h>

#include "hardware.h"
#include "cvra_cs.h"
#include "arm_init.h"
#include "strat_utils.h"


#define   TASK_STACKSIZE          2048
#define   INIT_TASK_PRIORITY        20
#define   HEARTBEAT_TASK_PRIORITY   41

OS_STK    init_task_stk[TASK_STACKSIZE];
OS_STK    heartbeat_task_stk[TASK_STACKSIZE];

void init_task(void *pdata);
void heartbeat_task(void *pdata);

/** Serial net interface */
struct netif slipf;

/** Shared semaphore to signal when lwIP init is done. */
sys_sem_t lwip_init_done;


/** Logs an event.
 *
 * This function is never called directly, but instead, the error
 * modules fills an error structure and calls it.
 * @param [in] e The error structure, filled with every needed info.
 */
void mylog(struct error * e, ...)
{
    va_list ap;
    va_start(ap, e);
    /* Prints the filename (not the full path) and line number. */
    fprintf(stderr, "%s:%d ", strrchr(e->file, '/') ? strrchr(e->file, '/')+1:e->file, e->line);
    vfprintf(stderr, e->text, ap);
    fprintf(stderr, "\r\n");
    va_end(ap);
}

void heartbeat_task(void *pdata)
{
    OS_CPU_SR cpu_sr;
    int32_t led_val;
    while (1) {
        OS_ENTER_CRITICAL();
        led_val = IORD(LED_BASE, 0);
        led_val ^= (1 << 2);
        IOWR(LED_BASE, 0, led_val);
        OS_EXIT_CRITICAL();
        OSTimeDlyHMSM(0, 0, 0, 500);
    }

}

void ipinit_done_cb(void *a)
{
    sys_sem_signal(&lwip_init_done);
}

void list_netifs(void)
{
    struct netif *n; /* used for iteration. */
    for (n = netif_list; n != NULL; n = n->next) {
        /* Converts the IP adress to a human readable format. */
        char buf[16+1];
        ipaddr_ntoa_r(&n->ip_addr, buf, 17);
        printf("%s: %s\n", n->name, buf);
    }
}

void ip_stack_init(void)
{
    /* Netif configuration */
    static ip_addr_t ipaddr, netmask, gw;

    IP4_ADDR(&gw, 10, 0, 0, 11); // toradex board IP
    IP4_ADDR(&ipaddr, 10, 0, 0, 10);
    IP4_ADDR(&netmask, 255,255,255,255);

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
    netif_add(&slipf, &ipaddr, &netmask, &gw, NULL, slipif_init, tcpip_input);

    netif_set_default(&slipf);
    netif_set_up(&slipf);

}

void init_task(void *pdata)
{
    /* Tells the user that we are up and running. */
    WARNING(0, "System boot !");

#if 0
    /* Inits the custom math lib. */
    NOTICE(0, "Fast math init.");
    fast_math_init();
#endif

    cvra_beacon_init(&robot.beacon, AVOIDING_BASE, AVOIDING_IRQ, 100, 10., 0.);
    cvra_beacon_set_direction_offset(&robot.beacon, 123);



    ip_stack_init();
    list_netifs();

    /* If the logic power supply is off, kindly ask the user to turn it on. */
    if ((IORD(PIO_BASE, 0) & 0xff) == 0) {
        printf("Hey sac a pain, la commande c'est en option ?\n");

        /* We refuse to let the user to a shell before he turns it on. */
        while ((IORD(PIO_BASE, 0) & 0xff) == 0);
        printf("Merci bien !\n");
    }

    /* Inits all the trajectory stuff, PID, odometry, etc... */
#if 1
    NOTICE(0, "Main control system init.");
    cvra_cs_init();
#endif



    /* Sets the bounding box for the avoidance module. */
    const int robot_size = 150;
    polygon_set_boundingbox(robot_size, robot_size, 3000-robot_size, 2000-robot_size);

    arm_highlevel_init();

    lua_do_settings();

    luaconsole_init();

    OSTaskCreateExt(heartbeat_task,
                    NULL,
                    &heartbeat_task_stk[TASK_STACKSIZE-1],
                    HEARTBEAT_TASK_PRIORITY,
                    HEARTBEAT_TASK_PRIORITY,
                    &heartbeat_task_stk[0],
                    TASK_STACKSIZE,
                    NULL, NULL);

    /* Tasks must delete themselves before exiting. */
    OSTaskDel(OS_PRIO_SELF);
}

int main(void)
{

    robot.verbosity_level = ERROR_SEVERITY_NOTICE;

    /* Setup UART speed, must be first. */
    cvra_set_uart_speed(COMPC_BASE, PIO_FREQ, 57600);
    cvra_set_uart_speed(COMDEBUG_BASE, PIO_FREQ, 115200);
//    cvra_set_uart_speed(COMBT2_BASE, 9600); 

    /* Inits the logging system. */
    error_register_emerg(mylog);
    error_register_error(mylog);
    error_register_warning(mylog);
    error_register_notice(mylog);


    /* Enabling this one will cause a lot of logs from subsystems to show up. */
    //error_register_debug(mylog);


    OSTaskCreateExt(init_task,
                    NULL,
                    &init_task_stk[TASK_STACKSIZE-1],
                    INIT_TASK_PRIORITY,
                    INIT_TASK_PRIORITY,
                    &init_task_stk[0],
                    TASK_STACKSIZE,
                    NULL, NULL);

    OSStart();


    for (;;);


    /* We will never reach it. */
    return 0;
}

