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
#include <cvra_servo.h>
#include <cvra_beacon.h>
#include <commandline.h>

#include "hardware.h"
#include "cvra_cs.h"
#include "strat_utils.h"

extern command_t commands_list[];

#define   TASK_STACKSIZE          2048
#define   INIT_TASK_PRIORITY        20
#define   SHELL_TASK_PRIORITY       40
#define   HEARTBEAT_TASK_PRIORITY   41

OS_STK    init_task_stk[TASK_STACKSIZE];
OS_STK    shell_task_stk[TASK_STACKSIZE];
OS_STK    heartbeat_task_stk[TASK_STACKSIZE];

void shell_task(void *pdata);
void init_task(void *pdata);
void heartbeat_task(void *pdata);


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


void init_task(void *pdata)
{
    /* Tells the user that we are up and running. */
    WARNING(0, "System boot !");

#if 0
    /* Inits the custom math lib. */
    NOTICE(0, "Fast math init.");
    fast_math_init();
#endif

    cvra_beacon_init(&robot.beacon, AVOIDING_BASE, AVOIDING_IRQ, 1000, 1., 1.);

    /* If the logic power supply is off, kindly ask the user to turn it on. */
    if ((IORD(PIO_BASE, 0) & 0xff) == 0) {
        printf("Hey sac a pain, la commande c'est en option ?\n");

        /* We refuse to let the user to a shell before he turns it on. */
        while ((IORD(PIO_BASE, 0) & 0xff) == 0);
        printf("Merci bien !\n");
    }

    /* Inits all the trajectory stuff, PID, odometry, etc... */
#if 0
    NOTICE(0, "Main control system init.");
    cvra_cs_init();
#endif

    /* Sets the bounding box for the avoidance module. */
    const int robot_size = 150;
    polygon_set_boundingbox(robot_size, robot_size, 3000-robot_size, 2000-robot_size);

    OSTaskCreateExt(shell_task,
                    NULL,
                    &shell_task_stk[TASK_STACKSIZE-1],
                    SHELL_TASK_PRIORITY,
                    SHELL_TASK_PRIORITY,
                    &shell_task_stk[0],
                    TASK_STACKSIZE,
                    NULL, NULL);

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



void shell_task(void *pdata)
{
    /* Inits the commandline interface. */
    commandline_init(commands_list);

    /* Runs the commandline system. */
    for (;;) commandline_input_char(getchar());
}

void heartbeat_task(void *pdata)
{
    OS_CPU_SR cpu_sr;
    int leds;
    while (1) {
        OS_ENTER_CRITICAL();

        leds = IORD(LED_BASE, 0);
        /* toggles bit 0. */
        leds = leds ^ (1<<0);
        IOWR(LED_BASE, 0, leds);

        OS_EXIT_CRITICAL();
        OSTimeDlyHMSM(0, 0, 0, 500);
    }

}

int main(void)
{

    robot.verbosity_level = ERROR_SEVERITY_NOTICE;

    /* Setup UART speed, must be first. */
/*    cvra_set_uart_speed(COMBT1_BASE, 9600);
    cvra_set_uart_speed(COMBT2_BASE, 9600); */

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

