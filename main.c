/** @file main.c
 * @author Antoine Albertelli
 * @author Florian "Flopy" Glardon
 * @date 2011
 * @brief Le fichier principal du programme.
 *
 * Ce fichier contient uniquement les fonctions de bases du code. Il se charge
 * de l'init des differents systemes et d'appeller le scheduling toutes les ms.
 */

#include <aversive.h>
#include <aversive/error.h>
#include <scheduler.h>
#include <fast_math.h>
#include <stdio.h>
#include <stdarg.h>
#include <uptime.h>
#include <string.h>
#include <cvra_servo.h>
#include <cvra_beacon.h>

#include <commandline.h>

/* nios2.h contient toutes les fonctions dont nous avons besoin pour dialoguer
 * avec le nios alt_irq_register, IORD, etc... */
#ifdef COMPILE_ON_ROBOT
#include <nios2.h>
#endif

#include "hardware.h"
#include "cvra_cs.h"
#include "strat_utils.h"

extern command_t commands_list[];

void do_nothing(int32_t val) {

}

/** Logs an event.
 *
 * This function is never called directly, but instead, the error
 * modules fills an error structure and calls it.
 * @param [in] e The error structure, filled with every needed info.
 */
void mylog(struct error * e, ...) {
	va_list ap;
	va_start(ap, e);	
    /* Prints the filename (not the full path) and line number. */
    fprintf(stderr, "%s:%d ", strrchr(e->file, '/') ? strrchr(e->file, '/')+1:e->file, e->line);
	vfprintf(stderr, e->text, ap);
	fprintf(stderr, "\r\n");
	va_end(ap);
}
/** Cette variable contient le temps maximum passe dans une boucle du scheduler.
 * Elle donne donc une assez bonne indication de l'occupation du CPU. */
int32_t longest_scheduler_interrupt_time=0;

#ifdef COMPILE_ON_ROBOT
/** Cette fonction est appellee a chaque overflow du timer (toutes les ms).
 * @param param Unused parameter, but requested by the NIOSII API.
 */
void main_timer_interrupt(__attribute__((unused)) void *param) {
	static int i=0;
	int32_t time;

	/* Chenillard sur les LEDs */
	i++;
	IOWR(LED_BASE, 0, i/100);
	if(i==0xfe*100)
	   	i=0;

	/* Reset le timer. */
	IOWR(TICK_BASE, 0, 0x00); 

	/* Execute les taches programmees en mesurant le temps mis. */
	time = uptime_get(); 
	scheduler_interrupt();
	time = uptime_get() - time;

	/* Si le temps est plus grand que le maximum, on le garde en memoire. */
	if(time > longest_scheduler_interrupt_time) {
	 longest_scheduler_interrupt_time = time;
	}
}
#endif

/** @brief Demarre le robot
 *
 * Cette fonction est la premiere executee par notre code. Elle est responsable
 * de l'initialisation de tous les sous-systemes dans un ordre permettant leur
 * bon fonctionnement.
 *
 * @bug Est-ce qu'on peut se debarasser de argc et argv ? antoine
 * @param argc, argv Unused parameters, but requested by the Nios II API.
 * @returns Never returns 
 */ 
int main(__attribute__((unused)) int argc, __attribute__((unused)) char **argv) {

	robot.verbosity_level = ERROR_SEVERITY_NOTICE;
    
	/* Setup UART speed, must be first. */
    cvra_set_uart_speed(COMBT1_BASE, 9600);
    cvra_set_uart_speed(COMBT2_BASE, 9600);

    /* Inits the logging system. */
    error_register_emerg(mylog);
    error_register_error(mylog);
    error_register_warning(mylog);
    error_register_notice(mylog);

    /* Enabling this one will cause a lot of logs from subsystems to show up. */
    //error_register_debug(mylog);
 
	/* Inits the custom math lib. */
    fast_math_init();

	/* Starts multitasking. */
	scheduler_init(); 

#ifdef COMPILE_ON_ROBOT
	/* Setups system tick. */
	alt_ic_isr_register(0, TICK_IRQ, main_timer_interrupt, NULL, 0);
#endif

	/* Inits all the trajectory stuff, PID, odometry, etc... */
	cvra_cs_init(); 

    /* Inits the arms. */
    arm_highlevel_init();    

    /* Release the servo in case they were doing something. */
    strat_release_servo(LEFT);
    strat_release_servo(RIGHT);
 
    /* Sets the bounding box for the avoidance module. */ 
    const int robot_size = 150;
    polygon_set_boundingbox(robot_size, robot_size, 3000-robot_size, 2000-robot_size);

    /* Inits the beacon system. The 2 coefficients were found by calibration. */
    cvra_beacon_init(&robot.beacon, AVOIDING_BASE, AVOIDING_IRQ, 127, -5.8618, 109.43);

    /* Tells the user that we are up and running. */
    WARNING(0, "System boot !");

    /* Optionnal : if this is the default software, load a demo for the arms. */
//#define FLASH_DEMO
#ifdef FLASH_DEMO
    strat_look_cool();
#endif

    /* If the logic power supply is off, kindly ask the user to turn it on. */
    if((IORD(PIO_BASE, 0) & 0xff) == 0) {
        printf("Hey sac a pain, la commande c'est en option ?\n");

        /* We refuse to let the user to a shell before he turns it on. */
        while((IORD(PIO_BASE, 0) & 0xff) == 0);
        printf("Merci bien !\n");
    }

    /* Inits the commandline interface. */
    commandline_init(commands_list);

    /* Runs the commandline system. */
    for(;;) commandline_input_char(getchar());
    	
    /* We will never reach it. */
	return 0;
}

