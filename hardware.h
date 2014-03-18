#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include <platform.h>


/** Sets the baudrate of a given UART
 * @warning UART base frequency might not be the base frequency of the system.
 */
void cvra_set_uart_speed(int32_t *uart_adress, int uart_freq, int baudrate);

/** Reads the starting cord.
 *
 * @return 1 if the starting cord is present, zero otherwise.
 */
int cvra_get_starter_cord(int32_t *base);

/** Waits for the starting cord to be pulled. */
void cvra_wait_starter_pull(int32_t *base);

#endif
