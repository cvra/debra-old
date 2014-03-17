//      hardware.c
//
//      Copyright 2009 Antoine Albertelli <a.albertelli@cvra.ch>
//
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.

/**
 * \file hardware.c
 * \brief This files contains the hardware functions.
 *
 * This files contains functions to do the hardware init of the board, to set the pwm sign of the VNH2SP30E, and to set the LEDS.
 */

#include <aversive.h>
#include <aversive/error.h>
#include <general_errors.h>

// UART are in an another avalon io clock domain
#define UART_FREQ PIO_FREQ

#define STARTER_BITMASK 0x1000

void cvra_set_uart_speed(int32_t *uart_adress, int baudrate) {
#ifdef COMPILE_ON_ROBOT
    int32_t divisor;
        /* Formule tiree du Embedded IP User Guide page 7-4 */
        divisor = (int32_t)(((float)UART_FREQ/(float)baudrate) + 0.5);
        IOWR(uart_adress, 0x04, divisor); // ecrit le diviseur dans le bon registre
#endif
    DEBUG(E_UART, "Changed UART speed to %d at adress %p\n", baudrate, uart_adress);
}

int cvra_get_starter_cord(void)
{
    return ((IORD(PIO_BASE, 0) & STARTER_BITMASK) == 0);
}

void cvra_wait_starter_pull(void)
{
    while(cvra_get_starter_cord());
}
