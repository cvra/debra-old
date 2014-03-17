//      hardware.h
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
 * \file hardware.h
 *  This file is the header file for hardware.c
 */

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include <aversive.h>
#include "arm.h"


/** Sets the baudrate of a given UART
 *
 * @author Antoine Albertelli, CVRA
 * @param [in] uart_adress The base adress of the altera UART module.
 * @param [in] baudrate The desired baudrate
 */
void cvra_set_uart_speed(int32_t *uart_adress, int baudrate);

/** Reads the starting cord.
 *
 * @return 1 if the starting cord is present, zero otherwise.
 */
int cvra_get_starter_cord(void);

/** Waits for the starting cord to be pulled. */
void cvra_wait_starter_pull(void);

#endif
