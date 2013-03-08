/*
 * com_balises.c
 *
 *  Created on: 3 mai 2012
 *      Author: Antoine Albertelli
 */

#include <aversive.h>

#include <scheduler.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>


void beaconTask(void *dummy);

static int fileDescriptor;

typedef enum {
	MAGIC1=0,
	MAGIC2,
	MAGIC3,
	POS_X_FOE_1H,
	POS_X_FOE_1L,
	POS_Y_FOE_1H,
	POS_Y_FOE_1L,
	POS_A_FOE_1H,
	POS_A_FOE_1L,
	POS_X_FOE_2H,
	POS_X_FOE_2L,
	POS_Y_FOE_2H,
	POS_Y_FOE_2L
} transmit_state_t;

static transmit_state_t state;

int pos1X, pos1Y, pos1A, pos2X, pos2Y;

void init_beacons(char *device) {
	fileDescriptor = open(device, O_RDONLY | O_NONBLOCK | O_NOCTTY );
	if(fileDescriptor == -1) {
		printf("Error opening file.");

	}
	else
		scheduler_add_periodical_event(beaconTask, beaconTask, 1000);

	state = MAGIC1;
}


void beaconTask(__attribute__((unused)) void *dummy) {
	char buf;
	while(read(fileDescriptor, &buf, 1) > 0) {
		switch(state) {
		case MAGIC1:
			if(buf == 'A') {
				state = MAGIC2;
			}
			break;

		case MAGIC2:
			if(buf == 'B') {
				state = MAGIC3;
			} else {
				state = MAGIC1;
			}
			break;

		case MAGIC3:
			if(buf == 'C') {
				state = POS_X_FOE_1H;
			} else {
				state = MAGIC1;
			}
			break;

		case POS_X_FOE_1H:
			pos1X = buf << 8;
			state++;
			break;

		case POS_X_FOE_1L:
			pos1X |= buf;
			state++;
			break;

		case POS_Y_FOE_1H:
			pos1Y = buf << 8;
					state++;
					break;

		case POS_Y_FOE_1L:
					pos1Y |= buf;
					state++;
					break;

		case POS_A_FOE_1H:
			pos1A = buf << 8;
					state++;
					break;

		case POS_A_FOE_1L:
					pos1A |= buf;
					state++;
					break;


		case POS_X_FOE_2H:
			pos2X = buf << 8;
					state++;
					break;

		case POS_X_FOE_2L:
					pos2X |= buf;
					state++;
					break;

		case POS_Y_FOE_2H:
			pos2X = buf << 8;
					state++;
					break;

		case POS_Y_FOE_2L:
					pos2X |= buf;
					state = MAGIC1;
					printf("opponnent %d %d\r", pos1X, pos1Y);
					break;

		}
		putchar(buf);
	}
}
