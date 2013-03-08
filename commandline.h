#ifndef _COMMANDLINE_H_
#define _COMMANDLINE_H_

/** Inits the command line interface. */
void commandline_init(void);

/** Inputs a char to the command line interface.
 *
 * This functions adds a character to the command line interface it is typically
 * called from a UART receive interrupt.
 *
 * @note commandline_init() must have been called before any call to this function.
 */
void commandline_input_char(char c);

#endif
