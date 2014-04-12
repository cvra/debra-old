#include "hardware.h"

#define STARTER_BITMASK (1 << 12)

void cvra_set_uart_speed(int32_t *uart_adress, int uart_freq, int baudrate)
{
    int32_t divisor;
    /* Formule tiree du Embedded IP User Guide page 7-4 */
    divisor = (int32_t)(((float)uart_freq/(float)baudrate) + 0.5);
    IOWR(uart_adress, 0x04, divisor); // ecrit le diviseur dans le bon registre
}

int cvra_get_starter_cord(int32_t *base)
{
    return ((IORD(base, 0) & STARTER_BITMASK) == 0);
}

void cvra_wait_starter_pull(int32_t *base)
{
    while(cvra_get_starter_cord(base));
}
