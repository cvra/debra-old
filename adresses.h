/** @file adresses.h
 * @author Antoine Albertelli <a.albertelli@cvra.ch
 * @date 2011
 * @brief Sert de couche d'abstraction pour permettre de renommer les
 * modules hardware sans devoir chercher dans tout le code pour trouver
 * les references.
 */
#ifndef _ADRESSES_H_
#define _ADRESSES_H_


#ifdef COMPILE_ON_ROBOT

/* System.h est un fichier cree par le toolchain qui contient les adresses
 * du hardware. */
#include <system.h>

#define LEFT_MOTOR_ADRESS   (int *)(MOTLEFT_BASE)       /**< Left motor board address */
#define RIGHT_MOTOR_ADRESS  (int *)(MOTRIGHT_BASE)      /**< Right motor board address */
#define ANALOG_SPI_ADRESS   (int *)(ANALOGIN_BASE)      /**< SPI board address for analog stuff  */
#define DIGITAL_OUTPUT0     (int *)(DIGITALOUT_BASE)    /**< Digital output board address */
#define Z_AXIS_DC_ADRESS    (int *)(ARMSHEIGHT_BASE)    /**< Z-axis board address */
#define LEFT_ARM_DC_ADRESS  (int *)(ARMLEFT_BASE)       /**< Left arm board address */
#define RIGHT_ARM_DC_ADRESS (int *)(ARMRIGHT_BASE)      /**< Right arm board address */

#else

/** Left motor board address */
#define LEFT_MOTOR_ADRESS   (int *)(0x0)
#define RIGHT_MOTOR_ADRESS  (int *)(0x0)
#define ANALOG_SPI_ADRESS   (int *)(0x0)
#define DIGITAL_OUTPUT0     (int *)(0x0)
#define Z_AXIS_DC_ADRESS    (int *)(0x0)
#define LEFT_ARM_DC_ADRESS  (int *)(0x0)
#define RIGHT_ARM_DC_ADRESS (int *)(0x0)

#endif



#endif
