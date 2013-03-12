/** @file adresses.h
 * @author Antoine Albertelli <a.albertelli@cvra.ch
 * @date 2011
 * @brief Sert de couche d'abstraction pour permettre de renommer les
 * modules hardware sans devoir chercher dans tout le code pour trouver
 * les references.
 */
#ifndef _ADRESSES_H_
#define _ADRESSES_H_


#ifndef COMPILE_ON_ROBOT

/* placeholders to check compilation. */
#define SERVOS_BASE (int *)(0)
#define ARMSMOTORCONTROLLER_BASE (int *)(0)
#define HEXMOTORCONTROLLER_BASE (int *)(0)
#define TIMECOUNTER_BASE (int *)(0)
#define ADCCONTROL_BASE (int *)(0)
#define PIO_BASE (int *)(0)

#endif

#endif
