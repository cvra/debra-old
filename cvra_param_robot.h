/*
 * cvra_param_robot.h
 *
 *  Created on: 10 avr. 2012
 *      Author: Flopy
 */

#ifndef CVRA_PARAM_ROBOT_H_
#define CVRA_PARAM_ROBOT_H_

//*******************************************************************************************
//                                             ROBOT
//*******************************************************************************************
//                       *********************************************
//                                             PID
//                       *********************************************
#define ROBOT_PID_DIST_P 250
#define ROBOT_PID_DIST_I 0
#define ROBOT_PID_DIST_D 400

#define ROBOT_PID_ANGL_P 200
#define ROBOT_PID_ANGL_I 0
#define ROBOT_PID_ANGL_D 400

//                       *********************************************
//                                         Dimensions
//                       *********************************************
#define ROBOT_ECART_ROUE 275.7880216836 //275.1716841994 // 276.8397370540  276
#define ROBOT_INC_MM 162.73750397497828327709 //162.487138584247  //Calculé: 162.9746

#define ROBOT_WHEEL_L_CORR 1.00107255365769 //1.00172702807009
#define ROBOT_WHEEL_R_CORR -0.998927446342313 //-0.99827297192991

//                       *********************************************
//                                             Blocage
//                       *********************************************
#define ROBOT_ANGLE_BD 10000
#define ROBOT_DIST_BD 5000

//                       *********************************************
//                                    Vitesse/Accélérations
//                       *********************************************
//#define ROBOT_DIST_SPEED_FAST 500 // MM PAR S
//#define ROBOT_DIST_SPEED_SLOW 300
//#define ROBOT_ANGL_SPEED_FAST 300 // DEG PAR S
//#define ROBOT_ANGL_SPEED_SLOW 100

//#define ROBOT_DIST_ACC 8 // MM PAR S^2
//#define ROBOT_ANGL_ACC 1 // DEG PAR S^2  // 4 normalement

//*******************************************************************************************
//                                             ARM
//*******************************************************************************************
//                       *********************************************
//                                             PID
//                       *********************************************
#define ROBOT_ARM_PID_Z_P -1000
#define ROBOT_ARM_PID_Z_I -40
#define ROBOT_ARM_PID_Z_D -100

#define ROBOT_ARM_PID_SHOUL_P 100
#define ROBOT_ARM_PID_SHOUL_I 2//8
#define ROBOT_ARM_PID_SHOUL_D 80

#define ROBOT_ARM_PID_ELB_P 100
#define ROBOT_ARM_PID_ELB_I 2//8
#define ROBOT_ARM_PID_ELB_D 80

//                       *********************************************
//                                         Dimensions
//                       *********************************************


#define ROBOT_ARM_LENGTH_0 153 /** Longueur epaule->coude en mm */
#define ROBOT_ARM_LENGTH_1 189 /** Longueur coude->main en mm */
#define ROBOT_ARM_LENGTH_DEC 25 /**Décalage pour une prise virtuelle plus loins, en mm */
#define ROBOT_ARM_OBJ_APPROCHE 40 /** Distance d'approche en dessus de l'objet à prendre, en mm */

#define ROBOT_ARM_Z_INC_MM -1639 /** Impulsions par mm sur l'axe Z. */
#define ROBOT_ARM_SHOULD_INC_RAD -77785 /** Impulsions par radian sur l'epaule. */
#define ROBOT_ARM_ELB_INC_RAD 56571 /** Impulsions par radian sur le coude */

#define ROBOT_ARM_OFFSET_X 127

//                       *********************************************
//                                      Position initiales
//                       *********************************************
#define ROBOT_ARM_POS_RESET_SHOULDER 0.0f /** Position de reset de l'epaule, en rad */
#define ROBOT_ARM_POS_RESET_ELBOW 0.0f	/** Position de reset du coude, en rad */
#define ROBOT_ARM_POS_RESET_Z 200 /** Position de reset de l'axe Z, en mm */
//                       *********************************************
//                                             Blocage
//                       *********************************************
//#define ROBOT_ARM_ELB_BD 8000 /** Seuil de blocage sur le coude. */
//#define ROBOT_ARM_SHOUL_BD 5000  /** Seuil de blocage sur l'epaule */
//#define ROBOT_ARM_Z_BD 2000  /** Seuil de blocage sur l'axe z */
//#define ROBOT_ARM_Z_BD_FAST 10000  /** Seuil de blocage sur l'axe z */
//#define ROBOT_ARM_CMT_BD 3   /** Compteur avant d'activer le choc */
//#define ROBOT_ARM_INIT_BD 200  /** Seuil de blocage pour l'initialisation */

//                       *********************************************
//                                             Butées
//                       *********************************************
/* Constante pour la prise de risques :) En rad et mm*/
#define ROBOT_ARM_BUTEE_Z (212)			/** Butee sur l'axe Z, en mm */
#define ROBOT_ARM_BUTEE_EPAULE 1.884955592f  /** Butee sur l'epaule en rad (equivalent a 110 deg) */
#define ROBOT_ARM_BUTEE_COUDE  6.28318f  /** Desactive via une grand valeur, sinon 150 deg */

#define ROBOT_ARM_RANGE_ALPHA 107.0f
#define ROBOT_ARM_RANGE_BETA 60.0f

#define NB_COINS 38
#define NB_LINGOT 7
#define NB_POS_PRISE 4

#define TABLE_X_MM 3000
#define TABLE_Y_MM 2000
#endif /* CVRA_PARAM_ROBOT_H_ */
