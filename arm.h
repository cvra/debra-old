/** @file arm.h
 * @author Antoine Albertelli, CVRA
 * @date 2013
 * @brief Toutes les fonctions liees a la nouvelle version des bras pour la coupe 2013.
 * @sa doc/APi Bras.odt
 */
#ifndef _ARM_H_
#define _ARM_H_

#include <control_system_manager.h>
#include <pid.h>
#include <blocking_detection_manager.h>
#include <vect2.h>


#include "keyframe.h"

/** Structure represantant tout ce qui est necessaire pour controller un bras.
 * @todo Checker le padding. */
typedef struct {
    vect2_cart offset_xy; /** Le vecteur de decalage entre le referentiel du robot et celui des bras. */
    float offset_rotation; /** Rotation entre le référentiel du robot et des bras en radian. */

    /* Control systems */
    struct cs z_axis_cs;    /** Control System de l'axe Z. */
    struct cs shoulder_cs;  /** Control System de l'epaule. */
    struct cs elbow_cs;     /** Control System du coude. */
    
    struct pid_filter z_axis_pid;   /** PID de l'axe Z. */
    struct pid_filter shoulder_pid; /** PID de l'epaule. */
    struct pid_filter elbow_pid;    /** PID du coude. */
    
    struct blocking_detection z_axis_bd;    /** Gestionnaire du blocage de l'axe Z. */
    struct blocking_detection shoulder_bd;  /** Gestionnaire du blocage de l'epaule. */
    struct blocking_detection elbow_bd;     /** Gestionnaire du blocage du coude. */ 

    /* Physical parameters. */
    int32_t z_axis_imp_per_mm;      /** Nombre d'impulsions par mmm sur l'axe Z. */
    int32_t shoulder_imp_per_rad;   /** Nombre d'impulsion par rad sur l'epaule. */
    int32_t elbow_imp_per_rad;      /** Nombre d'impulsion par rad sur le coude. */
    int length[2];                  /** Longueurs des 2 bras. length[0] est epaule <-> coude. */

    /* Path informations */
    arm_trajectory_t trajectory;    /** La trajectoire que le bras est en train d'executer. */
    int32_t last_loop;              /** Date du dernier passage dans la boucle, en us. */

    /* Debug informations */
    char name[64]; /** Le nom du bras, par exemple "left" */
} arm_t;

/** Init tous les regulateurs d'un bras donne et copie le nom du bras.
 * @param [in, out] arm Le bras a preparer.
 * @param [in] name Le nom du bras.
 * 
 * @note Le module arm garde sa propre copie du nom, il n'est donc pas necessaire
 * que l'appelant la garde en memoire.
 */
void arm_init(arm_t *arm, const char *name);

/** Connecte les entrees sortie d'un bras au regulateur.
 * @param [in,out] arm Le bras a regler.
 * @param [in] z_set_pwn La fonction a utiliser pour regler la PWM de l'axe Z.,
 * @param [in] z_set_pwm_param Le parametre qui sera passer a z_set_pwm avant la valeur.
 * @param [in] z_get_coder La fonction a utiliser pour recuperer la valeur du codeur de l'axe z.
 * @param [in] z_get_coder_param Le parametre qui sera passer a z_get_encoder.
 * @param [in] shoulder_set_pwn La fonction a utiliser pour regler la PWM de l'epaule.
 * @param [in] shoulder_set_pwm_param Le parametre qui sera passer a shoulder_set_pwm avant la valeur.
 * @param [in] shoulder_get_coder La fonction a utiliser pour recuperer la valeur du codeur de l'epaule.
 * @param [in] shoulder_get_coder_param Le parametre qui sera passer a shoulder_get_encoder.
 * @param [in] elbow_set_pwn La fonction a utiliser pour regler la PWM du coude.
 * @param [in] elbow_set_pwm_param Le parametre qui sera passer a elbow_set_pwm avant la valeur.
 * @param [in] elbow_get_coder La fonction a utiliser pour recuperer la valeur du codeur du coude.
 * @param [in] elbow_get_coder_param Le parametre qui sera passer a elbow_get_encoder.
 *
 * @note arm_init() doit etre appelle avant arm_connect_io().
 */ 
void arm_connect_io(arm_t *arm, 
                    void (*z_set_pwm)(void *, int32_t), void *z_set_pwm_param,
                    int32_t (*z_get_coder)(void *), void *z_get_coder_param,
                    void (*shoulder_set_pwm)(void *, int32_t), void *shoulder_set_pwm_param,
                    int32_t (*shoulder_get_coder)(void *), void *shoulder_get_coder_param,
                    void (*elbow_set_pwm)(void *, int32_t), void *elbow_set_pwm_param,
                    int32_t (*elbow_get_coder)(void *), void *elbow_get_coder_param);


/** Execute la trajectoire d'un bras.
 * @param [in] arm Le bras a deplacer.
 * @param [in] traj Une structure arm_trajectory_t dont les keyframes sont remplies de facon a decrire
 * la trajectoire a effecture.
 *
 * @note Les frames fournie sont supposees triees de la plus ancienne a la plus recente. 
 *
 * @note Le module bras garde sa propre copie de la trajectoire en cours. Il n'est pas necessaire pour la
 * fonction appellante de la garder en memoire.
 *
 * @warning Aucune verification sur la trajectoire n'est faite avant le deplacement. Par consequent des
 * mouvements violents peuvent arriver en cas de donnees mal preparees.
 */
void arm_execute_movement(arm_t *arm, arm_trajectory_t *traj); 

/** Manage un bras.
 * 
 * Cette fonction doit etre appellee a frequence fixe. Elle est chargee de la regulation d'un bras.
 * Elle s'occupe alors de :
 * - Calculer le point actuel a utiliser, en faisant une interpolation lineaire des points de la
 *   trajectoire.
 * - Calculer la cinematique inverse pour ce point.
 * - Selectioner la solution du mouvement la plus adaptee.
 * - Regler la consigne du PID.
 *
 * @param [in] a Un pointeur vers une structure de type arm_t. Il est caste en void * pour etre
 * compatible avec le module scheduler.
 *
 * @note Cette fonction ne fait pas de regulation en elle meme. Il faut appeller arm_manage_cs() pour
 * que le regulateur soit pris en compte.
 *
 * @note Si le dernier point a ete depasse ou que le premier point n'a pas encore ete atteint, la
 * trajectoire est bornee sur les points de depart et d'arrivee.
 */  
void arm_manage(void *a);

/** Manage le regulateur d'un bras.
 * @param [in] a Un pointeur vers une structure de type arm_t. Il est caste en void * pour etre
 * compatible avec le module scheduler.
 * @note arm_manage() doit etre appelle de temps en temps afin de rafraichir les consignes du regulateur.
 * @note Si il n'y a aucune trajectoire chargee sur le bras, la regulation est desactivee. 
 */
void arm_manage_cs(void *a);

/** Recupere la position d'un bras.
 * @param [in] arm Le bras dont on veut recuperer la position.
 * @param [out] x L'adresse ou sera stocke la position du bras en X.
 * @param [out] y L'adresse ou sera stocke la position du bras en Y.
 * @param [out] z L'adresse ou sera stocke la position du bras en Z.
 *
 * @note Si un des pointeurs vaut NULL alors la coordonnee concernee ne sera pas recuperee. Ex :
 * arm_get_position(arm, &x, NULL, NULL); met la position en x dans la variable X et ne stocke pas les 
 * 2 autres.
 */
void arm_get_position(arm_t *arm, float *x, float *y, float *z);

/** Efface la trajectoire d'un bras et donc coupe la regulation. 
 *
 * @note Le bras ne s'arrete pas. Son regulateur est simplement coupe
 * et il continue sur son inertie. Cette fonction est donc prevue pour
 * les arrets de securite, comme par exemple a la fin du match ou avant
 * une collision.
 *
 * @note Renvoyer une trajectoire redemarre le bras automatiquement. */
void arm_shutdown(arm_t *arm);

#endif
