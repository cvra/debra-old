#ifndef _ARM_CINEMATICS_H_
#define _ARM_CINEMATICS_H_

#include <vect_base.h>

typedef enum {
    SHOULDER_FRONT,
    SHOULDER_BACK,
} shoulder_mode_t;


/** Computes the inverse cinematics of an arm.
 *
 * The computation is done using the simplest algorithm : The intersection of
 * two circles, one being centered on shoulder, the other being centered on
 * target coordinates.
 *
 * When the intersection is known, computing angles is a simple matter of using
 * atan2 in a correct way.
 *
 * @param [in] arm The arm instance.
 * @param [in] x, y, z The coordinates of the target.
 * @param [out] alpha, beta Pointers where angles will be stored, alpha is shoulder.
 *
 * @return 0 en cas de succes, -1 en cas d'erreur.
 *
 * @todo We need an algorithm to choose between the 2 positions in case there
 * are multiple possibilities.
 */
int compute_inverse_cinematics(float x, float y,
            float *alpha, float *beta, const float l1, const float l2);


/** Computes the possible positions for the elbow.
 *
 * @param [in] x,y The coordinates of the hand.
 * @param [in] l1,l2 The length of the two parts of the arm.
 * @param [out] p1, p2 The possible positions.
 * @returns The number of possible positons (0, 1 or 2).
 */
int compute_possible_elbow_positions(float x, float y, float l1, float l2, point_t *p1, point_t *p2);

/**Inverts the shoulder mode depending on wheter the arm is facing the
 * left or right side of the robot. This is needed as "forward" and "backward"
 * are in robot frame while the transformations should be applied in arm frame.
 */
shoulder_mode_t mode_for_orientation(shoulder_mode_t mode, float arm_angle_offset);


float compute_shoulder_angle(point_t elbow, point_t hand);
float compute_elbow_angle(point_t elbow, point_t hand);

#endif
