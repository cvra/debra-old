#ifndef _ARM_CINEMATICS_H_
#define _ARM_CINEMATICS_H_


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
#endif
