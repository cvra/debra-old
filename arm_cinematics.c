#include <platform.h>
#include <math.h>
#include <circles.h>
#include "arm_cinematics.h"



int compute_inverse_cinematics(float x, float y, float *alpha, float *beta, const float l1, const float l2) {
    circle_t c1, c2;
    point_t p1, p2, chosen;

    c1.x = 0;
    c1.y = 0;
    c1.r = l1;

    c2.x = x;
    c2.y = y;
    c2.r = l2;

    int nbPos = circle_intersect(&c1, &c2, &p1, &p2);

    if(nbPos == 0)
        return -1;

#if 0
    if (nbPos == 1)
        chosen = p1;
    else
        chosen = choose_shoulder_solution(x, y, p1, p2, arm->shoulder_mode, arm->offset_rotation);
#endif


    *alpha = atan2f(chosen.y, chosen.x);
    *beta = atan2f(y-chosen.y, x-chosen.x); // tres tres sensible aux erreurs d'arrondis
    *beta = *beta - *alpha;

    if(*beta < -M_PI) {
        *beta = 2*M_PI + *beta;
    }
    if(*beta > M_PI) {
        *beta = 2*M_PI - *beta;
    }

    return 0;
}

int compute_possible_elbow_positions(point_t target, float l1, float l2,
                                        point_t *p1, point_t *p2)
{
    circle_t c1, c2;
    int nb_pos;

    c1.x = 0;
    c1.y = 0;
    c1.r = l1;

    c2.x = target.x;
    c2.y = target.y;
    c2.r = l2;

    return circle_intersect(&c1, &c2, p1, p2);
}

shoulder_mode_t mode_for_orientation(shoulder_mode_t mode, float arm_angle_offset)
{
    if (arm_angle_offset > 0.)
        return mode;

    if (mode == SHOULDER_BACK)
        return SHOULDER_FRONT;

    return SHOULDER_BACK;
}

point_t choose_shoulder_solution(point_t target, point_t elbow1,
                                 point_t elbow2, shoulder_mode_t mode)
{

    if (target.x < 0) {
        if (elbow1.x > elbow2.x)
            return elbow1;
        else
            return elbow2;
    }

    if (mode == SHOULDER_BACK) {
        if (elbow1.y > elbow2.y)
            return elbow1;
        else
            return elbow2;
    } else {
        if (elbow2.y > elbow1.y)
            return elbow1;
        else
            return elbow2;
    }

}

float compute_shoulder_angle(point_t elbow, point_t hand)
{
    return atan2f(elbow.y, elbow.x);
}

float compute_elbow_angle(point_t elbow, point_t hand)
{
    float dx, dy;
    dx = hand.x - elbow.x;
    dy = hand.y - elbow.y;
    return atan2f(dy, dx); // tres tres sensible aux erreurs d'arrondis
}
