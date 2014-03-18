#include <platform.h>
#include <math.h>
#include <circles.h>



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

int compute_possible_elbow_positions(float x, float y, float l1, float l2,
                                        point_t *p1, point_t *p2)
{
    circle_t c1, c2;
    int nb_pos;

    c1.x = 0;
    c1.y = 0;
    c1.r = l1;

    c2.x = x;
    c2.y = y;
    c2.r = l2;

    return circle_intersect(&c1, &c2, p1, p2);
}
