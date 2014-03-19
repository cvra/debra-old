#include "arm_utils.h"


point_t arm_coordinate_robot2arm(point_t target_point, vect2_cart offset_xy, float offset_angle)
{
    vect2_cart target;
    vect2_pol target_pol;
    target.x = target_point.x;
    target.y = target_point.y;

    vect2_sub_cart(&target, &offset_xy, &target);
    vect2_cart2pol(&target, &target_pol);

    target_pol.theta -= offset_angle;

    vect2_pol2cart(&target_pol, &target);

    target_point.x = target.x;
    target_point.y = target.y;

    return target_point;
}
