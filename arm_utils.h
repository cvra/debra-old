#ifndef _ARM_UTILS_H_
#define _ARM_UTILS_H_

#include <platform.h>
#include <vect2.h>
#include <polygon.h>

point_t arm_coordinate_robot2arm(point_t target_point, vect2_cart offset_xy, float offset_angle);

#endif
