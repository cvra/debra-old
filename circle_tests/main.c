#include <stdio.h>
#include "../arm.h"

#define SIZE 30

void main(void)
{
    point_t target, p1, p2;
    int p;

    for (target.x = -SIZE; target.x < SIZE; target.x+=1) {
        for (target.y = -SIZE; target.y < SIZE; target.y+=1) {
            p = compute_possible_elbow_positions(target,10,10, &p1, &p2);
            printf("%d;%d;%d;", (int)target.x, (int)target.y, p);
            printf("\n");
        }
    }
}
