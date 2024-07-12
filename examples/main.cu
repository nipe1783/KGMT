#include <iostream>
#include "planners/KGMT.cuh"
int main(void)
{
    float h_initial[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float h_goal[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    float h_ws = 10.0;
    KGMT kgmt(h_ws);
    kgmt.plan(h_initial, h_goal);
    return 0;
}