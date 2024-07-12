#include <iostream>
#include "planners/KGMT.cuh"
int main(void)
{
    float h_initial[SAMPLE_DIM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, h_goal[SAMPLE_DIM] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    float h_ws = 10.0;
    KGMT kgmt(h_ws, 10, 30000, 0.5, 1);
    kgmt.plan(h_initial, h_goal);
    return 0;
}