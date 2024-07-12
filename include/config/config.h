#pragma once

#define DIM 3
#define R1 4
#define R2 4

constexpr int int_pow(int base, int exp)
{
    return (exp == 0) ? 1 : base * int_pow(base, exp - 1);
}

#define NUM_R1_VERTICES (int_pow(R1, DIM))
#define NUM_R2_VERTICES (int_pow(R1, DIM) * int_pow(R2, DIM))
#define VERBOSE 1
