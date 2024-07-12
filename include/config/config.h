#pragma once

#define DIM 3
#define R1 4
#define R2 4
#define SAMPLE_DIM 7

#define NUM_R1_VERTICES ((DIM == 3) ? (R1 * R1 * R1) : (R1 * R1))
#define NUM_R2_VERTICES ((DIM == 3) ? (R1 * R1 * R1 * R2 * R2 * R2) : (R1 * R1 * R2 * R2))
#define R2_PER_R1 ((DIM == 3) ? (R2 * R2 * R2) : (R2 * R2))
#define EPSILON 1e-6
#define VERBOSE 1
