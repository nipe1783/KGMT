#pragma once

#define DIM 2
#define R1 4
#define R2 4
#define SAMPLE_DIM 7
#define NUM_DISC 10
#define WS_SIZE 10.0
#define MODEL 0
#define MAX_TREE_SIZE 30000
#define GOAL_THRESH 0.1
#define MAX_ITER 1

// --- UNICYCLE MODEL: MODEL 0 ---
#define UNI_MIN_ACC -1.0
#define UNI_MAX_ACC 1.0
#define UNI_MIN_STEERING -M_PI / 2
#define UNI_MAX_STEERING M_PI / 2
#define UNI_MIN_DT .1
#define UNI_MAX_DT 2.0

// --- DUBINS MODEL: MODEL 1 ---
// TODO: Implement Dubins model

#define NUM_R1_VERTICES ((DIM == 3) ? (R1 * R1 * R1) : (R1 * R1))
#define NUM_R2_VERTICES ((DIM == 3) ? (R1 * R1 * R1 * R2 * R2 * R2) : (R1 * R1 * R2 * R2))
#define R2_PER_R1 ((DIM == 3) ? (R2 * R2 * R2) : (R2 * R2))
#define EPSILON 1e-6
#define VERBOSE 1
