#pragma once

// #define DIM 3
// #define R1 8
// #define R2 4
// #define SAMPLE_DIM 10
// #define NUM_DISC 10
// #define WS_SIZE 1.1f
// #define MODEL 1
// #define MAX_TREE_SIZE 50000
// #define GOAL_THRESH 0.05f
// #define MAX_ITER 10

// // --- UNICYCLE MODEL: MODEL 0 ---
// #define UNI_MIN_ACC -1.0f
// #define UNI_MAX_ACC 1.0f
// #define UNI_MIN_STEERING -M_PI / 2
// #define UNI_MAX_STEERING M_PI / 2
// #define UNI_MIN_DT 0.1f
// #define UNI_MAX_DT 2.0f
// #define UNI_LENGTH 1.0f

// // --- DOUBLE INTEGRATOR: MODEL 1 ---
// #define DI_MIN_VEL -0.3f
// #define DI_MAX_VEL 0.3f
// #define DI_MIN_ACC -.2f
// #define DI_MAX_ACC .2f
// #define DI_MIN_DT 0.1f
// #define DI_MAX_DT 2.0f

// #define NUM_R1_VERTICES ((DIM == 3) ? (R1 * R1 * R1) : (R1 * R1))
// #define NUM_R2_VERTICES ((DIM == 3) ? (R1 * R1 * R1 * R2 * R2 * R2) : (R1 * R1 * R2 * R2))
// #define R2_PER_R1 ((DIM == 3) ? (R2 * R2 * R2) : (R2 * R2))
// #define R1_SIZE (WS_SIZE / R1)
// #define R2_SIZE (WS_SIZE / (R1 * R2))
// #define EPSILON 1e-6f
// #define VERBOSE 1

// --- TREES CONFIG ---
// #define DIM 3
// #define R1 8
// #define R2 4
// #define SAMPLE_DIM 10
// #define NUM_DISC 10
// #define WS_SIZE 1.0f
// #define MODEL 1
// #define MAX_TREE_SIZE 50000
// #define GOAL_THRESH 0.05f
// #define MAX_ITER 100

// // --- UNICYCLE MODEL: MODEL 0 ---
// #define UNI_MIN_ACC -1.0f
// #define UNI_MAX_ACC 1.0f
// #define UNI_MIN_STEERING -M_PI / 2
// #define UNI_MAX_STEERING M_PI / 2
// #define UNI_MIN_DT 0.1f
// #define UNI_MAX_DT 2.0f
// #define UNI_LENGTH 1.0f

// // --- DOUBLE INTEGRATOR: MODEL 1 ---
// #define DI_MIN_VEL -0.3f
// #define DI_MAX_VEL 0.3f
// #define DI_MIN_ACC -.2f
// #define DI_MAX_ACC .2f
// #define DI_MIN_DT 0.1f
// #define DI_MAX_DT 2.0f

// #define NUM_R1_VERTICES ((DIM == 3) ? (R1 * R1 * R1) : (R1 * R1))
// #define NUM_R2_VERTICES ((DIM == 3) ? (R1 * R1 * R1 * R2 * R2 * R2) : (R1 * R1 * R2 * R2))
// #define R2_PER_R1 ((DIM == 3) ? (R2 * R2 * R2) : (R2 * R2))
// #define R1_SIZE (WS_SIZE / R1)
// #define R2_SIZE (WS_SIZE / (R1 * R2))
// #define EPSILON 1e-6f
// #define VERBOSE 1

// --- PILLARS CONFIG ---
#define DIM 3
#define R1 8
#define R2 4
#define SAMPLE_DIM 10
#define NUM_DISC 10
#define WS_SIZE 1.0f
#define MODEL 1
#define MAX_TREE_SIZE 20000
#define GOAL_THRESH 0.05f
#define MAX_ITER 100

// --- UNICYCLE MODEL: MODEL 0 ---
#define UNI_MIN_ACC -1.0f
#define UNI_MAX_ACC 1.0f
#define UNI_MIN_STEERING -M_PI / 2
#define UNI_MAX_STEERING M_PI / 2
#define UNI_MIN_DT 0.1f
#define UNI_MAX_DT 2.0f
#define UNI_LENGTH 1.0f

// --- DOUBLE INTEGRATOR: MODEL 1 ---
#define DI_MIN_VEL -0.3f
#define DI_MAX_VEL 0.3f
#define DI_MIN_ACC -.2f
#define DI_MAX_ACC .2f
#define DI_MIN_DT 0.1f
#define DI_MAX_DT 2.0f

#define NUM_R1_VERTICES ((DIM == 3) ? (R1 * R1 * R1) : (R1 * R1))
#define NUM_R2_VERTICES ((DIM == 3) ? (R1 * R1 * R1 * R2 * R2 * R2) : (R1 * R1 * R2 * R2))
#define R2_PER_R1 ((DIM == 3) ? (R2 * R2 * R2) : (R2 * R2))
#define R1_SIZE (WS_SIZE / R1)
#define R2_SIZE (WS_SIZE / (R1 * R2))
#define EPSILON 1e-6f
#define VERBOSE 1

// float h_initial[SAMPLE_DIM] = {0.3, 0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
//           h_goal[SAMPLE_DIM]    = {.7, .95, .9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};