#pragma once

/***************************/
/* DUBINS AIRPLANE CONFIG  */
/***************************/
// --- PILLARS CONFIG ---
// #define W_DIM 3
// #define R1 8
// #define R2 4
// #define SAMPLE_DIM 10
// #define W_SIZE 1.0f
// #define MODEL 2
// #define MAX_TREE_SIZE 200000
// #define GOAL_THRESH 0.05f
// #define MAX_ITER 300
// #define STEP_SIZE 0.1f
// #define MAX_PROPAGATION_DURATION 10

// #define W_DIM 3
// #define C_DIM 2
// #define V_DIM 1

// #define W_MIN 0.0f
// #define W_MAX 1.0f

// #define C_MIN -M_PI
// #define C_MAX M_PI

// #define V_MIN 0.0f
// #define V_MAX 0.3f

// #define W_R1_LENGTH 8
// #define C_R1_LENGTH 8
// #define V_R1_LENGTH 1

// #define W_R2_LENGTH 2
// #define C_R2_LENGTH 3
// #define V_R2_LENGTH 1

// #define W_R1_SIZE ((W_MAX - W_MIN) / W_R1_LENGTH)
// #define C_R1_SIZE ((C_MAX - C_MIN) / C_R1_LENGTH)
// #define V_R1_SIZE ((V_MAX - V_MIN) / V_R1_LENGTH)

// #define NUM_R2_PER_R1 W_R2_LENGTH *W_R2_LENGTH *W_R2_LENGTH *C_R2_LENGTH *C_R2_LENGTH *V_R2_LENGTH

// // --- UNICYCLE MODEL: MODEL 0 ---
// #define A_MIN -1.0f
// #define A_MAX 1.0f
// #define UNI_MIN_STEERING -M_PI / 2
// #define UNI_MAX_STEERING M_PI / 2
// #define UNI_MIN_DT 0.1f
// #define UNI_MAX_DT 2.0f
// #define UNI_LENGTH 1.0f

// // --- DOUBLE INTEGRATOR: MODEL 1 ---
// #define STATE_DIM 6
// #define V_MIN -0.3f
// #define V_MAX 0.3f
// #define A_MIN -.2f
// #define A_MAX .2f
// #define DI_MIN_DT 0.1f  // TODO: remove these
// #define DI_MAX_DT 2.0f

// // --- DUBINS AIRPLANE: MODEL 2 ---
// #define STATE_DIM 6
// #define A_MIN -0.3f
// #define A_MAX 0.3f
// #define DUBINS_AIRPLANE_MIN_PR (-M_PI / 4)
// #define DUBINS_AIRPLANE_MAX_PR (M_PI / 4)
// #define DUBINS_AIRPLANE_MIN_YR (-M_PI / 4)
// #define DUBINS_AIRPLANE_MAX_YR (M_PI / 4)
// #define V_MIN 0.0f
// #define V_MAX0.3f
// #define DUBINS_AIRPLANE_MIN_YAW -M_PI
// #define DUBINS_AIRPLANE_MAX_YAW M_PI
// #define DUBINS_AIRPLANE_MIN_PITCH -M_PI / 3
// #define DUBINS_AIRPLANE_MAX_PITCH M_PI / 3

// #define NUM_PARTIAL_SUMS 1024  // NUM_R1_REGIONS / 32 only used when NUM_R1_REGIONS > 1024
// #define NUM_R1_REGIONS_KERNEL1 \
//     1024  // NUM_R1_REGIONS used inside kernel1. must be a constant so the code does not break when NUM_R1_REGIONS > 1024.
// #define NUM_R1_REGIONS                                                                                \
//     ((W_DIM == 3) ? (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH * V_R1_LENGTH) \
//                 : (W_R1_LENGTH * W_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH * V_R1_LENGTH * V_R1_LENGTH))

// #define NUM_R2_REGIONS                                                                                                             \
//     ((W_DIM == 3) ? (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH * V_R1_LENGTH * W_R2_LENGTH * W_R2_LENGTH * \
//                    W_R2_LENGTH * C_R2_LENGTH * C_R2_LENGTH * V_R2_LENGTH)                                                          \
//                 : (W_R1_LENGTH * W_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH * V_R1_LENGTH * V_R1_LENGTH * W_R2_LENGTH * W_R2_LENGTH * \
//                    C_R2_LENGTH * C_R2_LENGTH * V_R2_LENGTH * V_R2_LENGTH))

// #define R2_PER_R1 ((W_DIM == 3) ? (R2 * R2 * R2) : (R2 * R2))
// #define R1_SIZE (W_SIZE / R1)
// #define R2_SIZE (W_SIZE / (R1 * R2))
// #define EPSILON 1e-2f
// #define VERBOSE 1

/***************************/
/* 6D DOUBLE INTEGRATOR CONFIG  */
/***************************/

// #define MODEL 1

// #define MAX_TREE_SIZE 200000
// #define MAX_ITER 300
// #define STEP_SIZE 0.1f
// #define MAX_PROPAGATION_DURATION 10

// #define MODEL 1
// #define MAX_TREE_SIZE 200000
// #define GOAL_THRESH 0.05f

// #define STATE_DIM 6
// #define CONTROL_DIM 3
// #define SAMPLE_DIM (STATE_DIM + CONTROL_DIM + 1)

// #define W_DIM 3
// #define C_DIM 1
// #define V_DIM 3

// #define W_MIN 0.0f
// #define W_MAX 1.0f
// #define W_SIZE 1.0f

// #define C_MIN -M_PI
// #define C_MAX M_PI

// #define V_MIN -0.3f
// #define V_MAX 0.3f

// #define A_MIN -0.2f
// #define A_MAX 0.2f

// #define W_R1_LENGTH 8
// #define C_R1_LENGTH 1
// #define V_R1_LENGTH 2

// #define W_R2_LENGTH 2
// #define C_R2_LENGTH 1
// #define V_R2_LENGTH 2

// #define W_R1_SIZE ((W_MAX - W_MIN) / W_R1_LENGTH)
// #define C_R1_SIZE ((C_MAX - C_MIN) / C_R1_LENGTH)
// #define V_R1_SIZE ((V_MAX - V_MIN) / V_R1_LENGTH)

// #define W_R1_VOL (W_R1_SIZE * W_R1_SIZE * W_R1_SIZE)

// #define NUM_R1_REGIONS \
//     (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH * V_R1_LENGTH * V_R1_LENGTH * V_R1_LENGTH)
// #define NUM_R2_REGIONS                                                                                                                \
//     (NUM_R1_REGIONS * W_R2_LENGTH * W_R2_LENGTH * W_R2_LENGTH * V_R2_LENGTH * V_R2_LENGTH * \
//      V_R2_LENGTH)
// #define NUM_R2_PER_R1 W_R2_LENGTH *W_R2_LENGTH *W_R2_LENGTH *V_R2_LENGTH *V_R2_LENGTH *V_R2_LENGTH
// #define NUM_R1_REGIONS_KERNEL1 1024
// #define NUM_PARTIAL_SUMS 1024

// #define EPSILON 1e-2f
// #define VERBOSE 1

// // --- UNICYCLE MODEL: MODEL 0 ---
// #define UNI_MIN_STEERING -M_PI / 2
// #define UNI_MAX_STEERING M_PI / 2
// #define UNI_MIN_DT 0.1f
// #define UNI_MAX_DT 2.0f
// #define UNI_LENGTH 1.0f

// // --- DUBINS AIRPLANE: MODEL 2 ---
// #define DUBINS_AIRPLANE_MIN_PR (-M_PI / 4)
// #define DUBINS_AIRPLANE_MAX_PR (M_PI / 4)
// #define DUBINS_AIRPLANE_MIN_YR (-M_PI / 4)
// #define DUBINS_AIRPLANE_MAX_YR (M_PI / 4)
// #define DUBINS_AIRPLANE_MIN_YAW -M_PI
// #define DUBINS_AIRPLANE_MAX_YAW M_PI
// #define DUBINS_AIRPLANE_MIN_PITCH -M_PI / 3
// #define DUBINS_AIRPLANE_MAX_PITCH M_PI / 3

/***************************/
/* DUBINS AIRPLANE CONFIG  */
/***************************/

#define MODEL 2

#define MAX_TREE_SIZE 200000
#define MAX_ITER 300
#define STEP_SIZE 0.1f
#define MAX_PROPAGATION_DURATION 10

#define GOAL_THRESH 0.05f

#define STATE_DIM 6
#define CONTROL_DIM 3
#define SAMPLE_DIM (STATE_DIM + CONTROL_DIM + 1)

#define W_DIM 3
#define C_DIM 2
#define V_DIM 1

#define W_MIN 0.0f
#define W_MAX 1.0f
#define W_SIZE 1.0f

#define C_MIN -M_PI
#define C_MAX M_PI

#define V_MIN 0.0f
#define V_MAX 0.3f

#define A_MIN -0.3f
#define A_MAX 0.3f

#define W_R1_LENGTH 8
#define C_R1_LENGTH 8
#define V_R1_LENGTH 1

#define W_R2_LENGTH 2
#define C_R2_LENGTH 3
#define V_R2_LENGTH 1

#define W_R1_SIZE ((W_MAX - W_MIN) / W_R1_LENGTH)
#define C_R1_SIZE ((C_MAX - C_MIN) / C_R1_LENGTH)
#define V_R1_SIZE ((V_MAX - V_MIN) / V_R1_LENGTH)

#define W_R1_VOL (W_R1_SIZE * W_R1_SIZE * W_R1_SIZE)

#define NUM_R1_REGIONS (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH * V_R1_LENGTH)
#define NUM_R2_REGIONS (NUM_R1_REGIONS * W_R2_LENGTH * W_R2_LENGTH * W_R2_LENGTH * C_R2_LENGTH * C_R2_LENGTH * V_R2_LENGTH)
#define NUM_R2_PER_R1 W_R2_LENGTH *W_R2_LENGTH *W_R2_LENGTH *C_R2_LENGTH *C_R2_LENGTH *V_R2_LENGTH
#define NUM_R1_REGIONS_KERNEL1 1024
#define NUM_PARTIAL_SUMS 1024

#define EPSILON 1e-2f
#define VERBOSE 1

// --- UNICYCLE MODEL: MODEL 0 ---
#define UNI_MIN_STEERING -M_PI / 2
#define UNI_MAX_STEERING M_PI / 2
#define UNI_MIN_DT 0.1f
#define UNI_MAX_DT 2.0f
#define UNI_LENGTH 1.0f

// --- DUBINS AIRPLANE: MODEL 2 ---
#define DUBINS_AIRPLANE_MIN_PR (-M_PI / 4)
#define DUBINS_AIRPLANE_MAX_PR (M_PI / 4)
#define DUBINS_AIRPLANE_MIN_YR (-M_PI / 4)
#define DUBINS_AIRPLANE_MAX_YR (M_PI / 4)
#define DUBINS_AIRPLANE_MIN_YAW -M_PI
#define DUBINS_AIRPLANE_MAX_YAW M_PI
#define DUBINS_AIRPLANE_MIN_PITCH -M_PI / 3
#define DUBINS_AIRPLANE_MAX_PITCH M_PI / 3
