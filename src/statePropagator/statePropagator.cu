
#include "statePropagator/statePropagator.cuh"

__device__ bool propagateAndCheck(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount)
{
    PropagateAndCheckFunc func = getPropagateAndCheckFunc();
    return func ? func(x0, x1, seed, obstacles, obstaclesCount) : false;
}

/***************************/
/* UNICYCLE PROPAGATION FUNCTION */
/***************************/
__device__ bool propagateAndCheckUnicycle(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount)
{
    float a                 = UNI_MIN_ACC + curand_uniform(seed) * (UNI_MAX_ACC - UNI_MIN_ACC);
    float steering          = UNI_MIN_STEERING + curand_uniform(seed) * (UNI_MAX_STEERING - UNI_MIN_STEERING);
    float duration          = UNI_MIN_DT + curand_uniform(seed) * (UNI_MAX_DT - UNI_MIN_DT);
    int propagationDuration = 1 + (int)(curand_uniform(seed) * (MAX_PROPAGATION_DURATION));

    float x     = x0[0];
    float y     = x0[1];
    float theta = x0[2];
    float v     = x0[3];

    float cosTheta, sinTheta, tanSteering;
    float bbMin[DIM], bbMax[DIM];

    bool motionValid = true;
    for(int i = 0; i < propagationDuration; i++)
        {
            float x0State[DIM] = {x, y};
            cosTheta           = cos(theta);
            sinTheta           = sin(theta);
            tanSteering        = tan(steering);

            // --- State Propagation ---
            x += v * cosTheta * STEP_SIZE;
            y += v * sinTheta * STEP_SIZE;
            theta += (v / UNI_LENGTH) * tanSteering * STEP_SIZE;
            v += a * STEP_SIZE;
            float x1State[DIM] = {x, y};

            // --- Workspace Limit Check ---
            if(x < 0 || x > WS_SIZE || y < 0 || y > WS_SIZE)
                {
                    motionValid = false;
                    break;
                }

            // --- Obstacle Collision Check ---
            for(int d = 0; d < DIM; d++)
                {
                    if(x0State[d] > x1State[d])
                        {
                            bbMin[d] = x1State[d];
                            bbMax[d] = x0State[d];
                        }
                    else
                        {
                            bbMin[d] = x0State[d];
                            bbMax[d] = x1State[d];
                        }
                }

            motionValid = motionValid && isMotionValid(x0State, x1State, bbMin, bbMax, obstacles, obstaclesCount);
            if(!motionValid) break;
        }

    x1[0] = x, x1[1] = y, x1[2] = theta, x1[3] = v, x1[4] = a, x1[5] = steering, x1[6] = duration;
    return motionValid;
}

/***************************/
/* DOUBLE INTEGRATOR PROPAGATION FUNCTION */
/***************************/
__device__ bool propagateAndCheckDoubleIntEular(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount)
{
    float ax                = DI_MIN_ACC + curand_uniform(seed) * (DI_MAX_ACC - DI_MIN_ACC);
    float ay                = DI_MIN_ACC + curand_uniform(seed) * (DI_MAX_ACC - DI_MIN_ACC);
    float az                = DI_MIN_ACC + curand_uniform(seed) * (DI_MAX_ACC - DI_MIN_ACC);
    float duration          = DI_MIN_DT + curand_uniform(seed) * (DI_MAX_DT - DI_MIN_DT);
    int propagationDuration = 1 + (int)(curand_uniform(seed) * (MAX_PROPAGATION_DURATION));

    float x  = x0[0];
    float y  = x0[1];
    float z  = x0[2];
    float vx = x0[3];
    float vy = x0[4];
    float vz = x0[5];

    bool motionValid = true;
    float bbMin[DIM], bbMax[DIM];
    for(int i = 0; i < propagationDuration; i++)
        {
            float x0State[DIM] = {x, y, z};

            // --- State Propagation. Eular Method ---
            x += vx * STEP_SIZE;
            y += vy * STEP_SIZE;
            z += vz * STEP_SIZE;
            vx += ax * STEP_SIZE;
            vy += ay * STEP_SIZE;
            vz += az * STEP_SIZE;
            float x1State[DIM] = {x, y, z};

            // --- Workspace Limit Check ---
            if(x < 0 || x > WS_SIZE || y < 0 || y > WS_SIZE || z < 0 || z > WS_SIZE)
                {
                    motionValid = false;
                    break;
                }

            // --- Obstacle Collision Check ---
            for(int d = 0; d < DIM; d++)
                {
                    if(x0State[d] > x1State[d])
                        {
                            bbMin[d] = x1State[d];
                            bbMax[d] = x0State[d];
                        }
                    else
                        {
                            bbMin[d] = x0State[d];
                            bbMax[d] = x1State[d];
                        }
                }

            motionValid = motionValid && isMotionValid(x0State, x1State, bbMin, bbMax, obstacles, obstaclesCount);
            if(!motionValid) break;
        }

    x1[0] = x, x1[1] = y, x1[2] = z, x1[3] = vx, x1[4] = vy, x1[5] = vz, x1[6] = ax, x1[7] = ay, x1[8] = az, x1[9] = duration;
    return motionValid;
}

/***************************/
/* DOUBLE INTEGRATOR PROPAGATION FUNCTION */
/***************************/
__device__ bool propagateAndCheckDoubleIntRungeKutta(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount)
{
    float ax                = DI_MIN_ACC + curand_uniform(seed) * (DI_MAX_ACC - DI_MIN_ACC);
    float ay                = DI_MIN_ACC + curand_uniform(seed) * (DI_MAX_ACC - DI_MIN_ACC);
    float az                = DI_MIN_ACC + curand_uniform(seed) * (DI_MAX_ACC - DI_MIN_ACC);
    int propagationDuration = 1 + (int)(curand_uniform(seed) * (MAX_PROPAGATION_DURATION));

    float x  = x0[0];
    float y  = x0[1];
    float z  = x0[2];
    float vx = x0[3];
    float vy = x0[4];
    float vz = x0[5];

    bool motionValid = true;
    float bbMin[DIM], bbMax[DIM];
    for(int i = 0; i < propagationDuration; i++)
        {
            float x0State[DIM] = {x, y, z};

            // --- State Propagation. 4th order Runge Kutta ---
            x += (vx + (vx + 2 * (vx + ax * STEP_SIZE / 2) + (vx + ax * STEP_SIZE))) * STEP_SIZE / 6;
            y += (vy + (vy + 2 * (vy + ay * STEP_SIZE / 2) + (vy + ay * STEP_SIZE))) * STEP_SIZE / 6;
            z += (vz + (vz + 2 * (vz + az * STEP_SIZE / 2) + (vz + az * STEP_SIZE))) * STEP_SIZE / 6;
            vx += (ax + 2 * ax + 2 * ax + ax) * STEP_SIZE / 6;
            vy += (ay + 2 * ay + 2 * ay + ay) * STEP_SIZE / 6;
            vz += (az + 2 * az + 2 * az + az) * STEP_SIZE / 6;

            float x1State[DIM] = {x, y, z};

            // --- Workspace Limit Check ---
            if(x < 0 || x > WS_SIZE || y < 0 || y > WS_SIZE || z < 0 || z > WS_SIZE)
                {
                    motionValid = false;
                    break;
                }

            // --- Obstacle Collision Check ---
            for(int d = 0; d < DIM; d++)
                {
                    if(x0State[d] > x1State[d])
                        {
                            bbMin[d] = x1State[d];
                            bbMax[d] = x0State[d];
                        }
                    else
                        {
                            bbMin[d] = x0State[d];
                            bbMax[d] = x1State[d];
                        }
                }

            motionValid = motionValid && isMotionValid(x0State, x1State, bbMin, bbMax, obstacles, obstaclesCount);
            if(!motionValid) break;
        }

    x1[0] = x, x1[1] = y, x1[2] = z, x1[3] = vx, x1[4] = vy, x1[5] = vz, x1[6] = ax, x1[7] = ay, x1[8] = az,
    x1[9] = STEP_SIZE * propagationDuration;
    return motionValid;
}

/***************************/
/* GET PROPAGATION FUNCTION */
/***************************/
__device__ PropagateAndCheckFunc getPropagateAndCheckFunc()
{
    switch(MODEL)
        {
            case 0:
                return propagateAndCheckUnicycle;
            case 1:
                return propagateAndCheckDoubleIntRungeKutta;
            default:
                return nullptr;
        }
}