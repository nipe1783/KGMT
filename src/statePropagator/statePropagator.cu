
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
    float a        = UNI_MIN_ACC + curand_uniform(seed) * (UNI_MAX_ACC - UNI_MIN_ACC);
    float steering = UNI_MIN_STEERING + curand_uniform(seed) * (UNI_MAX_STEERING - UNI_MIN_STEERING);
    float duration = UNI_MIN_DT + curand_uniform(seed) * (UNI_MAX_DT - UNI_MIN_DT);
    float dt       = duration / NUM_DISC;

    float x     = x0[0];
    float y     = x0[1];
    float theta = x0[2];
    float v     = x0[3];

    float cosTheta, sinTheta, tanSteering;
    float bbMin[DIM], bbMax[DIM];

    bool motionValid = true;
    for(int i = 0; i < NUM_DISC; i++)
        {
            float x0State[DIM] = {x, y};
            cosTheta           = cos(theta);
            sinTheta           = sin(theta);
            tanSteering        = tan(steering);

            // --- State Propagation ---
            x += v * cosTheta * dt;
            y += v * sinTheta * dt;
            theta += v * tanSteering * dt;
            v += a * dt;
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
                            bbMin[d] = x0State[d];
                            bbMax[d] = x1State[d];
                        }
                    else
                        {
                            bbMin[d] = x1State[d];
                            bbMax[d] = x0State[d];
                        }
                }
            motionValid = isMotionValid(x0State, x1State, bbMin, bbMax, obstacles, obstaclesCount);
            if(!motionValid) break;
        }

    return true;
}

/***************************/
/* DUBINS PROPAGATION FUNCTION */
/***************************/
__device__ bool propagateAndCheckDubins(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount)
{
    printf("/***************************/\n");
    printf("/* DUBINS: TODO */\n");
    printf("/***************************/\n");
    // TODO: Implement Dubins
    return true;
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
                return propagateAndCheckDubins;
            default:
                return nullptr;
        }
}