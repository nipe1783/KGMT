
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

            // --- Dyanmics Validity Check ---
            if(vx < DI_MIN_VEL || vx > DI_MAX_VEL || vy < DI_MIN_VEL || vy > DI_MAX_VEL || vz < DI_MIN_VEL || vz > DI_MAX_VEL)
                {
                    motionValid = false;
                    break;
                }

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
/* DUBINS AIRPLANE PROPAGATION FUNCTION */
/***************************/
__device__ bool propagateAndCheckDubinsAirplaneRungeKutta(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount)
{
    float a                 = DUBINS_AIRPLANE_MIN_ACC + curand_uniform(seed) * (DUBINS_AIRPLANE_MAX_ACC - DUBINS_AIRPLANE_MIN_ACC);
    float yawRate           = DUBINS_AIRPLANE_MIN_YR + curand_uniform(seed) * (DUBINS_AIRPLANE_MAX_YR - DUBINS_AIRPLANE_MIN_YR);
    float pitchRate         = DUBINS_AIRPLANE_MIN_PR + curand_uniform(seed) * (DUBINS_AIRPLANE_MAX_PR - DUBINS_AIRPLANE_MIN_PR);
    int propagationDuration = 1 + (int)(curand_uniform(seed) * (MAX_PROPAGATION_DURATION));

    float x     = x0[0];
    float y     = x0[1];
    float z     = x0[2];
    float yaw   = x0[3];
    float pitch = x0[4];
    float v     = x0[5];

    bool motionValid = true;
    float bbMin[DIM], bbMax[DIM];

    for(int i = 0; i < propagationDuration; i++)
        {
            float x0State[DIM] = {x, y, z};

            // --- State Propagation using 4th Order Runge-Kutta Method ---
            x +=
              (STEP_SIZE / 6.0f) *
              (v * cosf(pitch) * cosf(yaw) +
               2.0f * ((v + 0.5f * STEP_SIZE * a) * cosf(pitch + 0.5f * STEP_SIZE * pitchRate) * cosf(yaw + 0.5f * STEP_SIZE * yawRate) +
                       (v + 0.5f * STEP_SIZE * a) * cosf(pitch + 0.5f * STEP_SIZE * pitchRate) * cosf(yaw + 0.5f * STEP_SIZE * yawRate)) +
               (v + STEP_SIZE * a) * cosf(pitch + STEP_SIZE * pitchRate) * cosf(yaw + STEP_SIZE * yawRate));
            y +=
              (STEP_SIZE / 6.0f) *
              (v * cosf(pitch) * sinf(yaw) +
               2.0f * ((v + 0.5f * STEP_SIZE * a) * cosf(pitch + 0.5f * STEP_SIZE * pitchRate) * sinf(yaw + 0.5f * STEP_SIZE * yawRate) +
                       (v + 0.5f * STEP_SIZE * a) * cosf(pitch + 0.5f * STEP_SIZE * pitchRate) * sinf(yaw + 0.5f * STEP_SIZE * yawRate)) +
               (v + STEP_SIZE * a) * cosf(pitch + STEP_SIZE * pitchRate) * sinf(yaw + STEP_SIZE * yawRate));
            z += (STEP_SIZE / 6.0f) * (v * sinf(pitch) +
                                       2.0f * ((v + 0.5f * STEP_SIZE * a) * sinf(pitch + 0.5f * STEP_SIZE * pitchRate) +
                                               (v + 0.5f * STEP_SIZE * a) * sinf(pitch + 0.5f * STEP_SIZE * pitchRate)) +
                                       (v + STEP_SIZE * a) * sinf(pitch + STEP_SIZE * pitchRate));
            yaw += STEP_SIZE * yawRate;
            pitch += STEP_SIZE * pitchRate;
            v += (STEP_SIZE / 6.0f) * (a + 2.0f * (a + a) + a);

            // --- Dynamics Validity Check ---'
            if(v < DUBINS_AIRPLANE_MIN_VEL || v > DUBINS_AIRPLANE_MAX_VEL)
                {
                    motionValid = false;
                    break;
                }
            else if(pitch < DUBINS_AIRPLANE_MIN_PITCH || pitch > DUBINS_AIRPLANE_MAX_PITCH)
                {
                    motionValid = false;
                    break;
                }

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

    x1[0] = x;
    x1[1] = y;
    x1[2] = z;
    x1[3] = yaw;
    x1[4] = pitch;
    x1[5] = v;
    x1[6] = yawRate;
    x1[7] = pitchRate;
    x1[8] = a;
    x1[9] = STEP_SIZE * propagationDuration;

    return motionValid;
}

/***************************/
/* QUAD COPTER PROPAGATION FUNCTION */
/***************************/
__device__ bool propagateAndCheckQuadRungeKutta(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount)
{
    float f1 = QUAD_MIN_T + curand_uniform(seed) * (QUAD_MAX_T - QUAD_MIN_T);
    float f2 = QUAD_MIN_T + curand_uniform(seed) * (QUAD_MAX_T - QUAD_MIN_T);
    float f3 = QUAD_MIN_T + curand_uniform(seed) * (QUAD_MAX_T - QUAD_MIN_T);
    float f4 = QUAD_MIN_T + curand_uniform(seed) * (QUAD_MAX_T - QUAD_MIN_T);
    f1       = .1;
    f2       = .4;
    f3       = .4;
    f4       = .1;

    int propagationDuration = 1 + (int)(curand_uniform(seed) * (MAX_PROPAGATION_DURATION));
    propagationDuration     = 1;

    float x     = x0[0];
    float y     = x0[1];
    float z     = x0[2];
    float phi   = x0[3];
    float theta = x0[4];
    float psi   = x0[5];
    float u     = x0[6];
    float v     = x0[7];
    float w     = x0[8];
    float p     = x0[9];
    float q     = x0[10];
    float r     = x0[11];

    bool motionValid = true;
    float bbMin[W_DIM], bbMax[W_DIM];

    float* h1 = new float[12];
    float* h2 = new float[12];
    float* h3 = new float[12];
    float* h4 = new float[12];

    for(int i = 0; i < propagationDuration; i++)
        {
            float x0State[W_DIM] = {x, y, z};

            fQuad(h1, x0, f1, f2, f3, f4);
            fQuad(h2, h1, f1, f2, f3, f4);
            fQuad(h3, h2, f1, f2, f3, f4);
            fQuad(h4, h3, f1, f2, f3, f4);

            x += STEP_SIZE / 6 * (h1[0] + 2.0f * h2[0] + 2.0f * h3[0] + h4[0]);
            y += STEP_SIZE / 6 * (h1[1] + 2.0f * h2[1] + 2.0f * h3[1] + h4[1]);
            z += STEP_SIZE / 6 * (h1[2] + 2.0f * h2[2] + 2.0f * h3[2] + h4[2]);
            phi += STEP_SIZE / 6 * (h1[3] + 2.0f * h2[3] + 2.0f * h3[3] + h4[3]);
            theta += STEP_SIZE / 6 * (h1[4] + 2.0f * h2[4] + 2.0f * h3[4] + h4[4]);
            psi += STEP_SIZE / 6 * (h1[5] + 2.0f * h2[5] + 2.0f * h3[5] + h4[5]);
            u += STEP_SIZE / 6 * (h1[6] + 2.0f * h2[6] + 2.0f * h3[6] + h4[6]);
            v += STEP_SIZE / 6 * (h1[7] + 2.0f * h2[7] + 2.0f * h3[7] + h4[7]);
            w += STEP_SIZE / 6 * (h1[8] + 2.0f * h2[8] + 2.0f * h3[8] + h4[8]);
            p += STEP_SIZE / 6 * (h1[9] + 2.0f * h2[9] + 2.0f * h3[9] + h4[9]);
            q += STEP_SIZE / 6 * (h1[10] + 2.0f * h2[10] + 2.0f * h3[10] + h4[10]);
            r += STEP_SIZE / 6 * (h1[11] + 2.0f * h2[11] + 2.0f * h3[11] + h4[11]);

            float x1State[W_DIM] = {x, y, z};

            // --- Workspace Limit Check ---
            if(x < 0 || x > WS_SIZE || y < 0 || y > WS_SIZE || z < 0 || z > WS_SIZE)
                {
                    motionValid = false;
                    break;
                }

            // --- Obstacle Collision Check ---
            for(int d = 0; d < W_DIM; d++)
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

    delete h1;
    delete h2;
    delete h3;
    delete h4;

    x1[0]  = x;
    x1[1]  = y;
    x1[2]  = z;
    x1[3]  = phi;
    x1[4]  = theta;
    x1[5]  = psi;
    x1[6]  = u;
    x1[7]  = v;
    x1[8]  = w;
    x1[9]  = p;
    x1[10] = q;
    x1[11] = r;
    x1[12] = f1;
    x1[13] = f2;
    x1[14] = f3;
    x1[15] = f4;
    x1[16] = STEP_SIZE * propagationDuration;
    printf("x: %f, y: %f, z: %f, phi: %f, theta: %f, psi: %f, u: %f, v: %f, w: %f, p: %f, q: %f, r: %f\n", x, y, z, phi, theta, psi, u, v,
           w, p, q, r);

    return motionValid;
}

__device__ float secf(float angle)
{
    return 1.0f / cosf(angle);
}

__device__ void fQuad(float* h, float* x0, float f1, float f2, float f3, float f4)
{
    // Extract values from x0
    float x     = x0[0];
    float y     = x0[1];
    float z     = x0[2];
    float phi   = x0[3];
    float theta = x0[4];
    float psi   = x0[5];
    float u     = x0[6];
    float v     = x0[7];
    float w     = x0[8];
    float p     = x0[9];
    float q     = x0[10];
    float r     = x0[11];

    // Compute h(1:3)
    h[0] = cosf(theta) * cosf(psi) * u + (sinf(phi) * sinf(theta) * cosf(psi) - cosf(phi) * sinf(psi)) * v +
           (cosf(phi) * sinf(theta) * cosf(psi) + sinf(phi) * sinf(psi)) * w;
    h[1] = cosf(theta) * sinf(psi) * u + (sinf(phi) * sinf(theta) * sinf(psi) + cosf(phi) * cosf(psi)) * v +
           (cosf(phi) * sinf(theta) * sinf(psi) - sinf(phi) * cosf(psi)) * w;
    h[2] = -sinf(theta) * u + sinf(phi) * cosf(theta) * v + cosf(phi) * cosf(theta) * w;

    // Compute h(4:6)
    h[3] = p + sinf(phi) * tanf(theta) * q + cosf(phi) * tanf(theta) * r;
    h[4] = cosf(phi) * q - sinf(phi) * r;
    h[5] = sinf(phi) * secf(theta) * q + cosf(phi) * secf(theta) * r;

    // Compute XYZ
    float XYZ[3];
    float sqrt_uvwpow = sqrtf(u * u + v * v + w * w);
    XYZ[0]            = -NU * sqrt_uvwpow * u;
    XYZ[1]            = -NU * sqrt_uvwpow * v;
    XYZ[2]            = -NU * sqrt_uvwpow * w;

    // Compute Z_c
    float Z_c = f1 + f2 + f3 + f4;

    // Compute h(7:9)
    h[6] = r * v - q * w + GRAVITY * (-sinf(theta)) + (1 / MASS) * (XYZ[0]) + 0;
    h[7] = p * w - r * u + GRAVITY * (cosf(theta) * sinf(phi)) + (1 / MASS) * (XYZ[1]) + 0;
    h[8] = q * u - p * v + GRAVITY * (cosf(theta) * cosf(phi)) + (1 / MASS) * (XYZ[2]) + (1 / MASS) * Z_c;

    // Compute LMN
    float LMN[3];
    float sqrt_pqrpow = sqrtf(p * p + q * q + r * r);
    LMN[0]            = -MU * sqrt_pqrpow * p;
    LMN[1]            = -MU * sqrt_pqrpow * q;
    LMN[2]            = -MU * sqrt_pqrpow * r;

    // Compute LMN_c
    float LMN_c[3];
    LMN_c[0] = -QUAD_ARM_LENGTH / sqrtf(2) * (-f1 - f2 + f3 + f4);
    LMN_c[1] = -QUAD_ARM_LENGTH / sqrtf(2) * (f1 - f2 - f3 + f4);
    LMN_c[2] = -KM * (f1 - f2 + f3 - f4);

    // Compute h(10:12)
    h[9]  = ((IX - IZ) / IX) * q * r + (1 / IX) * LMN[0] + (1 / IX) * LMN_c[0];
    h[10] = ((IZ - IX) / IY) * p * r + (1 / IY) * LMN[1] + (1 / IY) * LMN_c[1];
    h[11] = ((IX - IY) / IZ) * p * q + (1 / IZ) * LMN[2] + (1 / IZ) * LMN_c[2];
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
            case 2:
                return propagateAndCheckDubinsAirplaneRungeKutta;
            case 3:
                return propagateAndCheckQuadRungeKutta;
            default:
                return nullptr;
        }
}