
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
    float Zc = QUAD_MIN_Zc + curand_uniform(seed) * (QUAD_MAX_Zc - QUAD_MIN_Zc);
    float Lc = QUAD_MIN_Lc + curand_uniform(seed) * (QUAD_MAX_Lc - QUAD_MIN_Lc);
    float Mc = QUAD_MIN_Mc + curand_uniform(seed) * (QUAD_MAX_Mc - QUAD_MIN_Mc);
    float Nc = QUAD_MIN_Nc + curand_uniform(seed) * (QUAD_MAX_Nc - QUAD_MIN_Nc);

    int propagationDuration = 1 + (int)(curand_uniform(seed) * (MAX_PROPAGATION_DURATION));

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

            ode(h1, x0, Zc, Lc, Mc, Nc);
            ode(h2, h1, Zc, Lc, Mc, Nc);
            ode(h3, h2, Zc, Lc, Mc, Nc);
            ode(h4, h3, Zc, Lc, Mc, Nc);

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

            // --- Vehicle Dynamics Check ---
            if(u < QUAD_MIN_VEL || u > QUAD_MAX_VEL || v < QUAD_MIN_VEL || v > QUAD_MAX_VEL || w < QUAD_MIN_VEL || w > QUAD_MAX_VEL)
                {
                    motionValid = false;
                    break;
                }

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
    x1[12] = Zc;
    x1[13] = Lc;
    x1[14] = Mc;
    x1[15] = Nc;
    x1[16] = STEP_SIZE * propagationDuration;

    return motionValid;
}

__device__ void ode(float* x0dot, float* x0, float Zc, float Lc, float Mc, float Nc)
{
    float x, y, z, phi, theta, psi, u, v, w, p, q, r;
    x     = x0[0];
    y     = x0[1];
    z     = x0[2];
    phi   = x0[3];
    theta = x0[4];
    psi   = x0[5];
    u     = x0[6];
    v     = x0[7];
    w     = x0[8];
    p     = x0[9];
    q     = x0[10];
    r     = x0[11];

    x0dot[0] = cos(theta) * cos(psi) * u + (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) * v +
               (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w;

    x0dot[1] = cos(theta) * sin(psi) * u + (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) * v +
               (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w;

    x0dot[2] = -sin(theta) * u + sin(phi) * cos(theta) * v + cos(phi) * cos(theta) * w;

    x0dot[3] = p + (q * sin(phi) + r * cos(phi)) * tan(theta);

    x0dot[4] = q * cos(phi) - r * sin(phi);

    x0dot[5] = (q * sin(phi) + r * cos(phi)) / cos(theta);

    float XYZ = -NU * sqrt(u * u + v * v + w * w);
    float X   = XYZ * u;
    x0dot[6]  = (r * v - q * w) - GRAVITY * sin(theta) + MASS_INV * X;

    float Y  = XYZ * v;
    x0dot[7] = (p * w - r * u) + GRAVITY * cos(theta) * sin(phi) + MASS_INV * Y;

    float Z  = XYZ * w;
    x0dot[8] = (q * u - p * v) + GRAVITY * cos(theta) * cos(phi) + MASS_INV * Z + MASS_INV * Zc;

    float LMN = -MU * sqrt(p * p + q * q + r * r);
    float L   = LMN * p;
    x0dot[9]  = (IY - IZ) / IX * q * r + (1 / IX) * L + (1 / IX) * Lc;

    float M   = LMN * q;
    x0dot[10] = (IZ - IX) / IY * p * r + (1 / IY) * M + (1 / IY) * Mc;

    float N   = LMN * r;
    x0dot[11] = (IX - IY) / IZ * p * q + (1 / IZ) * N + (1 / IZ) * Nc;
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