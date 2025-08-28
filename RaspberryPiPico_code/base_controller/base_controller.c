#include <stdio.h>
#include "pico/stdlib.h"
#include "tusb.h"

#define PI 3.14159265359
#define DEG_TO_RAD PI / 180.0

absolute_time_t time;

// Kalman Filter Matricies ------------------------------------------------------------------------
//  For indexing matricies they will be sequentially indexed from 0 to n going
//      left to right, then top down

float xhat[2] = {
    0.0,
    0.0
};
float z[2] = {
    0.0,
    0.0
};
float A[4] = {
    1.0, 0.0,
    0.0, 1.0
};
float A_transpose[4];
float B[2] = {
    0.0,
    0.0
};

float P[4] = {
    0.0, 0.0,
    0.0, 0.0
};
float P_temp[4];
float C[4] = {
    1.0, 0.0,
    0.0, 1.0
};
float C_transpose[4];
float Q[4] = {
    0.1, 0.0,
    0.0, 0.01
};
float G[4];
float G_temp[4];
const float gps_variance = (0.5 * DEG_TO_RAD) * (0.5 * DEG_TO_RAD);
const float gyro_variance = (0.07 * DEG_TO_RAD) * (0.07 * DEG_TO_RAD);
float R[4] = {
    gps_variance, 0.0,
    0.0, gyro_variance
};


// Linear Algebra Funcs ---------------------------------------------------------------------------
void matmul2x2(float out[4], float m1[4], float m2[4]) {
    // matrix multiplication to multiply two 2x2s
    out[0] = m1[0] * m2[0] + m1[1] * m2[2];
    out[1] = m1[0] * m2[1] + m1[1] * m2[3];
    out[2] = m1[2] * m2[0] + m1[3] * m2[2];
    out[3] = m1[2] * m2[1] + m1[3] * m2[3];
}

void matmul2x1(float out[2], float m1[4], float m2[2]) {
    // matrix multiplication to multiply 2x2 x 2x1
    out[0] = m1[0] * m2[0] + m1[1] * m2[1];
    out[1] = m1[2] * m2[0] + m1[3] * m2[1];
}

void transpose2x2(float out[4], float matrix[4]) {
    out[0] = matrix[0];
    out[1] = matrix[2];
    out[2] = matrix[1];
    out[3] = matrix[3];
}

void add2x2(float m1[4], float m2[4]) {
    // matrix addition between two 2x2s
    for (int i = 0; i < 4; i++) {
        m1[i] = m1[i] + m2[i];
    }
}

void subtract2x2(float m1[4], float m2[4]) {
    // matrix addition between two 2x2s
    for (int i = 0; i < 4; i++) {
        m1[i] = m1[i] - m2[i];
    }
}

void invert2x2(float matrix[4]) {
    float det = matrix[0] * matrix[3] - matrix[2] * matrix[1];
    if (det == 0) { return; }
    
    matrix[0] = matrix[3] / det;
    matrix[1] = -matrix[1] / det;
    matrix[2] = -matrix[2] / det;
    matrix[3] = matrix[0] / det;
}

// ------------------------------------------------------------------------------------------------

void kalmanPredict(float u, float dt) {
    // xhat = A * xhat + B * u
    A[1] = dt;
    B[0] = -dt;
    matmul2x1(xhat, A, xhat);
    xhat[0] = xhat[0] + B[0] * u; 

    // P = A * P * A^T + Q
    matmul2x2(P, A, P);
    transpose2x2(A_transpose, A);
    matmul(P, P, A_transpose);
    add2x2(P, Q);
}

void kalmanUpdate(){
    float I[4] = {
        1.0, 0.0,
        0.0, 1.0
    };
    
    // G = P * C^T * ( C * P * C^T + R)^-1
    transpose2x2(C_transpose, C);
    matmul(G, P, C_transpose);
    matmul(G_temp, C, P);
    matmul(G_temp, G_temp, C_transpose);
    add2x2(G_temp, R);
    invert2x2(G_temp);
    matmul(G, G, G_temp);

    // xhat = xhat + G * (z - C * xhat)
    matmul2x1(xhat, C, xhat);
    z[0] = z[0] - xhat[0];
    z[1] = z[1] - xhat[1];
    matmul2x1(z, G, z);
    xhat[0] = xhat[0] + z[0];
    xhat[1] = xhat[1] + z[1];

    // P = (I - G * C) * P
    subtract2x2(I, G);
    matmul2x2(P_temp, I, C);
    matmul2x2(P, P_temp, P);
}

// Controller Vars --------------------------------------------------------------------------------
const float Vforward = 1.0;//0.95;
const float KaP = 0.05; // heading proportional gain
const float KaD = 0.005; // heading derivative gain
float u_r;
float u_l;
float last_alpha;
float alpha;
float Tp;
float Td;
float T;
float u_r_internal;
float u_l_internal;
float target_heading;
static uint64_t last_contr_call;
float minDC = 0.4;
float maxDC = 1.0;
float dt, de;
// Controller Funcs -------------------------------------------------------------------------------
float wrapTo2Pi(float a) {
	if (a > 2 * PI) { a -= 2 * PI; }
	else if (a < 0) { a += 2 * PI; }
	return a;
}

float shortestRotation(float a) {
	if (a > PI) { a -= 2 * PI; }
	else if (a < -PI) { a += 2 * PI; }
	return a;
}

static float constrainFloat(float x, float min, float max) {
	return ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)));
}

float calcT(float current_heading) {
    dt = to_us_since_boot(time) - last_contr_call;
    dt = max(dt * 1e-6, 1.0);
   
    alpha = shortestRotation(target_heading - current_heading);
    de = alpha - last_alpha;

    Tp = KaP * alpha;
    Td = constrainFloat(KaD * de / dt, -2.0, 2.0);

    last_alpha = alpha;
    last_contr_call = to_us_since_boot(time);

    return constrainFloat(Tp + Td, -0.5, 0.5);
}

void control(float current_heading) {
    T = calcT(current_heading);
    u_l_internal = Vforward + T;
    u_r_internal = Vforward - T;

    u_l = constrainFloat(abs(u_l_internal), minDC, maxDC);
    u_r = constrainFloat(abs(u_r_internal), minDC, maxDC);

    // Motor Commands
}
// ------------------------------------------------------------------------------------------------

int main()
{
    stdio_init_all();
    while(!tud_cdc_connected());

    // Main loop
    while(true) {
        // Get gyro value
            //kalmanPredict(gz, dt);

        // Get gps heading
            //z[0] = gps heading;
            //kalmanUpdate();

        control(wrapTo2Pi(xhat[0]));
    }
}