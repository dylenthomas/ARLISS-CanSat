#include "kalman_stuff.h"

// Kalman Filter Matricies ------------------------------------------------------------------------
//  For indexing matricies they will be sequentially indexed from 0 to n going
//      left to right, then top down

float xhat_heading[2] = { 0.0, 0.0 };
float z_heading[2] = { 0.0, 0.0 };
static float A_heading[4] = { 1.0, 0.0, 0.0, 1.0 };
static float B_heading[2] = { 0.0, 0.0 };

static float P_heading[4] = { 0.0, 0.0, 0.0, 0.0 };
static float C_heading[4] = { 1.0, 0.0, 0.0, 1.0 };
static float Q_heading[4] = { 0.1, 0.0, 0.0, 0.01};
static float G_heading[4];
static const float gps_heading_variance = (0.5 * DEG_TO_RAD) * (0.5 * DEG_TO_RAD);
static const float gyro_variance = (0.07 * DEG_TO_RAD) * (0.07 * DEG_TO_RAD);
static float R_heading[4] = { gps_heading_variance, 0.0, 0.0, gyro_variance };

float xhat_X[2] = { 0.0, 0.0 };
float z_X[2] = { 0.0, 0.0 };
static float A_X[4] = { 1.0, 0.0, 0.0, 1.0 };
static float B_X[2] = { 0.0, 0.0 };

static float P_X[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
static float G_X[4];

float xhat_Y[2] = { 0.0, 0.0 };
float z_Y[2] = { 0.0, 0.0 };
static float A_Y[4] = { 1.0, 0.0, 0.0, 1.0 };
static float B_Y[2] = { 0.0, 0.0};

static float P_Y[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
static float G_Y[4];

static float Q_pos[4] = { 0.05, 0.0, 0.0, 0.15 };
static const float gps_pos_variance = 2.5 * 2.5;
static const float vel_variance = 0.1 * 0.1;
static float R_pos[4] = { gps_pos_variance, 0.0, 0.0, vel_variance };
static float C_pos[4] = { 1.0, 0.0, 0.0, 1.0 };
static float C_pos_transpose[4];

// Linear Algebra Funcs ---------------------------------------------------------------------------
void matmul2x2(float out[4], float m1[4], float m2[4]) {
    // matrix multiplication to multiply two 2x2s
    float temp[4];
    temp[0] = m1[0] * m2[0] + m1[1] * m2[2];
    temp[1] = m1[0] * m2[1] + m1[1] * m2[3];
    temp[2] = m1[2] * m2[0] + m1[3] * m2[2];
    temp[3] = m1[2] * m2[1] + m1[3] * m2[3];
    for (int i = 0; i < 4; i++) {out[i] = temp[i];}
}

void matmul2x1(float out[2], float m1[4], float m2[2]) {
    // matrix multiplication to multiply 2x2 x 2x1
    float temp[2];
    temp[0] = m1[0] * m2[0] + m1[1] * m2[1];
    temp[1] = m1[2] * m2[0] + m1[3] * m2[1];
    for(int i = 0; i < 2; i++) {out[i] = temp[i];}
}

void transpose2x2(float out[4], float matrix[4]) {
    float temp[4];
    temp[0] = matrix[0];
    temp[1] = matrix[2];
    temp[2] = matrix[1];
    temp[3] = matrix[3];
    for (int i = 0; i < 4; i++) {out[i] = temp[i];}
}

void add2x2(float m1[4], float m2[4]) {
    // matrix addition between two 2x2s
    for (int i = 0; i < 4; i++) {
        m1[i] += m2[i];
    }
}

void subtract2x2(float m1[4], float m2[4]) {
    // matrix addition between two 2x2s
    for (int i = 0; i < 4; i++) {
        m1[i] -= m2[i];
    }
}

void invert2x2(float matrix[4]) {
    float det = matrix[0] * matrix[3] - matrix[2] * matrix[1];
    if (det == 0) { return; }

    float temp[4];
    temp[0] = matrix[3] / det;
    temp[1] = -matrix[1] / det;
    temp[2] = -matrix[2] / det;
    temp[3] = matrix[0] / det;
    for (int i = 0; i < 4; i++) {matrix[i] = temp[i];}
}

// ------------------------------------------------------------------------------------------------
static void kalmanPredict(
    float u,
    float dt,
    float A[4],
    float B[2],
    float xhat[2],
    float P[4],
    float Q[4]
) {
    float Ax[2];
    float AP[4];
    float A_transpose[4];
    float APAt[4];

    // xhat = A*xhat + B*u
    A[1] = dt;
    B[0] = -dt;
    //matmul2x1(Ax, A, xhat);
    //xhat[0] = Ax[0] + B[0] * u;
    xhat[0] = xhat[0] + dt * (u + xhat[1]);

    // P = A*P*A^T + Q
    matmul2x2(AP, A, P);
    transpose2x2(A_transpose, A);
    matmul2x2(APAt, AP, A_transpose);
    add2x2(APAt, Q);

    // copy values
    for (int i = 0; i < 4; i++) {
        P[i] = APAt[i];
    }
}

static void kalmanUpdate(
    float G[4],
    float P[4],
    float C[4],
    float R[4],
    float xhat[2],
    float z[2]
) {
    float C_transpose[4];
    float PCt[4];
    float CP[4];
    float CPCt[4];
    float Cxhat[2];
    float G_temp[2];
    float I[4] = {
        1.0, 0.0,
        0.0, 1.0
    };
    float GC[4];
    float P_temp[4];
    float I_transpose[4];

    // G = P * C^T * ( C * P * C^T + R)^-1
    transpose2x2(C_transpose, C);
    matmul2x2(PCt, P, C_transpose);
    matmul2x2(CP, C, P);
    matmul2x2(CPCt, CP, C_transpose);
    add2x2(CPCt, R);
    invert2x2(CPCt);
    matmul2x2(G, PCt, CPCt);

    // xhat = xhat + G * (z - C * xhat)
    matmul2x1(Cxhat, C, xhat);
    Cxhat[0] = z[0] - Cxhat[0];
    Cxhat[1] = z[1] - Cxhat[1];
    matmul2x1(G_temp, G, Cxhat);
    xhat[0] += G_temp[0];
    xhat[1] += G_temp[1];

    // P = (I - G * C) * P * (I - G * C)^T
    matmul2x2(GC, G, C);
    subtract2x2(I, GC);
    matmul2x2(P_temp, I, P);
    transpose2x2(I_transpose, I);
    matmul2x2(P, P_temp, I_transpose);
}

void kalmanPredict_heading(float u, float dt) {
    kalmanPredict(u, dt, A_heading, B_heading, xhat_heading, P_heading, Q_heading);
}

void kalmanUpdate_heading(){
    kalmanUpdate(G_heading, P_heading, C_heading, R_heading, xhat_heading, z_heading);
}

void kalmanPredict_X(float u, float dt) {
    kalmanPredict(u, dt, A_X, B_X, xhat_X, P_X, Q_pos);
}

void kalmanUpdate_X(){
    kalmanUpdate(G_X, P_X, C_pos, R_pos, xhat_X, z_X);
}

void kalmanPredict_Y(float u, float dt) {
    kalmanPredict(u, dt, A_Y, B_Y, xhat_Y, P_Y, Q_pos);
}

void kalmanUpdate_Y(){
    kalmanUpdate(G_Y, P_Y, C_pos, R_pos, xhat_Y, z_Y);
}