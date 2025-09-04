#ifndef KALMAN_VARS_H

#define KALMAN_VARS_H
#ifndef PI
#define PI 3.14159265359
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD PI / 180.0f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 180.0f / PI
#endif

extern float xhat_heading[2];
extern float z_heading[2];
extern float xhat_X[2];
extern float z_X[2];
extern float xhat_Y[2];
extern float z_Y[2];

// Linear Algebra Funcs ---------------------------------------------------------------------------
void matmul2x2(float out[4], float m1[4], float m2[4]);
void matmul2x1(float out[2], float m1[4], float m2[2]);
void transpose2x2(float out[4], float matrix[4]);
void add2x2(float m1[4], float m2[4]);
void subtract2x2(float m1[4], float m2[4]);
void invert2x2(float matrix[4]);

// ------------------------------------------------------------------------------------------------
static void kalmanPredict(float u, float dt, float A[4], float B[2], float xhat[2], float P[4], float Q[4]);
static void kalmanUpdate(float G[4], float P[4], float C[4], float R[4], float xhat[2], float z[2]);

void kalmanPredict_heading(float u, float dt);
void kalmanUpdate_heading();

void kalmanPredict_X(float u, float dt);
void kalmanUpdate_X();

void kalmanPredict_Y(float u, float dt);
void kalmanUpdate_Y();

#endif
