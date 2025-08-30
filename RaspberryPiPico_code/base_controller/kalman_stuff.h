#ifndef KALMAN_VARS_H

#define KALMAN_VARS
#ifndef PI
#define PI 3.14159265359
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD PI / 180.0f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 180.0f / PI
#endif

// Kalman Filter Matricies ------------------------------------------------------------------------
//  For indexing matricies they will be sequentially indexed from 0 to n going
//      left to right, then top down

float xhat_heading[2] = {
    0.0,
    0.0
};
float z_heading[2] = {
    0.0,
    0.0
};
float A_heading[4] = {
    1.0, 0.0,
    0.0, 1.0
};
float A_heading_transpose[4];
float B_heading[2] = {
    0.0,
    0.0
};

float P_heading[4] = {
    0.0, 0.0,
    0.0, 0.0
};
float P_heading_temp[4];
float C_heading[4] = {
    1.0, 0.0,
    0.0, 1.0
};
float C_heading_transpose[4];
float Q_heading[4] = {
    0.1, 0.0,
    0.0, 0.01
};
float G_heading[4];
float G_heading_temp[4];
const float gps_heading_variance = (0.5 * DEG_TO_RAD) * (0.5 * DEG_TO_RAD);
const float gyro_variance = (0.07 * DEG_TO_RAD) * (0.07 * DEG_TO_RAD);
float R_heading[4] = {
    gps_heading_variance, 0.0,
    0.0, gyro_variance
};

float xhat_X[2] = {
    0.0,
    0.0
};
float z_X[2] = {
    0.0,
    0.0
};
float A_X[4] = {
    1.0, 0.0,
    0.0, 1.0
};
float A_X_transpose[4];
float B_X[2] = {
    0.0,
    0.0
};

float P_X[4] = {
    0.0, 0.0,
    0.0, 0.0
};
float P_X_temp[4];
float C_X[4] = {
    1.0, 0.0,
    0.0, 1.0
};
float C_X_transpose[4];
float Q_X[4] = {
    0.1, 0.0,
    0.0, 0.01
};
float G_X[4];
float G_X_temp[4];

float xhat_Y[2] = {
    0.0,
    0.0
};
float z_Y[2] = {
    0.0,
    0.0
};
float A_Y[4] = {
    1.0, 0.0,
    0.0, 1.0
};
float A_Y_transpose[4];
float B_Y[2] = {
    0.0,
    0.0
};

float P_Y[4] = {
    0.0, 0.0,
    0.0, 0.0
};
float P_Y_temp[4];
float C_Y[4] = {
    1.0, 0.0,
    0.0, 1.0
};
float C_Y_transpose[4];
float Q_Y[4] = {
    0.1, 0.0,
    0.0, 0.01
};
float G_Y[4];
float G_Y_temp[4];

const float gps_pos_variance = 2.5 * 2.5;
const float vel_variance = 0.1 * 0.1;
float R_pos[4] = {
    gps_pos_variance, 0.0,
    0.0, vel_variance
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

void kalmanPredict_heading(float u, float dt) {
    // xhat = A * xhat + B * u
    A_heading[1] = dt;
    B_heading[0] = -dt;
    matmul2x1(xhat_heading, A_heading, xhat_heading);
    xhat_heading[0] = xhat_heading[0] + B_heading[0] * u; 

    // P = A * P * A^T + Q
    matmul2x2(P_heading, A_heading, P_heading);
    transpose2x2(A_heading_transpose, A_heading);
    matmul2x2(P_heading, P_heading, A_heading_transpose);
    add2x2(P_heading, Q_heading);
}

void kalmanUpdate_heading(){
    float I[4] = {
        1.0, 0.0,
        0.0, 1.0
    };
    
    // G = P * C^T * ( C * P * C^T + R)^-1
    transpose2x2(C_heading_transpose, C_heading);
    matmul2x2(G_heading, P_heading, C_heading_transpose);
    matmul2x2(G_heading_temp, C_heading, P_heading);
    matmul2x2(G_heading_temp, G_heading_temp, C_heading_transpose);
    add2x2(G_heading_temp, R_heading);
    invert2x2(G_heading_temp);
    matmul2x2(G_heading, G_heading, G_heading_temp);

    // xhat = xhat + G * (z - C * xhat)
    matmul2x1(xhat_heading, C_heading, xhat_heading);
    z_heading[0] = z_heading[0] - xhat_heading[0];
    z_heading[1] = z_heading[1] - xhat_heading[1];
    matmul2x1(z_heading, G_heading, z_heading);
    xhat_heading[0] = xhat_heading[0] + z_heading[0];
    xhat_heading[1] = xhat_heading[1] + z_heading[1];

    // P = (I - G * C) * P
    subtract2x2(I, G_heading);
    matmul2x2(P_heading_temp, I, C_heading);
    matmul2x2(P_heading, P_heading_temp, P_heading);
}

void kalmanPredict_X(float u, float dt) {
    // xhat = A * xhat + B * u
    A_X[1] = dt;
    B_X[0] = -dt;
    matmul2x1(xhat_X, A_X, xhat_X);
    xhat_X[0] = xhat_X[0] + B_X[0] * u; 

    // P = A * P * A^T + Q
    matmul2x2(P_X, A_X, P_X);
    transpose2x2(A_X_transpose, A_X);
    matmul2x2(P_X, P_X, A_X_transpose);
    add2x2(P_X, Q_X);
}

void kalmanUpdate_X(){
    float I[4] = {
        1.0, 0.0,
        0.0, 1.0
    };
    
    // G = P * C^T * ( C * P * C^T + R)^-1
    transpose2x2(C_X_transpose, C_X);
    matmul2x2(G_X, P_X, C_X_transpose);
    matmul2x2(G_X_temp, C_X, P_X);
    matmul2x2(G_X_temp, G_X_temp, C_X_transpose);
    add2x2(G_X_temp, R_pos);
    invert2x2(G_X_temp);
    matmul2x2(G_X, G_X, G_X_temp);

    // xhat = xhat + G * (z - C * xhat)
    matmul2x1(xhat_X, C_X, xhat_X);
    z_X[0] = z_X[0] - xhat_X[0];
    z_X[1] = z_X[1] - xhat_X[1];
    matmul2x1(z_X, G_X, z_X);
    xhat_X[0] = xhat_X[0] + z_X[0];
    xhat_X[1] = xhat_X[1] + z_X[1];

    // P = (I - G * C) * P
    subtract2x2(I, G_X);
    matmul2x2(P_X_temp, I, C_X);
    matmul2x2(P_X, P_X_temp, P_X);
}

void kalmanPredict_Y(float u, float dt) {
    // xhat = A * xhat + B * u
    A_Y[1] = dt;
    B_Y[0] = -dt;
    matmul2x1(xhat_Y, A_Y, xhat_Y);
    xhat_Y[0] = xhat_Y[0] + B_Y[0] * u; 

    // P = A * P * A^T + Q
    matmul2x2(P_Y, A_Y, P_Y);
    transpose2x2(A_Y_transpose, A_Y);
    matmul2x2(P_Y, P_Y, A_Y_transpose);
    add2x2(P_Y, Q_Y);
}

void kalmanUpdate_Y(){
    float I[4] = {
        1.0, 0.0,
        0.0, 1.0
    };
    
    // G = P * C^T * ( C * P * C^T + R)^-1
    transpose2x2(C_Y_transpose, C_Y);
    matmul2x2(G_Y, P_Y, C_Y_transpose);
    matmul2x2(G_Y_temp, C_Y, P_Y);
    matmul2x2(G_Y_temp, G_Y_temp, C_Y_transpose);
    add2x2(G_Y_temp, R_pos);
    invert2x2(G_Y_temp);
    matmul2x2(G_Y, G_Y, G_Y_temp);

    // xhat = xhat + G * (z - C * xhat)
    matmul2x1(xhat_Y, C_Y, xhat_Y);
    z_Y[0] = z_Y[0] - xhat_Y[0];
    z_Y[1] = z_Y[1] - xhat_Y[1];
    matmul2x1(z_Y, G_Y, z_Y);
    xhat_Y[0] = xhat_Y[0] + z_Y[0];
    xhat_Y[1] = xhat_Y[1] + z_Y[1];

    // P = (I - G * C) * P
    subtract2x2(I, G_Y);
    matmul2x2(P_Y_temp, I, C_Y);
    matmul2x2(P_Y, P_Y_temp, P_Y);
}

#endif
