#include <math.h>
#include <string.h>

#include "rotaciones.h"

void mat_identidad(float M[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            M[i][j] = (i == j) ? 1.0f : 0.0f;
}

void mat_mul(float A[3][3], float B[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                out[i][j] += A[i][k] * B[k][j];
        }
    }
}

void rotacion_directa(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3])
{
    const float k = (float)(M_PI / 180.0);
    float sr = sinf(roll_deg * k),  cr = cosf(roll_deg * k);
    float sp = sinf(pitch_deg * k), cp = cosf(pitch_deg * k);
    float sy = sinf(yaw_deg * k),   cy = cosf(yaw_deg * k);

    R[0][0] = cy * cp;  R[0][1] = cy * sp * sr - sy * cr;  R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;  R[1][1] = sy * sp * sr + cy * cr;  R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;      R[2][1] = cp * sr;                 R[2][2] = cp * cr;
}

void aplicar_micro(float C[3][3], int eje, float ang_deg, int pasos)
{
    float d = (ang_deg / (float)pasos) * (float)(M_PI / 180.0);
    float R_mu[3][3];
    mat_identidad(R_mu);

    switch (eje) {
        case 0: R_mu[1][2] = -d; R_mu[2][1] =  d; break;
        case 1: R_mu[0][2] =  d; R_mu[2][0] = -d; break;
        case 2: R_mu[0][1] = -d; R_mu[1][0] =  d; break;
    }

    float C_nueva[3][3];
    for (int p = 0; p < pasos; p++) {
        mat_mul(C, R_mu, C_nueva);
        memcpy(C, C_nueva, sizeof(C_nueva));
    }
}

void ortogonalizar_matriz(float m[3][3])
{
    float n0 = sqrtf(m[0][0]*m[0][0] + m[0][1]*m[0][1] + m[0][2]*m[0][2]);
    m[0][0] /= n0; m[0][1] /= n0; m[0][2] /= n0;

    float dot = m[1][0]*m[0][0] + m[1][1]*m[0][1] + m[1][2]*m[0][2];
    m[1][0] -= dot * m[0][0]; m[1][1] -= dot * m[0][1]; m[1][2] -= dot * m[0][2];
    float n1 = sqrtf(m[1][0]*m[1][0] + m[1][1]*m[1][1] + m[1][2]*m[1][2]);
    m[1][0] /= n1; m[1][1] /= n1; m[1][2] /= n1;

    m[2][0] = m[0][1]*m[1][2] - m[0][2]*m[1][1];
    m[2][1] = m[0][2]*m[1][0] - m[0][0]*m[1][2];
    m[2][2] = m[0][0]*m[1][1] - m[0][1]*m[1][0];
}
