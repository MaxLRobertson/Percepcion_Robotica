#pragma once

/* Utilidades de matrices de rotacion 3x3 (reutilizadas del TP de microrotaciones). */

void mat_identidad(float M[3][3]);
/* out = A * B (out debe ser distinta de A y B) */
void mat_mul(float A[3][3], float B[3][3], float out[3][3]);
/* R = Rz(yaw) * Ry(pitch) * Rx(roll). Angulos en grados. */
void rotacion_directa(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]);
/* Aplica 'pasos' microrotaciones de ang_deg/pasos sobre un eje (0=X, 1=Y, 2=Z): C = C * R_mu. */
void aplicar_micro(float C[3][3], int eje, float ang_deg, int pasos);
/* Gram-Schmidt sobre las filas de la matriz. */
void ortogonalizar_matriz(float m[3][3]);
