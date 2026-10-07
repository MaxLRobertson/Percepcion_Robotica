//--------Librerías--------
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "esp_cpu.h"   // esp_cpu_get_cycle_count() (ESP-IDF 5.x)

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

//--------Macros--------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Poner en 1 para ortogonalizar la matriz de microrotación al final
// (usa la función que tenías comentada). En 0 se ve el error "puro" de la aproximación.
#define ORTOGONALIZAR_MICRO 0

#define CMD_LEN   7      // [vx, vy, vz, ax, ay, az, pasos]
#define MAX_PASOS 100000

static const char *TAG = "MICROROTACION";

//--------Objetos ROS globales (los usa el callback)--------
static rcl_subscription_t sub_cmd;
static rcl_publisher_t pub_dir;     // mat_orient       (rotación directa)
static rcl_publisher_t pub_mic;     // mat_orient_micro (microrotaciones)

static std_msgs__msg__Float32MultiArray msg_in;    // comando entrante
static std_msgs__msg__Float32MultiArray msg_dir;   // matriz directa saliente
static std_msgs__msg__Float32MultiArray msg_mic;   // matriz micro saliente

// Buffers estáticos (micro-ROS no reserva memoria solo para las secuencias)
static float buf_in[CMD_LEN];
static float buf_dir[9];
static float buf_mic[9];

//--------Matemática--------
static void mat_identidad(float M[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            M[i][j] = (i == j) ? 1.0f : 0.0f;
}

// out = A * B  (out debe ser distinta de A y B)
static void mat_mul(float A[3][3], float B[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                out[i][j] += A[i][k] * B[k][j];
        }
    }
}

// Rotación directa: R = Rz(yaw) * Ry(pitch) * Rx(roll). Ángulos en grados.
// (misma convención que usa la PC para extraer los ángulos)
static void rotacion_directa(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3])
{
    const float k = (float)(M_PI / 180.0);
    float sr = sinf(roll_deg * k),  cr = cosf(roll_deg * k);
    float sp = sinf(pitch_deg * k), cp = cosf(pitch_deg * k);
    float sy = sinf(yaw_deg * k),   cy = cosf(yaw_deg * k);

    R[0][0] = cy * cp;  R[0][1] = cy * sp * sr - sy * cr;  R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;  R[1][1] = sy * sp * sr + cy * cr;  R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;      R[2][1] = cp * sr;                 R[2][2] = cp * cr;
}

// Aplica 'pasos' microrotaciones de ángulo (ang_deg/pasos) alrededor de un eje.
// eje: 0 = X, 1 = Y, 2 = Z.  Usa C = C * R_mu (igual que tu código original).
static void aplicar_micro(float C[3][3], int eje, float ang_deg, int pasos)
{
    float d = (ang_deg / (float)pasos) * (float)(M_PI / 180.0);   // micro-ángulo [rad]
    float R_mu[3][3];
    mat_identidad(R_mu);

    switch (eje) {
        case 0: R_mu[1][2] = -d; R_mu[2][1] =  d; break;   // X
        case 1: R_mu[0][2] =  d; R_mu[2][0] = -d; break;   // Y
        case 2: R_mu[0][1] = -d; R_mu[1][0] =  d; break;   // Z
    }

    float C_nueva[3][3];
    for (int p = 0; p < pasos; p++) {
        mat_mul(C, R_mu, C_nueva);
        memcpy(C, C_nueva, sizeof(C_nueva));
    }
}

#if ORTOGONALIZAR_MICRO
static void ortogonalizar_matriz(float m[3][3])
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
#endif

// Pasa la matriz 3x3 al buffer del mensaje, fila por fila (row-major)
static void matriz_a_buffer(float M[3][3], float *buf)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            buf[3 * i + j] = M[i][j];
}

static void log_matriz(const char *nombre, float M[3][3])
{
    ESP_LOGI(TAG, "%s:", nombre);
    for (int i = 0; i < 3; i++)
        ESP_LOGI(TAG, "[ %.4f  %.4f  %.4f ]", M[i][0], M[i][1], M[i][2]);
}

//--------Callback de la suscripción a rot_cmd--------
// Llega: [vx, vy, vz, ax, ay, az, pasos]  (ángulos en grados)
// Se ejecuta cuando la PC aprieta "Enviar".
static void cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;

    if (m->data.size != CMD_LEN) {
        ESP_LOGW(TAG, "Comando ignorado: se esperaban %d valores y llegaron %d",
                 CMD_LEN, (int)m->data.size);
        return;
    }

    float roll  = m->data.data[3];    // ax
    float pitch = m->data.data[4];    // ay
    float yaw   = m->data.data[5];    // az
    int pasos   = (int)(m->data.data[6] + 0.5f);
    if (pasos < 1) pasos = 1;
    if (pasos > MAX_PASOS) pasos = MAX_PASOS;

    ESP_LOGI(TAG, "Comando: v=(%.3f, %.3f, %.3f) | Rx=%.3f Ry=%.3f Rz=%.3f | pasos=%d",
             m->data.data[0], m->data.data[1], m->data.data[2], roll, pitch, yaw, pasos);

    // ---- 1) Rotación directa ----
    uint32_t c0 = esp_cpu_get_cycle_count();

    float R_dir[3][3];
    rotacion_directa(roll, pitch, yaw, R_dir);

    // ---- 2) Microrotaciones ----
    // C = I * Rz^N * Ry^N * Rx^N  ->  equivale a Rz*Ry*Rx (misma convención).
    // Se hacen 'pasos' microrotaciones por cada eje con ángulo distinto de cero.
    uint32_t c1 = esp_cpu_get_cycle_count();

    float C[3][3];
    mat_identidad(C);
    if (yaw   != 0.0f) aplicar_micro(C, 2, yaw,   pasos);
    if (pitch != 0.0f) aplicar_micro(C, 1, pitch, pasos);
    if (roll  != 0.0f) aplicar_micro(C, 0, roll,  pasos);

    uint32_t c2 = esp_cpu_get_cycle_count();
#if ORTOGONALIZAR_MICRO
    ortogonalizar_matriz(C);
#endif

    log_matriz("Matriz directa (mat_orient)", R_dir);
    log_matriz("Matriz micro (mat_orient_micro)", C);

    ESP_LOGI(TAG, "Directa: %lu ciclos | Micro: %lu ciclos (pasos=%d)",
         (unsigned long)(c1 - c0), (unsigned long)(c2 - c1), pasos);

    // ---- 3) Publicar resultados ----
    matriz_a_buffer(R_dir, buf_dir);
    matriz_a_buffer(C, buf_mic);
    RCSOFTCHECK(rcl_publish(&pub_dir, &msg_dir, NULL));
    RCSOFTCHECK(rcl_publish(&pub_mic, &msg_mic, NULL));
}

//--------Tarea principal de micro-ROS--------
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Nodo
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_microrot", "", &support));
    ESP_LOGI(TAG, "Nodo creado correctamente");

    // Publishers (Float32MultiArray, 9 elementos, row-major)
    RCCHECK(rclc_publisher_init_default(
        &pub_dir, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "mat_orient"));
    RCCHECK(rclc_publisher_init_default(
        &pub_mic, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "mat_orient_micro"));

    // Suscriptor al comando de la PC
    RCCHECK(rclc_subscription_init_default(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "rot_cmd"));
    ESP_LOGI(TAG, "Publishers y suscriptor creados correctamente");

    // Memoria de los mensajes (hay que reservarla a mano en micro-ROS)
    msg_in.data.data = buf_in;
    msg_in.data.size = 0;
    msg_in.data.capacity = CMD_LEN;
    msg_in.layout.dim.data = NULL;
    msg_in.layout.dim.size = 0;
    msg_in.layout.dim.capacity = 0;
    msg_in.layout.data_offset = 0;

    msg_dir.data.data = buf_dir;
    msg_dir.data.size = 9;
    msg_dir.data.capacity = 9;
    msg_dir.layout.dim.data = NULL;
    msg_dir.layout.dim.size = 0;
    msg_dir.layout.dim.capacity = 0;
    msg_dir.layout.data_offset = 0;

    msg_mic.data.data = buf_mic;
    msg_mic.data.size = 9;
    msg_mic.data.capacity = 9;
    msg_mic.layout.dim.data = NULL;
    msg_mic.layout.dim.size = 0;
    msg_mic.layout.dim.capacity = 0;
    msg_mic.layout.data_offset = 0;

    // Executor con 1 handle (la suscripción)
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_in, &cmd_callback, ON_NEW_DATA));

    ESP_LOGI(TAG, "Esperando comandos en /rot_cmd ...");

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // Liberación de recursos (no se alcanza por el while(1), se deja por prolijidad)
    RCCHECK(rcl_subscription_fini(&sub_cmd, &node));
    RCCHECK(rcl_publisher_fini(&pub_dir, &node));
    RCCHECK(rcl_publisher_fini(&pub_mic, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                MICRO_ROS_APP_STACK,
                NULL,
                MICRO_ROS_APP_TASK_PRIO,
                NULL);
}