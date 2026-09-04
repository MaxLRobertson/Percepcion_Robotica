#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "as5600.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include "uros_network_interfaces.h"

/* Entradas analogicas (placa ESP32 clasica). Las tres pertenecen a ADC1,
 * que puede usarse con Wi-Fi; ADC2 no se usa por esa razon. */
#define GPIO_DIRECTO             32
#define GPIO_BUFFER              33
#define GPIO_AMPLIFICADO         34
#define CANAL_DIRECTO            ADC_CHANNEL_4
#define CANAL_BUFFER             ADC_CHANNEL_5
#define CANAL_AMPLIFICADO        ADC_CHANNEL_6

#define CANTIDAD_MUESTRAS        16
#define ADC_VALOR_MAXIMO         4095.0f
#define ADC_TENSION_NOMINAL_V    3.3f
#define TENSION_MAXIMA_POTE_V    1.96f
#define PERIODO_PUBLICACION_MS   100
#define MICROROS_STACK_BYTES     16000
#define MICROROS_TASK_PRIORITY   5

/* Bus I2C compartido con el encoder magnetico AS5600. */
#define GPIO_I2C_SDA             21
#define GPIO_I2C_SCL             22
#define I2C_PUERTO               I2C_NUM_0

/* Offset para que 0 grados del AS5600 coincida con el cero mecanico del
 * dispositivo diseñado. Ajustar con los datos de la Tabla 1 (practica 3). */
#define AS5600_OFFSET_GRADOS     0.0f

typedef struct {
    float tension_directa_v;
    float tension_buffer_v;
    float tension_amplificada_v;
    float posicion_porcentaje;
    uint16_t as5600_raw_angle;
    uint16_t as5600_angle;
    float as5600_grados;
    bool as5600_valido;
} medicion_t;

static const char *TAG = "potenciometro";

static adc_oneshot_unit_handle_t adc1;
static adc_cali_handle_t calibracion_adc;
static bool calibracion_disponible;

static rcl_publisher_t publicador_directo;
static rcl_publisher_t publicador_buffer;
static rcl_publisher_t publicador_amplificado;
static rcl_publisher_t publicador_posicion;
static rcl_publisher_t publicador_encoder_angulo;

static std_msgs__msg__Float32 mensaje_directo;
static std_msgs__msg__Float32 mensaje_buffer;
static std_msgs__msg__Float32 mensaje_amplificado;
static std_msgs__msg__Float32 mensaje_posicion;
static std_msgs__msg__Float32 mensaje_encoder_angulo;

#define RCCHECK(funcion)                                                        \
    do {                                                                        \
        const rcl_ret_t resultado = (funcion);                                  \
        if (resultado != RCL_RET_OK) {                                          \
            ESP_LOGE(TAG, "Error micro-ROS %d en linea %d",                    \
                     (int)resultado, __LINE__);                                 \
            vTaskDelete(NULL);                                                  \
        }                                                                       \
    } while (0)

#define RCSOFTCHECK(funcion)                                                    \
    do {                                                                        \
        const rcl_ret_t resultado = (funcion);                                  \
        if (resultado != RCL_RET_OK) {                                          \
            ESP_LOGW(TAG, "No se pudo publicar, error %d", (int)resultado);    \
        }                                                                       \
    } while (0)

static bool iniciar_calibracion_adc(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    /* Esquema del ESP32-S3 (y otros chips nuevos); la compensacion se */
    /* calcula por canal, se usa el del pote directo como referencia. */
    adc_cali_curve_fitting_config_t configuracion = {
        .unit_id = ADC_UNIT_1,
        .chan = CANAL_DIRECTO,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    const esp_err_t resultado =
        adc_cali_create_scheme_curve_fitting(&configuracion, &calibracion_adc);

    if (resultado == ESP_OK) {
        ESP_LOGI(TAG, "Calibracion ADC curve-fitting activada");
        return true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    /* Esquema del ESP32 clasico. */
    adc_cali_line_fitting_config_t configuracion = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    const esp_err_t resultado =
        adc_cali_create_scheme_line_fitting(&configuracion, &calibracion_adc);

    if (resultado == ESP_OK) {
        ESP_LOGI(TAG, "Calibracion ADC line-fitting activada");
        return true;
    }
#else
    ESP_LOGW(TAG, "Este chip no tiene un esquema de calibracion ADC soportado");
    ESP_LOGW(TAG, "Se usara la conversion nominal raw * 3.3 / 4095");
    return false;
#endif

    if (resultado == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibracion no disponible (eFuse sin quemar en este chip)");
    } else {
        ESP_LOGW(TAG, "No se pudo iniciar la calibracion ADC: %s",
                 esp_err_to_name(resultado));
    }

    ESP_LOGW(TAG, "Se usara la conversion nominal raw * 3.3 / 4095");
    return false;
}

static void configurar_adc(void)
{
    adc_oneshot_unit_init_cfg_t configuracion_unidad = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&configuracion_unidad, &adc1));

    adc_oneshot_chan_cfg_t configuracion_canal = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc1, CANAL_DIRECTO, &configuracion_canal));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc1, CANAL_BUFFER, &configuracion_canal));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc1, CANAL_AMPLIFICADO, &configuracion_canal));

    calibracion_disponible = iniciar_calibracion_adc();

    ESP_LOGI(TAG, "ADC1 configurado a 12 bits y 12 dB");
    ESP_LOGI(TAG, "GPIO%d=CH4 directo, GPIO%d=CH5 buffer, GPIO%d=CH6 amplificado",
             GPIO_DIRECTO, GPIO_BUFFER, GPIO_AMPLIFICADO);
}

static void configurar_i2c(void)
{
    const i2c_master_bus_config_t configuracion_bus = {
        .i2c_port = I2C_PUERTO,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_i2c;
    ESP_ERROR_CHECK(i2c_new_master_bus(&configuracion_bus, &bus_i2c));
    ESP_ERROR_CHECK(as5600_init(bus_i2c));

    ESP_LOGI(TAG, "I2C configurado a 400 kHz: SDA=GPIO%d SCL=GPIO%d",
             GPIO_I2C_SDA, GPIO_I2C_SCL);
}

static float convertir_a_voltios(int valor_raw)
{
    if (calibracion_disponible) {
        int milivoltios = 0;
        const esp_err_t resultado = adc_cali_raw_to_voltage(
            calibracion_adc, valor_raw, &milivoltios);

        if (resultado == ESP_OK) {
            return (float)milivoltios / 1000.0f;
        }

        ESP_LOGW(TAG, "Fallo la conversion calibrada: %s",
                 esp_err_to_name(resultado));
    }

    return ((float)valor_raw * ADC_TENSION_NOMINAL_V) / ADC_VALOR_MAXIMO;
}

static float leer_tension_promedio(adc_channel_t canal)
{
    float suma_voltios = 0.0f;

    for (int muestra = 0; muestra < CANTIDAD_MUESTRAS; muestra++) {
        int valor_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, canal, &valor_raw));
        suma_voltios += convertir_a_voltios(valor_raw);
    }

    /* Se promedian valores calibrados sin descartar su parte fraccionaria. */
    return suma_voltios / (float)CANTIDAD_MUESTRAS;
}

static float limitar_porcentaje(float porcentaje)
{
    if (porcentaje < 0.0f) {
        return 0.0f;
    }
    if (porcentaje > 100.0f) {
        return 100.0f;
    }
    return porcentaje;
}

static float aplicar_calibracion_as5600(float grados_crudos)
{
    float grados = grados_crudos - AS5600_OFFSET_GRADOS;

    while (grados < 0.0f) {
        grados += 360.0f;
    }
    while (grados >= 360.0f) {
        grados -= 360.0f;
    }

    return grados;
}

static medicion_t tomar_medicion(void)
{
    medicion_t medicion = {
        .tension_directa_v = leer_tension_promedio(CANAL_DIRECTO),
        .tension_buffer_v = leer_tension_promedio(CANAL_BUFFER),
        .tension_amplificada_v = leer_tension_promedio(CANAL_AMPLIFICADO),
    };

    medicion.posicion_porcentaje = limitar_porcentaje(
        (medicion.tension_directa_v / TENSION_MAXIMA_POTE_V) * 100.0f);

    const esp_err_t resultado_raw =
        as5600_leer_raw_angle(&medicion.as5600_raw_angle);
    const esp_err_t resultado_angle =
        as5600_leer_angle(&medicion.as5600_angle);

    medicion.as5600_valido =
        (resultado_raw == ESP_OK) && (resultado_angle == ESP_OK);
    medicion.as5600_grados = medicion.as5600_valido
        ? aplicar_calibracion_as5600(as5600_cuenta_a_grados(medicion.as5600_angle))
        : 0.0f;

    if (!medicion.as5600_valido) {
        ESP_LOGW(TAG, "No se pudo leer el AS5600 (raw=%d, angle=%d)",
                 (int)resultado_raw, (int)resultado_angle);
    }

    return medicion;
}

static void mostrar_medicion(const medicion_t *medicion)
{
    static unsigned int contador = 0;
    contador++;

    /* Un mensaje por segundo evita saturar el monitor serie. */
    if (contador >= 10) {
        contador = 0;
        ESP_LOGI(TAG,
                 "Directo %.5f V | Buffer %.5f V | Amp %.5f V | %.2f %% || "
                 "AS5600 RAW-ANGLE=%u ANGLE=%u (%.2f deg)",
                 medicion->tension_directa_v,
                 medicion->tension_buffer_v,
                 medicion->tension_amplificada_v,
                 medicion->posicion_porcentaje,
                 (unsigned)medicion->as5600_raw_angle,
                 (unsigned)medicion->as5600_angle,
                 medicion->as5600_grados);
    }
}

static void publicar_medicion(void)
{
    const medicion_t medicion = tomar_medicion();

    mensaje_directo.data = medicion.tension_directa_v;
    mensaje_buffer.data = medicion.tension_buffer_v;
    mensaje_amplificado.data = medicion.tension_amplificada_v;
    mensaje_posicion.data = medicion.posicion_porcentaje;
    mensaje_encoder_angulo.data = medicion.as5600_grados;

    RCSOFTCHECK(rcl_publish(&publicador_directo, &mensaje_directo, NULL));
    RCSOFTCHECK(rcl_publish(&publicador_buffer, &mensaje_buffer, NULL));
    RCSOFTCHECK(rcl_publish(&publicador_amplificado, &mensaje_amplificado, NULL));
    RCSOFTCHECK(rcl_publish(&publicador_posicion, &mensaje_posicion, NULL));
    if (medicion.as5600_valido) {
        RCSOFTCHECK(rcl_publish(
            &publicador_encoder_angulo, &mensaje_encoder_angulo, NULL));
    }

    mostrar_medicion(&medicion);
}

static void temporizador_callback(rcl_timer_t *temporizador, int64_t ultima_llamada)
{
    (void)ultima_llamada;

    if (temporizador != NULL) {
        publicar_medicion();
    }
}

static void esperar_agent(rmw_init_options_t *opciones_rmw)
{
    ESP_LOGI(TAG, "Esperando micro-ROS Agent en %s:%s",
             CONFIG_MICRO_ROS_AGENT_IP,
             CONFIG_MICRO_ROS_AGENT_PORT);

    while (rmw_uros_ping_agent_options(1000, 1, opciones_rmw) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Agent no disponible; reintentando");
    }
}

static void crear_publicador_float(rcl_publisher_t *publicador,
                                   rcl_node_t *nodo,
                                   const char *nombre_topic)
{
    *publicador = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_default(
        publicador,
        nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        nombre_topic));
}

static void tarea_micro_ros(void *argumento)
{
    (void)argumento;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t opciones = rcl_get_zero_initialized_init_options();

    RCCHECK(rcl_init_options_init(&opciones, allocator));

    rmw_init_options_t *opciones_rmw =
        rcl_init_options_get_rmw_init_options(&opciones);
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        opciones_rmw));

    esperar_agent(opciones_rmw);
    RCCHECK(rclc_support_init_with_options(
        &support, 0, NULL, &opciones, &allocator));

    rcl_node_t nodo = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(
        &nodo, "potentiometer_node", "", &support));

    crear_publicador_float(
        &publicador_directo, &nodo, "/pot/direct_voltage");
    crear_publicador_float(
        &publicador_buffer, &nodo, "/pot/buffer_voltage");
    crear_publicador_float(
        &publicador_amplificado, &nodo, "/pot/amplified_voltage");
    crear_publicador_float(
        &publicador_posicion, &nodo, "/pot/position");
    crear_publicador_float(
        &publicador_encoder_angulo, &nodo, "/encoder/angle");

    std_msgs__msg__Float32__init(&mensaje_directo);
    std_msgs__msg__Float32__init(&mensaje_buffer);
    std_msgs__msg__Float32__init(&mensaje_amplificado);
    std_msgs__msg__Float32__init(&mensaje_posicion);
    std_msgs__msg__Float32__init(&mensaje_encoder_angulo);

    rcl_timer_t temporizador = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default2(
        &temporizador,
        &support,
        RCL_MS_TO_NS(PERIODO_PUBLICACION_MS),
        temporizador_callback,
        true));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &temporizador));

    ESP_LOGI(TAG, "Nodo potentiometer_node listo; publicacion a 10 Hz");

    while (true) {
        RCSOFTCHECK(rclc_executor_spin_some(
            &executor, RCL_MS_TO_NS(20)));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    configurar_adc();
    configurar_i2c();

    ESP_LOGI(TAG, "Wi-Fi configurado: SSID=%s", CONFIG_ESP_WIFI_SSID);
    ESP_LOGI(TAG, "Agent configurado: %s:%s",
             CONFIG_MICRO_ROS_AGENT_IP,
             CONFIG_MICRO_ROS_AGENT_PORT);

    ESP_ERROR_CHECK(uros_network_interface_initialize());

    /* Evita que el ahorro de energia agregue latencia a los publishers confiables. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Ahorro de energia Wi-Fi desactivado para reducir latencia");

    const BaseType_t tarea_creada = xTaskCreate(
        tarea_micro_ros,
        "micro_ros",
        MICROROS_STACK_BYTES,
        NULL,
        MICROROS_TASK_PRIORITY,
        NULL);
    ESP_ERROR_CHECK(tarea_creada == pdPASS ? ESP_OK : ESP_FAIL);
}
