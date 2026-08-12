//--------Librerías--------
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//Verifica que el build se configure con el middleware Micro XRCE-DDS.
//Si esto es así (usualmente es así) se habilita el header para setear-
//-IP/puerto del agente y opciones RMW.
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//Macros de chequeo y configuración:
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5

//--------Parámetros del encoder (ajustar acá)--------
#define ENCODER_GPIO_A          4      //GPIO señal A del encoder
#define ENCODER_GPIO_B          5      //GPIO señal B del encoder
#define ENCODER_PPR             600    //Pulsos por revolución (dato de fábrica)
#define ENCODER_DECODE_FACTOR   4      //x4: cuenta los 4 flancos (A y B, subida y bajada)
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * ENCODER_DECODE_FACTOR) //cuentas por vuelta -> 2400, resolución 0.15°

#define PCNT_HIGH_LIMIT          30000 //límite superior del registro de hw antes de wrap
#define PCNT_LOW_LIMIT          -30000 //límite inferior del registro de hw antes de wrap
#define PCNT_GLITCH_FILTER_NS    1000  //filtro de ruido/rebotes en ns

//Log tag y objetos globales:
static const char *TAG = "micro_ros";
rcl_publisher_t publisher;  //estos son globales para que el cb de timer pueda publicar.
std_msgs__msg__Float32MultiArray msg;
static float msg_data[3]; //[0]=velocidad rad/s, [1]=angulo acumulado deg, [2]=angulo relativo deg (0-360)

//--------Objetos y estado del PCNT--------
static pcnt_unit_handle_t pcnt_unit = NULL;
static volatile int64_t pulse_count_accum = 0; //acumula los wraps del contador de hw
static int64_t last_total_count = 0;           //cuentas totales en la lectura anterior
static int64_t last_time_us = 0;               //timestamp de la lectura anterior

//cb que dispara el PCNT al llegar a los watch points (high/low limit), o sea cuando
//el contador de hardware está por desbordar. Acumula el valor para no perder cuentas.
static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    if (edata->watch_point_value == PCNT_HIGH_LIMIT) {
        pulse_count_accum += PCNT_HIGH_LIMIT;
    } else if (edata->watch_point_value == PCNT_LOW_LIMIT) {
        pulse_count_accum += PCNT_LOW_LIMIT;
    }
    return false; //no se necesita despertar ninguna tarea de alta prioridad
}

//Configura el periférico PCNT en modo cuadratura x4 (decodifica los 4 flancos A/B).
static void encoder_pcnt_init(void)
{
    ESP_LOGI(TAG, "Configurando PCNT para encoder en GPIO A=%d, B=%d", ENCODER_GPIO_A, ENCODER_GPIO_B);

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = PCNT_GLITCH_FILTER_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    //Canal A: flanco en A, nivel de control dado por B.
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    //Canal B: flanco en B, nivel de control dado por A.
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_B,
        .level_gpio_num = ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    //Configuración de acciones para decodificación en cuadratura x4:
    //cada canal cuenta en ambos flancos, y el nivel del otro canal define si suma o resta.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    //Habilita el pull-up interno en A y B: necesario porque el encoder es de salida
    //open-collector y así no hace falta resistencia externa.
    ESP_ERROR_CHECK(gpio_set_pull_mode(ENCODER_GPIO_A, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(ENCODER_GPIO_B, GPIO_PULLUP_ONLY));

    //Watch points en los límites, para detectar el desborde del registro de hw.
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_LOW_LIMIT));

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    ESP_LOGI(TAG, "PCNT configurado correctamente (x4, %d cuentas/vuelta)", ENCODER_COUNTS_PER_REV);
}

//cb del timer: 

//* Se invoca periódicamente por el executor.
//* Lee el PCNT, calcula velocidad angular y ángulos, publica el mensaje y muestra por consola.

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);    //silencia warnings.
    if (timer != NULL) {
        int current_raw_count = 0;
        pcnt_unit_get_count(pcnt_unit, &current_raw_count);

        //cuentas totales acumuladas (incluyendo los wraps del registro de hw)
        int64_t total_count = pulse_count_accum + current_raw_count;

        //delta de tiempo real desde la última lectura (más preciso que asumir 1000ms fijo)
        int64_t now_us = esp_timer_get_time();
        double dt_s = (last_time_us == 0) ? 1.0 : (double)(now_us - last_time_us) / 1e6;
        if (dt_s <= 0) {
            dt_s = 1.0;
        }

        int64_t delta_count = total_count - last_total_count;

        //velocidad angular en rad/s
        float vel_rad_s = (float)(((double)delta_count / ENCODER_COUNTS_PER_REV) * 2.0 * M_PI / dt_s);

        //ángulo acumulado (sin límite, puede superar 360° o ser negativo)
        float angle_acum_deg = (float)(((double)total_count / ENCODER_COUNTS_PER_REV) * 360.0);

        //ángulo relativo, wrapeado entre 0 y 360°
        float angle_rel_deg = fmodf(angle_acum_deg, 360.0f);
        if (angle_rel_deg < 0.0f) {
            angle_rel_deg += 360.0f;
        }

        //arma y publica: [velocidad_rad_s, angulo_acumulado_deg, angulo_relativo_deg]
        msg_data[0] = vel_rad_s;
        msg_data[1] = angle_acum_deg;
        msg_data[2] = angle_rel_deg;

        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        ESP_LOGI(TAG, "Vel: %.3f rad/s | Angulo acumulado: %.2f deg | Angulo relativo: %.2f deg",
                 vel_rad_s, angle_acum_deg, angle_rel_deg);

        last_total_count = total_count;
        last_time_us = now_us;
    }
}

//Tarea principal de micro-ROS:
void micro_ros_task(void * arg)
{
    //configuración e inicialización de la gestión de mem:
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

    //configuración del trnasporte (si XRCE-DDS)
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        //obtiene rmw_options embebidas en init_options:
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	    //Fija la IP y puerto del micro-ROS Agent (por macros de sdkconfig, definidas vía menuconfig).
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    #endif

	//Crea el support que contiene el contexto ROS2 y config necesarias para crear nodos, timers, etc.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	//Nodo:
	rcl_node_t node;
	//crea el nodo llamado esp32_publisher.
    RCCHECK(rclc_node_init_default(&node, "esp32_publisher", "", &support));
    ESP_LOGI(TAG, "Nodo creado correctamente");

	//Se crea el publicador del tipo std_msgs/msg/Float32MultiArray en el tópico encoder_data
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"encoder_data"));
    ESP_LOGI(TAG, "Publisher creado correctamente.");

    //Inicializa el mensaje: usa un buffer estático, sin necesidad de rosidl_runtime_c allocators.
    msg.data.data = msg_data;
    msg.data.size = 3;
    msg.data.capacity = 3;
    msg.layout.dim.data = NULL;
    msg.layout.dim.size = 0;
    msg.layout.dim.capacity = 0;
    msg.layout.data_offset = 0;

	//Crea un timmer de 100ms (10 Hz) y cb timer_callback.
	rcl_timer_t timer;
	const unsigned int timer_timeout = 100;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback,
		true));

	//Crea el executor con capacidad para 1 handle (el timer en este caso).
    //Si quisieramos agregar suscripciones, servicios, etc. se aumenta la capacidad.
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	//Registra el timer en el executor:
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    //Loop principal.
	while(1){
        //Procesa eventos no bloqueantes, con un timeout de 100ms para esperar.
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000); //timer para no saturar la cpu.
	}
	//Liberación de recursos: 
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
  	vTaskDelete(NULL);
}

void app_main(void)
{
    //Si el transporte está configurado para wifi, inicializa la interfaz de red para micro-ros.
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    //Inicializa el periférico PCNT antes de arrancar la tarea de micro-ROS.
    encoder_pcnt_init();

    //Crea la tarea FreeRTOS con el stack y prioridad definidas.
    xTaskCreate(micro_ros_task,
            "micro_ros_task",
            MICRO_ROS_APP_STACK,
            NULL,
            MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
