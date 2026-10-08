//--------Librerías--------
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
// #include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "driver/i2c_master.h"  // Nuevo driver obligatorio en ESP-IDF v5.x

#include "freertos/semphr.h"

SemaphoreHandle_t uros_mutex = NULL;

//Verifica que el build se configure con el middleware Micro XRCE-DDS.
//Si esto es así (usualmente es así) se habilita el header para setear-
//-IP/puerto del agente y opciones RMW.
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

//Macros de chequeo y configuración:
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5

//Log tag y objetos globales:
// static const char *TAG = "micro_ros";
rcl_publisher_t publisher;  //estos dos son gloables para que el cb de timer pueda publicar.
// std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu msg;

static const char *TAG = "ESP32S3_max";

// Configuración de Pines I2C para ESP32-S3
#define I2C_MASTER_SDA_IO           11    
#define I2C_MASTER_SCL_IO           12    
#define I2C_MASTER_FREQ_HZ          400000 

// Registros de la MPU6050
#define MPU6050_ADDR                0x68  
#define MPU6050_ACCEL_XOUT_H        0x3B  
#define MPU6050_PWR_MGMT_1          0x6B  

//Sensibilidad para los fondos de escala más chicos (los que configura este código):
//±2g para acelerómetro, ±250°/s para giroscopio. Si cambiás *_CONFIG, hay que
//actualizar también estos factores según la tabla del datasheet.
#define MPU6050_ACCEL_SENS_LSB_PER_G     16384.0f
#define MPU6050_GYRO_SENS_LSB_PER_DEGS   131.0f

#define GRAVITY_MS2       9.80665f
#define DEG_TO_RAD        0.017453293f

#define IMU_FRAME_ID       "imu_link"
#define PUBLISH_PERIOD_MS   100   //100ms = 10Hz

// Handles globales para el nuevo driver estructurado de Espressif
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t mpu6050_dev_handle;



/**
 * @brief Inicializa el Bus Maestro I2C y añade la MPU6050 como dispositivo esclavo
 */
static esp_err_t i2c_master_init(void) {
    // 1. Configurar el bus maestro global
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) return err;

    // 2. Vincular el dispositivo esclavo MPU6050 al bus asignado
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_config, &mpu6050_dev_handle);
}

/**
 * @brief Despierta a la MPU6050 configurando el registro de energía
 */
static esp_err_t mpu6050_init(void) {
    // Enviar al registro PWR_MGMT_1 (0x6B) el valor 0x00 para sacarlo de Sleep Mode
    uint8_t wake_cmd[2] = {MPU6050_PWR_MGMT_1, 0x00};
    return i2c_master_transmit(mpu6050_dev_handle, wake_cmd, sizeof(wake_cmd), -1);
}



// Variable global para guardar la referencia/manejador de la tarea
TaskHandle_t imu_task_handle = NULL;


//cb del timer: 

//* Se invoca periódicamente por el executor.
//* Publica msg en el tópico del publisher y luego incrementa msg.data.

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);    //silencia warnings.
	if (timer != NULL && imu_task_handle != NULL ) {

		// Despierta inmediatamente a la tarea de la IMU
        xTaskNotifyGive(imu_task_handle);
	}
}


void lectura_imu_task(void * pvParameters)
{
	uint8_t reg_start = MPU6050_ACCEL_XOUT_H;
	uint8_t raw_buffer[14]; // Buffer para los 14 bytes secuenciales del sensor
	int16_t ax_raw, ay_raw, az_raw, temp, gx_raw, gy_raw, gz_raw;

	ESP_LOGI(TAG, "Tarea de lectura IMU iniciada y esperando al Timer...");

	while (1)
	{
		// Bloquea la tarea indefinidamente (portMAX_DELAY) hasta que el timer_callback llame a xTaskNotifyGive
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


		// En v5.x se usa i2c_master_transmit_receive para operaciones combinadas de lectura de registros
		esp_err_t ret = i2c_master_transmit_receive(mpu6050_dev_handle, &reg_start, 1, raw_buffer, 14, -1);
		
		if (ret == ESP_OK) {
			// Recomponer correctamente los bytes significativos altos (H) y bajos (L)
			ax_raw = (int16_t)((raw_buffer[0] << 8) | raw_buffer[1]);
			ay_raw = (int16_t)((raw_buffer[2] << 8) | raw_buffer[3]);
			az_raw = (int16_t)((raw_buffer[4] << 8) | raw_buffer[5]);
			
			temp = (int16_t)((raw_buffer[6] << 8) | raw_buffer[7]);
			
			gx_raw = (int16_t)((raw_buffer[8] << 8) | raw_buffer[9]);
			gy_raw = (int16_t)((raw_buffer[10] << 8) | raw_buffer[11]);
			gz_raw = (int16_t)((raw_buffer[12] << 8) | raw_buffer[13]);

			// Ecuación matemática del datasheet para la conversión de temperatura
			float temperature_c = ((float)temp / 340.0) + 36.53;

			ESP_LOGI(TAG, "ACCEL: X=%d | Y=%d | Z=%d  ||  GYRO: X=%d | Y=%d | Z=%d  ||  TEMP: %.2f°C",
					ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temperature_c);


			//conversión a unidades SI: aceleración en m/s^2, velocidad angular en rad/s.
			msg.linear_acceleration.x = (ax_raw / MPU6050_ACCEL_SENS_LSB_PER_G) * GRAVITY_MS2;
			msg.linear_acceleration.y = (ay_raw / MPU6050_ACCEL_SENS_LSB_PER_G) * GRAVITY_MS2;
			msg.linear_acceleration.z = (az_raw / MPU6050_ACCEL_SENS_LSB_PER_G) * GRAVITY_MS2;

			msg.angular_velocity.x = ((gx_raw / MPU6050_GYRO_SENS_LSB_PER_DEGS) * DEG_TO_RAD);
			msg.angular_velocity.y = ((gy_raw / MPU6050_GYRO_SENS_LSB_PER_DEGS) * DEG_TO_RAD);
			msg.angular_velocity.z = ((gz_raw / MPU6050_GYRO_SENS_LSB_PER_DEGS) * DEG_TO_RAD);

			// 🔒 Tomamos el Mutex antes de publicar
			if (xSemaphoreTake(uros_mutex, portMAX_DELAY) == pdTRUE) {
				RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
				xSemaphoreGive(uros_mutex); // 🔓 Lo soltamos inmediatamente
			}

			// RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
			ESP_LOGI(TAG, "Publicanding...");
			
		} else {
			ESP_LOGE(TAG, "Error de lectura I2C (Código: %s)", esp_err_to_name(ret));
		}


        //publica:
		// RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		// ESP_LOGI(TAG, "Publishing: %d", (int)msg.data);
        // msg.data++;
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
    	RCCHECK(rclc_node_init_default(&node, "imu_reader", "", &support));
    ESP_LOGI(TAG, "Nodo creado correctamente");
	
	//Se crea el publicador del tipo std_msgs/msg/Int32 en el tópico datos_int32
		RCCHECK(rclc_publisher_init_default(
			&publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
			"imu_data"));
		ESP_LOGI(TAG, "Publisher creado correctamente.");


	//--------Inicialización del mensaje (una sola vez, no cambia por ciclo)--------
    // rosidl_runtime_c__String__init(&msg.header.frame_id);
    // rosidl_runtime_c__String__assign(&msg.header.frame_id, IMU_FRAME_ID);

    //No calculamos orientación en esta etapa (sin DMP/fusión): se marca como no
    //disponible con covariance[0] = -1, tal como indica la convención del mensaje Imu.
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;

    //angular_velocity_covariance y linear_acceleration_covariance quedan en 0 (todos
    //los elementos de msg ya arrancan en 0 por ser una variable global): 0 indica
    //"covarianza desconocida", válido mientras no caractericemos el ruido del sensor.
	
	//Crea un timmer de 1000ms y cb timer_callback.
	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
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

	// msg.data = 0;

    // //Loop principal.
	// while(1){
    //     //Procesa eventos no bloqueantes, con un timeout de 100ms para esperar.
		// rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	// 	usleep(1000); //timer para no saturar la cpu.

	// }

	// rclc_executor_spin(&executor);


	while(1) {
		// 🔒 Tomamos el Mutex antes de revisar la red
		if (xSemaphoreTake(uros_mutex, portMAX_DELAY) == pdTRUE) {
			// Revisa la red por solo 10ms
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
			xSemaphoreGive(uros_mutex); // 🔓 Lo soltamos para que la IMU pueda publicar
		}
		
		// Le damos un respiro de 10ms al RTOS
		vTaskDelay(pdMS_TO_TICKS(10)); 
	}


	//Liberación de recursos: 
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
  	vTaskDelete(NULL);
}

void app_main(void)
{
	// 1. Inicializar periférico I2C bajo el nuevo estándar v5.x
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "Nuevo Bus I2C de ESP-IDF v5 inicializado.");

    // 2. Despertar Sensor
    if (mpu6050_init() == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 vinculada y despertada con éxito.");
    } else {
        ESP_LOGE(TAG, "Fallo de comunicación con MPU6050. Revisa cableado.");
        return;
    }
	
	//Creo un semaforo para que las tareas no se pelien
	uros_mutex = xSemaphoreCreateMutex();


    //Si el transporte está configurado para wifi, inicializa la interfaz de red para micro-ros.
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

	//Creacion de la tarea de lectura de IMU y asignacion a nucleo 1
	xTaskCreatePinnedToCore(lectura_imu_task,
            "lectura_imu_task",
            4096,
            NULL,
            MICRO_ROS_APP_TASK_PRIO +1,
            &imu_task_handle,
			1);

    //Crea la tarea FreeRTOS con el stack y prioridad definidas.
    xTaskCreatePinnedToCore(micro_ros_task,
            "micro_ros_task",
            MICRO_ROS_APP_STACK,
            NULL,
            MICRO_ROS_APP_TASK_PRIO,
            NULL,
			0);
}