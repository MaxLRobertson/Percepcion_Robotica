#include "as5600.h"

#include "esp_log.h"

#define AS5600_DIRECCION_I2C     0x36
#define AS5600_FRECUENCIA_HZ     400000
#define AS5600_TIMEOUT_MS        50

#define AS5600_REGISTRO_RAW_ANGLE 0x0C
#define AS5600_REGISTRO_ANGLE     0x0E
#define AS5600_REGISTRO_MAGNITUDE 0x1B

static const char *TAG = "as5600";

static i2c_master_dev_handle_t dispositivo_as5600;

esp_err_t as5600_init(i2c_master_bus_handle_t bus_i2c)
{
    const i2c_device_config_t configuracion_dispositivo = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_DIRECCION_I2C,
        .scl_speed_hz = AS5600_FRECUENCIA_HZ,
    };

    const esp_err_t resultado = i2c_master_bus_add_device(
        bus_i2c, &configuracion_dispositivo, &dispositivo_as5600);

    if (resultado == ESP_OK) {
        ESP_LOGI(TAG, "AS5600 agregado al bus I2C (0x%02X, %d Hz)",
                 AS5600_DIRECCION_I2C, AS5600_FRECUENCIA_HZ);
    } else {
        ESP_LOGE(TAG, "No se pudo agregar el AS5600 al bus I2C: %s",
                 esp_err_to_name(resultado));
    }

    return resultado;
}

static esp_err_t as5600_leer_registro_12_bits(uint8_t registro, uint16_t *valor)
{
    uint8_t datos[2] = {0};

    const esp_err_t resultado = i2c_master_transmit_receive(
        dispositivo_as5600, &registro, 1, datos, sizeof(datos),
        AS5600_TIMEOUT_MS);

    if (resultado != ESP_OK) {
        return resultado;
    }

    /* Los registros de 12 bits del AS5600 vienen en 2 bytes big-endian,
     * con los 4 bits mas significativos del primer byte en cero. */
    *valor = (uint16_t)(((datos[0] & 0x0F) << 8) | datos[1]);
    return ESP_OK;
}

esp_err_t as5600_leer_raw_angle(uint16_t *raw_angle)
{
    return as5600_leer_registro_12_bits(AS5600_REGISTRO_RAW_ANGLE, raw_angle);
}

esp_err_t as5600_leer_angle(uint16_t *angle)
{
    return as5600_leer_registro_12_bits(AS5600_REGISTRO_ANGLE, angle);
}

esp_err_t as5600_leer_magnitud(uint16_t *magnitud)
{
    return as5600_leer_registro_12_bits(AS5600_REGISTRO_MAGNITUDE, magnitud);
}

float as5600_cuenta_a_grados(uint16_t cuenta)
{
    return ((float)cuenta * 360.0f) / 4096.0f;
}
