#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/* Encoder magnetico absoluto AS5600 (interfaz I2C, direccion fija 0x36). */

esp_err_t as5600_init(i2c_master_bus_handle_t bus_i2c);

/* Cuenta cruda del sensor (registro RAW ANGLE), 0..4095, sin filtrar ni escalar. */
esp_err_t as5600_leer_raw_angle(uint16_t *raw_angle);

/* Cuenta del sensor (registro ANGLE), 0..4095, con el filtrado y el escalado
 * por ZPOS/MPOS/MANG que tenga configurados el chip. */
esp_err_t as5600_leer_angle(uint16_t *angle);

/* Magnitud de la señal del iman (registro MAGNITUDE), util para verificar
 * que el iman esta bien alineado con el sensor. */
esp_err_t as5600_leer_magnitud(uint16_t *magnitud);

/* Convierte una cuenta de 12 bits (0..4095) a grados (0..360). */
float as5600_cuenta_a_grados(uint16_t cuenta);
