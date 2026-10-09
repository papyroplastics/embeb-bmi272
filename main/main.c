#include <stdint.h>

#include "soc/gpio_num.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <portmacro.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

#include "bmi270.h"

float map_int16_to_float(int16_t x, float range) {
  if (x < 0)
    return (float)x * range / ((float)INT16_MAX + 1);
  else
    return (float)x * range / (float)INT16_MAX;
}

void app_main(void) {
  /* Init driver */
  i2c_master_bus_handle_t bus_handle;
  i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 5,
    .scl_io_num = 4,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

  i2c_master_dev_handle_t dev_handle;
  i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BMI2_I2C_PRIM_ADDR,
    .scl_speed_hz = 400000,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

  /* Init a */
  uint8_t buf[2];
  buf[0] = BMI2_CHIP_ID_ADDR;
  uint8_t chip_id = 0;
  ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 1, &chip_id, 1, -1));
  if (chip_id != BMI270_DSD_CHIP_ID) {
    printf("CHIP_ID=0x%hhX\n", chip_id);
    return;
  }

  /* Init b */
  buf[0] = BMI2_PWR_CONF_ADDR;
  buf[1] = 0x00;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));
  vTaskDelay(pdMS_TO_TICKS(1));

  buf[0] = BMI2_INIT_CTRL_ADDR;
  buf[1] = 0x00;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));

  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, bmi270_config_file, bmi270_config_file_size, -1));

  buf[0] = BMI2_INIT_CTRL_ADDR;
  buf[1] = 0x01;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));

  /* Init c */
  vTaskDelay(pdMS_TO_TICKS(20));
  uint8_t init_status = 0;
  buf[0] = BMI2_INTERNAL_STATUS_ADDR;
  ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 1, &init_status, 1, -1));
  if ((init_status & 1) != 1) {
    printf("INIT_STATUS=0x%hhX\n", init_status);
    return;
  }

  /* Sensor config */
  buf[0] = BMI2_PWR_CTRL_ADDR;
  buf[1] = 0b0110;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));

  buf[0] = BMI2_ACC_CONF_ADDR;
  buf[1] = 0x01<<7 | 0x02<<4 | 0x0b;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));

  buf[0] = BMI2_ACC_RANGE_ADDR;
  buf[1] = 0x02;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));

  buf[0] = BMI2_GYR_CONF_ADDR;
  buf[1] = 0x01<<7 | 0x02<<4 | 0x0b;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));

  buf[0] = BMI2_GYR_RANGE_ADDR;
  buf[1] = 0x01;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));

  buf[0] = BMI2_PWR_CONF_ADDR;
  buf[1] = 0x02;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));

  /* Read data */
  TickType_t previous_time = xTaskGetTickCount();

  for (uint32_t i = 0; i < 1000; i++) {
    buf[0] = BMI2_ACC_X_LSB_ADDR;
    uint16_t sensor_data[6];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 1, (uint8_t*)sensor_data, 12, -1));

    printf("ACC:\t");
    for (uint32_t j = 0; j < 3; j++) {
      printf("%.3f\t", map_int16_to_float(sensor_data[j], 8));
    }
    printf("GYR:\t");
    for (uint32_t j = 3; j < 6; j++) {
      printf("%.3f\t", map_int16_to_float(sensor_data[j], 1000));
    }
    printf("\n");

    xTaskDelayUntil(&previous_time, pdMS_TO_TICKS(1));
  }

  /* Deinit driver */
  ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
  ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
  printf("I2C de-initialized successfully\n");
}
