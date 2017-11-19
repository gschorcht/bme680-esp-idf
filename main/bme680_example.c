/**
 * Simple example with one sensor connected either to I2C bus 0 or
 * SPI bus 1.
 *
 * Harware configuration:
 *
 *   I2C   +-------------------------+     +----------+
 *         | ESP8266  Bus 0          |     | BME680   |
 *         |          GPIO 5 (SCL)   ------> SCL      |
 *         |          GPIO 4 (SDA)   ------- SDA      |
 *         +-------------------------+     +----------+
 *
 *         +-------------------------+     +----------+
 *         | ESP32    Bus 0          |     | BME680   |
 *         |          GPIO 16 (SCL)  >-----> SCL      |
 *         |          GPIO 17 (SDA)  ------- SDA      |
 *         +-------------------------+     +----------+
 *
 *   SPI   +-------------------------+     +----------+
 *         | ESP8266  Bus 1          |     | BME680   |
 *         |          GPIO 14 (SCK)  ------> SCK      |
 *         |          GPIO 13 (MOSI) ------> SDI      |
 *         |          GPIO 12 (MISO) <------ SDO      |
 *         |          GPIO 2  (CS)   ------> CS       |
 *         +-------------------------+     +----------+

 *         +-------------------------+     +----------+
 *         | ESP32    Bus 0          |     | BME680   |
 *         |          GPIO 16 (SCK)  ------> SCK      |
 *         |          GPIO 17 (MOSI) ------> SDI      |
 *         |          GPIO 18 (MISO) <------ SDO      |
 *         |          GPIO 19 (CS)   ------> CS       |
 *         +-------------------------+     +----------+
 */

// Uncomment to use SPI
// #define SPI_USED

/* -- platform dependent includes ----------------------------- */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp8266_wrapper.h"

#include "bme680.h"

#include <sys/time.h>

#else  // ESP8266 (esp-open-rtos)

#include <stdio.h>

#include "espressif/esp_common.h"
#include "espressif/sdk_private.h"

#include "esp/uart.h"
#include "esp/spi.h"
#include "i2c/i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bme680/bme680.h"

#endif

/** -- platform dependent definitions ------------------------------ */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth
#define TASK_STACK_DEPTH 2048

// define SPI interface for BME680 sensors
#define SPI_BUS       HSPI_HOST
#define SPI_SCK_GPIO  16
#define SPI_MOSI_GPIO 17
#define SPI_MISO_GPIO 18
#define SPI_CS_GPIO   19

// define I2C interfaces for BME680 sensors
#define I2C_BUS       0
#define I2C_SCL_PIN   16
#define I2C_SDA_PIN   17
#define I2C_FREQ      100000

#else  // ESP8266 (esp-open-rtos)

// user task stack depth
#define TASK_STACK_DEPTH 256

// define SPI interface for BME680 sensors
#define SPI_BUS       1
#define SPI_CS_GPIO   2   // GPIO 15, the default CS of SPI bus 1, can't be used

// define I2C interfaces for BME680 sensors
#define I2C_BUS       0
#define I2C_SCL_PIN   5
#define I2C_SDA_PIN   4
#define I2C_FREQ      I2C_FREQ_100K

#endif  // ESP_PLATFORM

static bme680_sensor_t* sensor = 0;

/*
 * User task that triggers measurements of sensor every seconds. It uses
 * function *vTaskDelay* to wait for measurement results. Busy wating
 * alternative is shown in comments
 */
void user_task(void *pvParameters)
{
    bme680_values_float_t values;

    TickType_t last_wakeup = xTaskGetTickCount();

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration = bme680_get_measurement_duration(sensor);

    while (1)
    {
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement (sensor))
        {
            // passive waiting until measurement results are available
            vTaskDelay (duration);

            // alternatively: busy waiting until measurement results are available
            // while (bme680_is_measuring (sensor)) ;

            // get the results and do something with them
            if (bme680_get_results_float (sensor, &values))
                printf("%.3f BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                       (double)sdk_system_get_time()*1e-3,
                       values.temperature, values.humidity,
                       values.pressure, values.gas_resistance);
        }
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
    }
}


#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)
void app_main()
#else // esp-open-rtos (ESP8266)
void user_init(void)
#endif
{
    #ifdef ESP_OPEN_RTOS  // ESP8266
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    #endif

    vTaskDelay(1);
    
    /** -- MANDATORY PART -- */

    #ifdef SPI_USED
    // Init the sensor connected to SPI.
    #ifdef ESP_OPEN_RTOS
    sensor = bme680_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    #else
    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num=SPI_MISO_GPIO,
        .mosi_io_num=SPI_MOSI_GPIO,
        .sclk_io_num=SPI_SCK_GPIO,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    if (spi_bus_initialize(SPI_BUS, &spi_bus_cfg, 1) == ESP_OK)
        sensor = bme680_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    #endif
    #else
    // Init all I2C bus interfaces at which BME680 sensors are connected
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

    // Init the sensor connected to I2C.
    sensor = bme680_init_sensor (I2C_BUS, BME680_I2C_ADDRESS_2, 0);
    #endif

    if (sensor)
    {
        // Create a task that uses the sensor
        xTaskCreate(user_task, "user_task", TASK_STACK_DEPTH, NULL, 2, NULL);

        /** -- OPTIONAL PART -- */

        // Changes the oversampling rates to 4x oversampling for temperature
        // and 2x oversampling for humidity. Pressure measurement is skipped.
        bme680_set_oversampling_rates(sensor, osr_4x, osr_none, osr_2x);

        // Change the IIR filter size for temperature and pressure to 7.
        bme680_set_filter_size(sensor, iir_size_7);

        // Change the heater profile 0 to 200 degree Celcius for 100 ms.
        bme680_set_heater_profile (sensor, 0, 200, 100);
        bme680_use_heater_profile (sensor, 0);

        // Set ambient temperature to 10 degree Celsius
        bme680_set_ambient_temperature (sensor, 10);
    }
}
