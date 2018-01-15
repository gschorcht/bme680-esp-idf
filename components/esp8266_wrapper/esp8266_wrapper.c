/**
 * Wrapper module for source code compatibility with esp-open-rtos.
 */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

#include <sys/time.h>
#include <string.h>

#include "driver/spi_master.h"
#include "driver/spi_common.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp8266_wrapper.h"

// esp-open-rtos SDK function wrapper

uint32_t sdk_system_get_time ()
{
    struct timeval time;
    gettimeofday(&time,0);
    return time.tv_sec*1e6 + time.tv_usec;
}

bool gpio_isr_service_installed = false;
bool auto_pull_up = false;
bool auto_pull_down = true;

esp_err_t gpio_set_interrupt(gpio_num_t gpio, 
                             gpio_int_type_t type, 
                             gpio_interrupt_handler_t handler)
{
    if (!gpio_isr_service_installed)
        gpio_isr_service_installed = (gpio_install_isr_service(0) == ESP_OK);

    gpio_config_t gpio_cfg = {
       .pin_bit_mask = ((uint64_t)(((uint64_t)1)<< gpio)),
       .mode = GPIO_MODE_INPUT,
       .pull_up_en = auto_pull_up,
       .pull_down_en = auto_pull_down,
       .intr_type = type
    };
    gpio_config(&gpio_cfg);

    // set interrupt handler
    gpio_isr_handler_add(gpio, (gpio_isr_t)handler, (void*)gpio);
    
    return ESP_OK;
}

void gpio_enable (gpio_num_t gpio, const gpio_mode_t mode)
{
    gpio_config_t gpio_cfg = {
       .pin_bit_mask = ((uint64_t)(((uint64_t)1)<< gpio)),
       .mode = mode,
       .pull_up_en = auto_pull_up,
       .pull_down_en = auto_pull_down,
    };
    gpio_config(&gpio_cfg);
}

// esp-open-rtos I2C interface wrapper

#define I2C_ACK_VAL  0x0
#define I2C_NACK_VAL 0x1

void i2c_init (int bus, gpio_num_t scl, gpio_num_t sda, uint32_t freq)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    i2c_param_config(bus, &conf);
    i2c_driver_install(bus, I2C_MODE_MASTER, 0, 0, 0);
}

int i2c_slave_write (uint8_t bus, uint8_t addr, const uint8_t *reg, 
                     uint8_t *data, uint32_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    if (reg)
        i2c_master_write_byte(cmd, *reg, true);
    if (data)
        i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    return err;
}

int i2c_slave_read (uint8_t bus, uint8_t addr, const uint8_t *reg, 
                    uint8_t *data, uint32_t len)
{
    if (len == 0) return true;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (reg)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, *reg, true);
        if (!data)
            i2c_master_stop(cmd);
    }
    if (data)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_READ, true);
        if (len > 1) i2c_master_read(cmd, data, len-1, I2C_ACK_VAL);
        i2c_master_read_byte(cmd, data + len-1, I2C_NACK_VAL);
        i2c_master_stop(cmd);
    }
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

// esp-open-rtos SPI interface wrapper

#define SPI_MAX_BUS 3   // ESP32 features three SPIs (SPI_HOST, HSPI_HOST and VSPI_HOST)
#define SPI_MAX_CS  34  // GPIO 33 is the last port that can be used as output

spi_device_handle_t spi_handles[SPI_MAX_CS] = { 0 };

bool spi_bus_init (spi_host_device_t host, uint8_t sclk , uint8_t miso, uint8_t mosi)
{
    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num=miso,
        .mosi_io_num=mosi,
        .sclk_io_num=sclk,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    return (spi_bus_initialize(host, &spi_bus_cfg, 1) == ESP_OK);
}

bool spi_device_init (uint8_t bus, uint8_t cs)
{
    if (bus >= SPI_MAX_BUS || cs >= SPI_MAX_CS)
        return false;
        
    if ((spi_handles[cs] = malloc (sizeof(spi_device_handle_t))) == 0)
        return false;

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1e6,   // 1 MHz clock
        .mode = 0,               // SPI mode 0
        .spics_io_num = cs,      // CS GPIO
        .queue_size = 1,
        .flags = 0,              // no flags set
        .command_bits = 0,       // no command bits used
        .address_bits = 0,       // register address is first byte in MOSI
        .dummy_bits = 0          // no dummy bits used
    };

    if (spi_bus_add_device(bus, &dev_cfg, &(spi_handles[cs])) != ESP_OK)
    {
        free (spi_handles[cs]);
        return false;
    }
    
    return true;
}

size_t spi_transfer_pf (uint8_t bus, uint8_t cs, const uint8_t *mosi, uint8_t *miso, uint16_t len)
{
    spi_transaction_t spi_trans;

    if (cs >= SPI_MAX_CS)
        return 0;

    memset(&spi_trans, 0, sizeof(spi_trans)); // zero out spi_trans;
    spi_trans.tx_buffer = mosi;
    spi_trans.rx_buffer = miso;
    spi_trans.length=len*8;
    
    if (spi_device_transmit(spi_handles[cs], &spi_trans) != ESP_OK)
        return 0;

    return len;
}

#endif  // ESP32 (ESP-IDF)

