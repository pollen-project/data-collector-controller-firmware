#include <iostream>
#include <string>

#include <hardware/gpio.h>

#include "driver_ina219_basic.h"
#include "dht.h"

#include "Sensors.h"

#include <pico/time.h>

using namespace std;

void Sensors::init() {
    gpio_put(dht_power_pin, true);
    gpio_set_pulls(GPIO_DHT1, true, false);
    gpio_set_pulls(GPIO_DHT2, true, false);

    ina219_basic_init(&power_solar, INA219_ADDRESS_0, 0.1);
    ina219_basic_init(&power_battery, INA219_ADDRESS_1, 0.1);
    dht_init(&dht1, DHT22, pio0, GPIO_DHT1, false);
    dht_init(&dht2, DHT22, pio1, GPIO_DHT2, false);

    gpio_disable_pulls(GPIO_DHT1);
    gpio_disable_pulls(GPIO_DHT2);
    gpio_put(dht_power_pin, false);
}

void Sensors::read_power() {
    ina219_basic_read(&power_solar,
        &sensor_data.power.solar.voltage,
        &sensor_data.power.solar.current,
        &sensor_data.power.solar.power);
    ina219_basic_read(&power_battery,
        &sensor_data.power.battery.voltage,
        &sensor_data.power.battery.current,
        &sensor_data.power.battery.power);
}

void Sensors::read_environment() {
    dht_result_t result;
    int timeout = 10;

    gpio_put(dht_power_pin, true);

    gpio_set_pulls(GPIO_DHT1, true, false);
    gpio_set_pulls(GPIO_DHT2, true, false);

    sleep_ms(1000);

    do {
        sleep_ms(100);
        dht_start_measurement(&dht1);
        result = dht_finish_measurement_blocking(&dht1,
        &sensor_data.environment.box.humidity,
        &sensor_data.environment.box.temperature);
    } while (result != DHT_RESULT_OK && --timeout);

    timeout = 10;

    do {
        sleep_ms(100);
        dht_start_measurement(&dht2);
        result = dht_finish_measurement_blocking(&dht2,
        &sensor_data.environment.outside.humidity,
        &sensor_data.environment.outside.temperature);
    } while (result != DHT_RESULT_OK && --timeout);

    gpio_disable_pulls(GPIO_DHT1);
    gpio_disable_pulls(GPIO_DHT2);
    gpio_put(dht_power_pin, false);
}
