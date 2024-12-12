#include <iostream>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include <hardware/rtc.h>
#include <hardware/structs/clocks.h>
#include <pico/runtime_init.h>
#include <pico/sleep.h>
#include <pico/util/datetime.h>

#include "driver_ina219_basic.h"
#include "dht.h"
#include "jems.h"
#include "MQTT.h"
#include "GPS.h"
#include "Sensors.h"

#define GPIO_NBIOT_RST 2
#define GPIO_POWER_GPS 3
#define GPIO_POWER_SENSORS 20
#define GPIO_DHT1 13
#define GPIO_DHT2 15
#define GPIO_PGOOD 21
#define GPIO_CHG 22

#define UART_NBIOT_ID uart0
#define UART_NBIOT_BAUD_RATE 115200
#define UART_NBIOT_TX_PIN 0
#define UART_NBIOT_RX_PIN 1
#define UART_GPS_ID uart1
#define UART_GPS_BAUD_RATE 9600
#define UART_GPS_TX_PIN 8
#define UART_GPS_RX_PIN 9

#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define MODULE_NBIOT_ENABLE true
#define MODULE_GPS_ENABLE true
#define MODULE_ENERGY_ENABLE true

#define JSON_MAX_LEVEL 10

#define POWER_AVG_COUNT 1
#define POWER_AVG_READING_COUNT 60   // 60
#define WAKE_INTERVAL_MS 10000   // 10000
#define WAKE_TIMEOUT_MS 120000
#define REPORT_INTERVAL_MS (1 * 60 * 1000)
#define GPS_INTERVAL 360     // 360

using namespace std;

repeating_timer timer;
repeating_timer power_timer;

int power_avg_count = 0;
int power_reading_count = 0;
power_t power_avg[POWER_AVG_COUNT] = {0};

string json_sensors;
string json_gps;

static jems_level_t jems_levels[JSON_MAX_LEVEL];
static jems_t jems;

int gps_interval = 2;

static bool awake;
static volatile bool mqtt_ready = false;
static volatile bool gps_ready = true;

bool power_timer_callback(__unused struct repeating_timer *t);
void sleep();
void on_nbiot_rx();
void on_gps_rx();
void send_gps_data();

MQTT mqtt(UART_NBIOT_ID, [](bool ready) {
    cout << "mqtt ready: " << ready << endl;
    mqtt_ready = ready;
});
GPS gps(UART_GPS_ID, GPIO_POWER_GPS, [] {
    cout << "gps ready: 1" << endl;
    gps_ready = true;
});
Sensors sensors(GPIO_POWER_SENSORS);

static void alarm_sleep_callback(uint alarm_id) {
    printf("alarm woke us up\n");
    uart_default_tx_wait_blocking();
    awake = true;
    hardware_alarm_set_callback(alarm_id, NULL);
    hardware_alarm_unclaim(alarm_id);
}

void sleep() {
    absolute_time_t sleep_start_time = get_absolute_time();
    cout << "waiting for mqtt & gps... " << sleep_start_time << endl;
    while (true) {  // !mqtt_ready || !gps_ready
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(100);

        if (mqtt_ready && gps_ready) {
            cout << "mqtt & gps ready" << endl;
            break;
        }

        absolute_time_t now = get_absolute_time();

        if (sleep_start_time + (WAKE_TIMEOUT_MS * 1000) < now) {
            cout << "mqtt & gps timeout" << endl;
            mqtt_ready = true;
            gps_ready = true;
            break;
        }

        tight_loop_contents();
    }
    cout << "sleeping" << endl;
    // gpio_put(PICO_DEFAULT_LED_PIN, true);
    // sleep_ms(1000);
    // gpio_put(PICO_DEFAULT_LED_PIN, false);

    gps.stop();

    int UART_NBIOT_IRQ = UART_NBIOT_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(UART_NBIOT_IRQ, false);
    irq_clear(UART_NBIOT_IRQ);
    uart_set_irq_enables(UART_NBIOT_ID, false, false);

    int UART_GPS_IRQ = UART_GPS_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(UART_GPS_IRQ, false);
    irq_clear(UART_GPS_IRQ);
    uart_set_irq_enables(UART_GPS_ID, false, false);

    sleep_run_from_xosc();
    awake = false;

    uart_default_tx_wait_blocking();
    if (sleep_goto_sleep_for(WAKE_INTERVAL_MS, &alarm_sleep_callback)) {
        while (!awake) {
            printf("Should be sleeping\n");
        }
    }

    sleep_power_up();

    UART_NBIOT_IRQ = UART_NBIOT_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_NBIOT_IRQ, on_nbiot_rx);
    irq_set_enabled(UART_NBIOT_IRQ, true);
    uart_set_irq_enables(UART_NBIOT_ID, true, false);

    UART_GPS_IRQ = UART_GPS_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_GPS_IRQ, on_gps_rx);
    irq_set_enabled(UART_GPS_IRQ, true);
    uart_set_irq_enables(UART_GPS_ID, true, false);

    if (!--gps_interval) {
        gps_ready = false;
        gps_interval = GPS_INTERVAL;
        gps.get_position_once(send_gps_data);
    }

    power_timer_callback(NULL);

    sleep();
}

void on_nbiot_rx() {
    mqtt.on_rx();
}

void on_gps_rx() {
    gps.on_rx();
}

static void write_char(char ch, uintptr_t arg) {
    *reinterpret_cast<string *>(arg) += ch;
}

void send_data() {
    uint8_t charger_chg = !gpio_get(GPIO_CHG);
    uint8_t charger_pgood = !gpio_get(GPIO_PGOOD);
    power_t *pavg = &power_avg[0];

    sensors.read_environment();

    json_sensors.clear();
    jems_init(&jems, jems_levels, JSON_MAX_LEVEL, write_char, reinterpret_cast<uintptr_t>(&json_sensors));

    jems_object_open(&jems);            // {
    jems_string(&jems, "dht22");   //   "dht22"
    jems_array_open(&jems);             //     [

    jems_object_open(&jems);
    jems_string(&jems, "t");
    jems_number(&jems, sensors.sensor_data.environment.box.temperature);
    jems_string(&jems, "rh");
    jems_number(&jems, sensors.sensor_data.environment.box.humidity);
    jems_object_close(&jems);

    jems_object_open(&jems);
    jems_string(&jems, "t");
    jems_number(&jems, sensors.sensor_data.environment.outside.temperature);
    jems_string(&jems, "rh");
    jems_number(&jems, sensors.sensor_data.environment.outside.humidity);
    jems_object_close(&jems);

    jems_array_close(&jems);            //      ],

    jems_string(&jems, "power");   //   "power"
    jems_object_open(&jems);             //     {
    jems_string(&jems, "Vsol");
    // jems_integer(&jems, static_cast<int>(sensors.sensor_data.power.solar.voltage));
    jems_integer(&jems, static_cast<int>(pavg->solar.voltage));
    jems_string(&jems, "Isol");
    // jems_integer(&jems, static_cast<int>(sensors.sensor_data.power.solar.current));
    jems_integer(&jems, static_cast<int>(pavg->solar.current));
    jems_string(&jems, "Vbat");
    // jems_integer(&jems, static_cast<int>(sensors.sensor_data.power.battery.voltage));
    jems_integer(&jems, static_cast<int>(pavg->battery.voltage));
    jems_string(&jems, "Ibat");
    // jems_integer(&jems, static_cast<int>(sensors.sensor_data.power.battery.current));
    jems_integer(&jems, static_cast<int>(pavg->battery.current));
    jems_string(&jems, "is_charging");
    jems_bool(&jems, charger_chg);
    jems_string(&jems, "pgood");
    jems_bool(&jems, charger_pgood);
    jems_object_close(&jems);               //    }

    jems_object_close(&jems);     // }

    mqtt.publish(json_sensors.data());
}

void send_gps_data() {
    if (!gps.gps_data_ready) {
        gps_ready = true;
        return;
    }

    mqtt_ready = false;

    json_gps.clear();
    jems_init(&jems, jems_levels, JSON_MAX_LEVEL, write_char, reinterpret_cast<uintptr_t>(&json_gps));

    jems_object_open(&jems);          // {

    jems_string(&jems, "gps");   //   "gps"
    jems_string(&jems, gps.gps_data.c_str());

    jems_object_close(&jems);          // }

    // json_gps = R"({"gps":""})";
    mqtt.publish(json_gps.data());

    gps.gps_data_ready = false;
    gps_ready = true;
}

// void send_power_data() {
//     json_sensors.clear();
//     jems_init(&jems, jems_levels, JSON_MAX_LEVEL, write_char, reinterpret_cast<uintptr_t>(&json_sensors));
//
//     jems_object_open(&jems);            // {
//     jems_string(&jems, "power_history");   //   "dht22"
//     jems_array_open(&jems);             //     [
//
//     for (int i = 0; i < power_avg_count; i++) {
//         power_t *pavg = &power_avg[i];
//
//         jems_object_open(&jems);             //     {
//         jems_string(&jems, "timestamp");
//         jems_string(&jems, pavg->timestamp.c_str());
//         jems_string(&jems, "Vsol");
//         jems_integer(&jems, static_cast<int>(pavg->solar.voltage));
//         jems_string(&jems, "Isol");
//         jems_integer(&jems, static_cast<int>(pavg->solar.current));
//         jems_string(&jems, "Vbat");
//         jems_integer(&jems, static_cast<int>(pavg->battery.voltage));
//         jems_string(&jems, "Ibat");
//         jems_integer(&jems, static_cast<int>(pavg->battery.current));
//         jems_object_close(&jems);
//     }
//
//     jems_array_close(&jems);            //      ],
//
//     jems_object_close(&jems);     // }
//
//     mqtt.publish(json_sensors.data());
// }

// bool repeating_timer_callback(__unused struct repeating_timer *t) {
//     sensors.read_power();
//     sensors.read_environment();
//
//     if (!--gps_interval) {
//         gps_interval = GPS_INTERVAL;
//         gps.get_position_once(send_gps_data);
//     }
//
//     send_data();
//
//     return true;
// }

bool power_timer_callback(__unused struct repeating_timer *t) {
    sensors.read_power();
    //sensors.read_environment();

    power_t *pavg = &power_avg[power_avg_count];

    pavg->battery.voltage += sensors.sensor_data.power.battery.voltage;
    pavg->battery.current += sensors.sensor_data.power.battery.current;
    pavg->solar.voltage += sensors.sensor_data.power.solar.voltage;
    pavg->solar.current += sensors.sensor_data.power.solar.current;

    power_reading_count++;

    if (power_reading_count >= POWER_AVG_READING_COUNT) {
        pavg->battery.voltage /= (float)POWER_AVG_READING_COUNT;
        pavg->battery.current /= (float)POWER_AVG_READING_COUNT;
        pavg->solar.voltage /= (float)POWER_AVG_READING_COUNT;
        pavg->solar.current /= (float)POWER_AVG_READING_COUNT;

        datetime_t t;
        char datetime_buf[32];
        rtc_get_datetime(&t);
        sprintf(datetime_buf, "20%02d-%02d-%02dT%02d:%02d:%02d", t.year, t.month, t.day, t.hour, t.min, t.sec);
        pavg->timestamp = datetime_buf;

        cout << pavg->timestamp << " - " << pavg->battery.current << endl;

        power_reading_count = 0;
        power_avg_count++;
    }

    if (power_avg_count >= POWER_AVG_COUNT) {
        mqtt_ready = false;
        send_data();

        for (int i = 0; i < POWER_AVG_COUNT; i++) {
            power_avg[i].battery.voltage = 0;
            power_avg[i].battery.current = 0;
            power_avg[i].solar.voltage = 0;
            power_avg[i].solar.current = 0;
        }

        power_avg_count = 0;
        power_reading_count = 0;
    }

    return true;
}

int main() {
    stdio_init_all();
    rtc_init();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    gpio_init(GPIO_POWER_GPS);
    gpio_set_dir(GPIO_POWER_GPS, GPIO_OUT);
    gpio_put(GPIO_POWER_GPS, false);

    gpio_init(GPIO_POWER_SENSORS);
    gpio_set_dir(GPIO_POWER_SENSORS, GPIO_OUT);
    gpio_put(GPIO_POWER_SENSORS, false);

    gpio_init(GPIO_PGOOD);
    gpio_set_dir(GPIO_PGOOD, GPIO_IN);
    gpio_pull_up(GPIO_PGOOD);

    gpio_init(GPIO_CHG);
    gpio_set_dir(GPIO_CHG, GPIO_IN);
    gpio_pull_up(GPIO_CHG);

    sleep_ms(1000);

    cout << "Hello World" << endl;
    gpio_put(PICO_DEFAULT_LED_PIN, false);

#if MODULE_NBIOT_ENABLE
    gpio_init(GPIO_NBIOT_RST);
    gpio_set_dir(GPIO_NBIOT_RST, GPIO_OUT);
    gpio_put(GPIO_NBIOT_RST, true);

    uart_init(UART_NBIOT_ID, UART_NBIOT_BAUD_RATE);
    gpio_set_function(UART_NBIOT_TX_PIN, UART_FUNCSEL_NUM(UART_NBIOT_ID, UART_TX_PIN));
    gpio_set_function(UART_NBIOT_RX_PIN, UART_FUNCSEL_NUM(UART_NBIOT_ID, UART_RX_PIN));
    uart_set_hw_flow(UART_NBIOT_ID, false, false);
    uart_set_format(UART_NBIOT_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_NBIOT_ID, false);
#endif

#if MODULE_GPS_ENABLE
    uart_init(UART_GPS_ID, UART_GPS_BAUD_RATE);
    gpio_set_function(UART_GPS_TX_PIN, UART_FUNCSEL_NUM(UART_GPS_ID, UART_TX_PIN));
    gpio_set_function(UART_GPS_RX_PIN, UART_FUNCSEL_NUM(UART_GPS_ID, UART_RX_PIN));
    uart_set_hw_flow(UART_GPS_ID, false, false);
    uart_set_format(UART_GPS_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_GPS_ID, false);
#endif

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
#if MODULE_NBIOT_ENABLE
    int UART_NBIOT_IRQ = UART_NBIOT_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_NBIOT_IRQ, on_nbiot_rx);
    irq_set_enabled(UART_NBIOT_IRQ, true);
    uart_set_irq_enables(UART_NBIOT_ID, true, false);
#endif

#if MODULE_GPS_ENABLE
    int UART_GPS_IRQ = UART_GPS_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_GPS_IRQ, on_gps_rx);
    irq_set_enabled(UART_GPS_IRQ, true);
    uart_set_irq_enables(UART_GPS_ID, true, false);
#endif

#if MODULE_ENERGY_ENABLE
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    sensors.init();

    // add_repeating_timer_ms(REPORT_INTERVAL_MS, repeating_timer_callback, nullptr, &timer);
    // add_repeating_timer_ms(POWER_INTERVAL_MS, power_timer_callback, nullptr, &power_timer);
#endif

    std::cout << "Resetting NB-IoT board... ";
    gpio_put(GPIO_NBIOT_RST, false);
    sleep_ms(500);
    gpio_put(GPIO_NBIOT_RST, true);
    std::cout << "done" << std::endl;

    sleep();

    while (1)
        tight_loop_contents();

    return 0;
}
