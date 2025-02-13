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

// Hardware IO pins
#define GPIO_NBIOT_RST 2        // NB-IoT module reset
#define GPIO_POWER_GPS 3        // 5v power on/off
#define GPIO_POWER_SENSORS 20   // 3.3v power on/off
#define GPIO_DHT1 13            // Inside sensor data input
#define GPIO_DHT2 15            // Outside sensor data input
#define GPIO_PGOOD 21           // Charge controller "power good" signal
#define GPIO_CHG 22             // Charge controller "is charging" signal

// Hardware UART config for the NB-IoT and GPS modules
#define UART_NBIOT_ID uart0
#define UART_NBIOT_BAUD_RATE 115200
#define UART_NBIOT_TX_PIN 0
#define UART_NBIOT_RX_PIN 1
#define UART_GPS_ID uart1
#define UART_GPS_BAUD_RATE 9600
#define UART_GPS_TX_PIN 8
#define UART_GPS_RX_PIN 9

// Generic UART config
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// Jems JSON library config
#define JSON_MAX_LEVEL 10

// Enable/disable parts of the firmware
#define MODULE_NBIOT_ENABLE true
#define MODULE_GPS_ENABLE true
#define MODULE_ENERGY_ENABLE true

// Measurement intervals
#define POWER_AVG_COUNT 1
#define POWER_AVG_READING_COUNT 60
#define WAKE_INTERVAL_MS 10000
#define WAKE_TIMEOUT_MS 120000
#define REPORT_INTERVAL_MS (1 * 60 * 1000)
#define GPS_INTERVAL 8640     // 1h: 360; 24h: 8640

using namespace std;

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

bool do_measurements();
void sleep();
void on_nbiot_rx();
void on_gps_rx();
void send_gps_data();

// Initialize MQTT, GPS, and Sensors modules
// MQTT and GPS have callbacks for when they are ready to sleep
MQTT mqtt(UART_NBIOT_ID, [](bool ready) {
    cout << "mqtt ready: " << ready << endl;
    mqtt_ready = ready;
});
GPS gps(UART_GPS_ID, GPIO_POWER_GPS, [] {
    cout << "gps ready: 1" << endl;
    gps_ready = true;
});
Sensors sensors(GPIO_POWER_SENSORS);

// Handle waking from sleep mode
static void alarm_sleep_callback(uint alarm_id) {
    printf("alarm woke us up\n");
    uart_default_tx_wait_blocking();
    awake = true;
    hardware_alarm_set_callback(alarm_id, NULL);
    hardware_alarm_unclaim(alarm_id);
}

// Wait for all modules to be ready and go to sleep
void sleep() {
    absolute_time_t sleep_start_time = get_absolute_time();
    cout << "waiting for mqtt & gps... " << sleep_start_time << endl;
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(100);

        if (mqtt_ready && gps_ready) {
            cout << "mqtt & gps ready" << endl;
            break;
        }

        // Stop waiting after WAKE_TIMEOUT_MS has passed
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

    // Turn off GPS
    gps.stop();

    // Disable UART RX interrupts to prevent unwanted wake
    int UART_NBIOT_IRQ = UART_NBIOT_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(UART_NBIOT_IRQ, false);
    irq_clear(UART_NBIOT_IRQ);
    uart_set_irq_enables(UART_NBIOT_ID, false, false);

    int UART_GPS_IRQ = UART_GPS_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(UART_GPS_IRQ, false);
    irq_clear(UART_GPS_IRQ);
    uart_set_irq_enables(UART_GPS_ID, false, false);

    // Run from external oscillator
    sleep_run_from_xosc();
    awake = false;

    // Put CPU to sleep
    uart_default_tx_wait_blocking();
    if (sleep_goto_sleep_for(WAKE_INTERVAL_MS, &alarm_sleep_callback)) {
        while (!awake) {
            printf("Should be sleeping\n");
        }
    }

    // Restore clocks after wake
    sleep_power_up();

    // Enable UART RX interrupts
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

    do_measurements();

    // Start next sleep cycle
    sleep();
}

// UART RX handler
void on_nbiot_rx() {
    mqtt.on_rx();
}

// UART RX handler
void on_gps_rx() {
    gps.on_rx();
}

// JEMS JSON helper
static void write_char(char ch, uintptr_t arg) {
    *reinterpret_cast<string *>(arg) += ch;
}

void send_data() {
    // Read power values
    uint8_t charger_chg = !gpio_get(GPIO_CHG);
    uint8_t charger_pgood = !gpio_get(GPIO_PGOOD);
    power_t *pavg = &power_avg[0];

    // Read sensors
    sensors.read_environment();

    // Construct a JSON object
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
    jems_integer(&jems, static_cast<int>(pavg->solar.voltage));
    jems_string(&jems, "Isol");
    jems_integer(&jems, static_cast<int>(pavg->solar.current));
    jems_string(&jems, "Vbat");
    jems_integer(&jems, static_cast<int>(pavg->battery.voltage));
    jems_string(&jems, "Ibat");
    jems_integer(&jems, static_cast<int>(pavg->battery.current));
    jems_string(&jems, "is_charging");
    jems_bool(&jems, charger_chg);
    jems_string(&jems, "pgood");
    jems_bool(&jems, charger_pgood);
    jems_object_close(&jems);               //    }

    jems_object_close(&jems);     // }

    // Send JSON object via MQTT
    mqtt.publish(json_sensors.data());
}

void send_gps_data() {
    if (!gps.gps_data_ready) {
        gps_ready = true;
        return;
    }

    mqtt_ready = false;

    // Construct a JSON object
    json_gps.clear();
    jems_init(&jems, jems_levels, JSON_MAX_LEVEL, write_char, reinterpret_cast<uintptr_t>(&json_gps));

    jems_object_open(&jems);          // {

    jems_string(&jems, "gps");   //   "gps"
    jems_string(&jems, gps.gps_data.c_str());

    jems_object_close(&jems);          // }

    // Send JSON via MQTT
    mqtt.publish(json_gps.data());

    gps.gps_data_ready = false;
    gps_ready = true;
}

bool do_measurements() {
    sensors.read_power();

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
    // Initialize microcontroller hardware
    stdio_init_all();
    rtc_init();

    // Status LED pin as output and turn it on
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    // 5v power control pin as output and turn it off
    gpio_init(GPIO_POWER_GPS);
    gpio_set_dir(GPIO_POWER_GPS, GPIO_OUT);
    gpio_put(GPIO_POWER_GPS, false);

    // 3.3v power control pin as output and turn it off
    gpio_init(GPIO_POWER_SENSORS);
    gpio_set_dir(GPIO_POWER_SENSORS, GPIO_OUT);
    gpio_put(GPIO_POWER_SENSORS, false);

    // Charger "power good" signal pin as input with pull-up
    gpio_init(GPIO_PGOOD);
    gpio_set_dir(GPIO_PGOOD, GPIO_IN);
    gpio_pull_up(GPIO_PGOOD);

    // Charger "is charging" signal pin as input with pull-up
    gpio_init(GPIO_CHG);
    gpio_set_dir(GPIO_CHG, GPIO_IN);
    gpio_pull_up(GPIO_CHG);

    sleep_ms(1000);

    cout << "Hello World" << endl;
    gpio_put(PICO_DEFAULT_LED_PIN, false);

#if MODULE_NBIOT_ENABLE
    // Set NB-IoT module reset pin as output
    gpio_init(GPIO_NBIOT_RST);
    gpio_set_dir(GPIO_NBIOT_RST, GPIO_OUT);
    gpio_put(GPIO_NBIOT_RST, true);

    // Initialize UART for NB-IoT
    uart_init(UART_NBIOT_ID, UART_NBIOT_BAUD_RATE);
    gpio_set_function(UART_NBIOT_TX_PIN, UART_FUNCSEL_NUM(UART_NBIOT_ID, UART_TX_PIN));
    gpio_set_function(UART_NBIOT_RX_PIN, UART_FUNCSEL_NUM(UART_NBIOT_ID, UART_RX_PIN));
    uart_set_hw_flow(UART_NBIOT_ID, false, false);
    uart_set_format(UART_NBIOT_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_NBIOT_ID, false);

    // UART RX interrupt handler
    int UART_NBIOT_IRQ = UART_NBIOT_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_NBIOT_IRQ, on_nbiot_rx);
    irq_set_enabled(UART_NBIOT_IRQ, true);
    uart_set_irq_enables(UART_NBIOT_ID, true, false);
#endif

#if MODULE_GPS_ENABLE
    // Initialize UART for GPS
    uart_init(UART_GPS_ID, UART_GPS_BAUD_RATE);
    gpio_set_function(UART_GPS_TX_PIN, UART_FUNCSEL_NUM(UART_GPS_ID, UART_TX_PIN));
    gpio_set_function(UART_GPS_RX_PIN, UART_FUNCSEL_NUM(UART_GPS_ID, UART_RX_PIN));
    uart_set_hw_flow(UART_GPS_ID, false, false);
    uart_set_format(UART_GPS_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_GPS_ID, false);

    // UART RX interrupt handler
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
#endif

    // The NB-IoT module needs a reset signal after power on
    std::cout << "Resetting NB-IoT board... ";
    gpio_put(GPIO_NBIOT_RST, false);
    sleep_ms(500);
    gpio_put(GPIO_NBIOT_RST, true);
    std::cout << "done" << std::endl;

    // Start the main sleep-wake cycle
    sleep();

    // Infinite loop to keep the main function busy
    while (1)
        tight_loop_contents();

    return 0;
}
