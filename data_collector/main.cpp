#include <cstdio>
#include <iostream>
#include <string.h>
#include <vector>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"

#include "driver_ina219_basic.h"
#include "dht.h"

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

#define MAX_LINE_LENGTH 100

#define MODULE_NBIOT_ENABLE true
#define MODULE_GPS_ENABLE true
#define MODULE_ENERGY_ENABLE true

using namespace std;

static ina219_handle_t power_solar;
static ina219_handle_t power_battery;

dht_t dht1;
dht_t dht2;

bool mqtt_connected = false;
bool mqtt_ready_to_send = false;
char json[1024] = {0};
char gps_data[512] = {0};
char gps_data_tmp[512] = {0};


// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void mqtt_open() {
    uart_puts(UART_NBIOT_ID, "AT+QMTOPEN=0,\"137.135.83.217\",1883\r\n");
}

void mqtt_connect() {
    uart_puts(UART_NBIOT_ID, "AT+QMTCONN=0,\"pollen-bc660\"\r\n");
}

void mqtt_publish() {
    uart_puts(UART_NBIOT_ID, "AT+QMTPUB=0,0,0,0,\"/pollen\"\r\n");
    mqtt_ready_to_send = true;
}

void mqtt_publish_data() {
    uart_puts(UART_NBIOT_ID, json);
    uart_puts(UART_NBIOT_ID, "\x1a");
    mqtt_ready_to_send = false;
}

uint8_t nbiot_rx_line[MAX_LINE_LENGTH];
uint8_t nbiot_rx_index = 0;

char gps_rx_line[MAX_LINE_LENGTH];
uint8_t gps_rx_index = 0;

vector<vector<string>> nbiot_cmds = {
    {"AT+QIDNSCFG=0,\"8.8.8.8\"\r\n", "OK"},
    {"AT+QMTOPEN=0,\"137.135.83.217\",1883\r\n", "+QMTOPEN: 0,0"},
    {"AT+QMTCONN=0,\"pollen-bc660\"\r\n", "+QMTCONN: 0,0"},
    // {"AT+QMTPUB=0,0,0,0,\"/pollen\"\r\n", ">"},
    // {"test\x1A", ""}
};
uint8_t nbiot_cmd_index = 0;
int8_t nbiot_sent_cmd_index = -1;

void nbiot_send_next_cmd() {
    if (nbiot_cmd_index >= nbiot_cmds.size()) {
        mqtt_connected = true;
        return;
    }
    cout << "Sending command: " << nbiot_cmds[nbiot_cmd_index][0] << endl;
    uart_puts(UART_NBIOT_ID, nbiot_cmds[nbiot_cmd_index][0].c_str());
    // uart_puts(UART_NBIOT_ID, "\r\n");
    nbiot_sent_cmd_index = nbiot_cmd_index;
}

// RX interrupt handler
void on_nbiot_rx() {
    while (uart_is_readable(UART_NBIOT_ID)) {
        uint8_t ch = uart_getc(UART_NBIOT_ID);

        if (ch == '\n' || ch == '\r') {
            if (!nbiot_rx_index) {
                return;
            }

            // Check for newline (end of line)
            nbiot_rx_line[nbiot_rx_index] = '\0';  // Null-terminate the string

            std::cout << "NB-IoT: " << nbiot_rx_line << std::endl;

            // if (nbiot_cmd_index > 0 && nbiot_rx_index == 2 && nbiot_rx_line[0] == 'O' && nbiot_rx_line[1] == 'K') {
            //     nbiot_send_next_cmd();
            // }

            if (strncmp((char *)nbiot_rx_line, "+CEREG: 5", 9) == 0) {
                nbiot_cmd_index = 0;
                nbiot_send_next_cmd();
            }

            if (!mqtt_connected && nbiot_sent_cmd_index > -1 && strncmp((char *)nbiot_rx_line,
                nbiot_cmds[nbiot_sent_cmd_index][1].c_str(),
                nbiot_cmds[nbiot_sent_cmd_index][1].length()) == 0)
            {
                nbiot_cmd_index++;
                nbiot_send_next_cmd();
            }

            if (mqtt_connected && strncmp((char *)nbiot_rx_line, ">", 1) == 0) {
                mqtt_publish_data();
            }

            // std::cout << line << std::endl;

            // if (strstr(reinterpret_cast<const char *>(line), "+QMTOPEN: 0,0") != nullptr) {
            //     mqtt_connect();
            // }
            //
            // if (strstr(reinterpret_cast<const char *>(line), "+QMTCONN: 0,0,0") != nullptr) {
            //     mqtt_connect();
            // }

            nbiot_rx_index = 0;
            return;
        }

        if (nbiot_rx_index < MAX_LINE_LENGTH - 1) {
            nbiot_rx_line[nbiot_rx_index++] = ch;  // Store the character in the buffer
        }
    }
}

void on_gps_rx() {
    while (uart_is_readable(UART_GPS_ID)) {
        uint8_t ch = uart_getc(UART_GPS_ID);

        if (ch == '\n' || ch == '\r') {
            if (!gps_rx_index) {
                return;
            }

            // Check for newline (end of line)
            gps_rx_line[gps_rx_index] = '\0';  // Null-terminate the string

            if (strncmp((char *)gps_rx_line, "$GPRMC", 6) == 0) {
                strcpy(gps_data_tmp, (char *)gps_rx_line);
                strcat(gps_data_tmp, "\\n");
            }
            else if (strncmp((char *)gps_rx_line, "$GPGGA", 6) == 0) {
                strcat(gps_data_tmp, (char *)gps_rx_line);
                strcpy(gps_data, gps_data_tmp);
            }

            gps_rx_index = 0;
            return;
        }

        if (gps_rx_index < MAX_LINE_LENGTH - 1) {
            gps_rx_line[gps_rx_index++] = ch;  // Store the character in the buffer
        }
    }
}

bool repeating_timer_callback(__unused struct repeating_timer *t) {
    float v_sol = 0, i_sol = 0, v_bat = 0, i_bat = 0, mW = 0;

    if (ina219_basic_read(&power_solar, &v_sol, &i_sol, &mW) != 0)
    {
        return true;
    }
    if (ina219_basic_read(&power_battery, &v_bat, &i_bat, &mW) != 0)
    {
        return true;
    }

    float t1 = 0, rh1 = 0, t2 = 0, rh2 = 0;
    dht_result_t result;
    dht_start_measurement(&dht1);
    result = dht_finish_measurement_blocking(&dht1, &rh1, &t1);
    if (result == DHT_RESULT_TIMEOUT) {
        printf("DHT1 sensor not responding. Please check your wiring.");
    }
    dht_start_measurement(&dht2);
    result = dht_finish_measurement_blocking(&dht2, &rh2, &t2);
    if (result == DHT_RESULT_TIMEOUT) {
        printf("DHT2 sensor not responding. Please check your wiring.");
    }

    uint8_t charger_chg = !gpio_get(GPIO_CHG);
    char charger_chg_str[6];
    if (charger_chg) {
        strcpy(charger_chg_str, "true");
    }
    else {
        strcpy(charger_chg_str, "false");
    }

    uint8_t charger_pgood = !gpio_get(GPIO_PGOOD);
    char charger_pgood_str[6];
    if (charger_pgood) {
        strcpy(charger_pgood_str, "true");
    }
    else {
        strcpy(charger_pgood_str, "false");
    }

    sprintf(json, R"({"dht22":[{"temperature":%.1f,"humidity":%.1f},{"temperature":%.1f,"humidity":%.1f}],"gps":"%s","power":{"Vsol":%.0f,"Isol":%.0f,"Vbat":%.0f,"Ibat":%.0f,"is_charging":%s,"pgood":%s}})", t1, rh1, t2, rh2, gps_data, v_sol, i_sol, v_bat, i_bat, charger_chg_str, charger_pgood_str);
    // cout << json << endl;

    if (mqtt_connected) {
        mqtt_publish();
    }

    return true;
}

int main() {
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    gpio_init(GPIO_POWER_GPS);
    gpio_set_dir(GPIO_POWER_GPS, GPIO_OUT);
    gpio_put(GPIO_POWER_GPS, true);

    gpio_init(GPIO_POWER_SENSORS);
    gpio_set_dir(GPIO_POWER_SENSORS, GPIO_OUT);
    gpio_put(GPIO_POWER_SENSORS, true);

    gpio_init(GPIO_PGOOD);
    gpio_set_dir(GPIO_PGOOD, GPIO_IN);
    gpio_pull_up(GPIO_PGOOD);

    gpio_init(GPIO_CHG);
    gpio_set_dir(GPIO_CHG, GPIO_IN);
    gpio_pull_up(GPIO_CHG);

    dht_init(&dht1, DHT22, pio0, GPIO_DHT1, true);
    dht_init(&dht2, DHT22, pio1, GPIO_DHT2, true);

    sleep_ms(1000);

    cout << "Hello World" << endl;

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

    sleep_ms(1000);

#if MODULE_ENERGY_ENABLE
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    // gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    cout << "INA219 init...";
    if (ina219_basic_init(&power_solar, INA219_ADDRESS_0, 0.1) != 0)
    {
        cout << "fail" << endl;
        return 1;
    }
    ina219_basic_init(&power_battery, INA219_ADDRESS_1, 0.1);
    cout << "done" << endl;

    struct repeating_timer timer;
    add_repeating_timer_ms(5000, repeating_timer_callback, NULL, &timer);
#endif

#if MODULE_NBIOT_ENABLE
    sleep_ms(1000);
    std::cout << "Resetting NB-IoT board... ";
    gpio_put(GPIO_NBIOT_RST, false);
    sleep_ms(500);
    gpio_put(GPIO_NBIOT_RST, true);
    std::cout << "done" << std::endl;

    sleep_ms(5000);

    // uart_puts(UART_NBIOT_ID, "ATE0\r\n");
    // mqtt_open();

    std::cout << "Sending ATI" << std::endl;
    uart_puts(UART_NBIOT_ID, "ATI\r\n");
#endif

    // while (true) {
    //     gpio_put(PICO_DEFAULT_LED_PIN, true);
    //     gpio_put(GPIO_POWER_GPS, true);
    //     gpio_put(GPIO_POWER_SENSORS, true);
    //     sleep_ms(10000);
    //     gpio_put(PICO_DEFAULT_LED_PIN, false);
    //     gpio_put(GPIO_POWER_GPS, false);
    //     gpio_put(GPIO_POWER_SENSORS, false);
    //     sleep_ms(10000);
    // }

    while (1)
        tight_loop_contents();

    return 0;
}
