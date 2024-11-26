#include <cstdio>
#include <iostream>
#include <string.h>
#include <vector>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define GPIO_NBIOT_RST 2

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

using namespace std;

void mqtt_open() {
    uart_puts(UART_NBIOT_ID, "AT+QMTOPEN=0,\"137.135.83.217\",1883\r\n");
}

void mqtt_connect() {
    uart_puts(UART_NBIOT_ID, "AT+QMTCONN=0,\"pollen-bc660\"\r\n");
}

void mqtt_pub() {
    uart_puts(UART_NBIOT_ID, "AT+QMTPUB=0,0,0,0,\"/pollen\"\r\n");
}

uint8_t nbiot_rx_line[MAX_LINE_LENGTH];
uint8_t nbiot_rx_index = 0;

vector<vector<string>> nbiot_cmds = {
    {"AT+QIDNSCFG=0,\"8.8.8.8\"\r\n", "OK"},
    {"AT+QMTOPEN=0,\"137.135.83.217\",1883\r\n", "+QMTOPEN: 0,0"},
    {"AT+QMTCONN=0,\"pollen-bc660\"\r\n", "+QMTCONN: 0,0"},
    {"AT+QMTPUB=0,0,0,0,\"/pollen\"\r\n", ">"},
    {"test\u001A", ""}
};
uint8_t nbiot_cmd_index = 0;
int8_t nbiot_sent_cmd_index = -1;

void nbiot_send_next_cmd() {
    if (nbiot_cmd_index >= nbiot_cmds.size()) {
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

            std::cout << "Received line (" << static_cast<int>(nbiot_rx_index) << "): " << nbiot_rx_line << std::endl;

            // if (nbiot_cmd_index > 0 && nbiot_rx_index == 2 && nbiot_rx_line[0] == 'O' && nbiot_rx_line[1] == 'K') {
            //     nbiot_send_next_cmd();
            // }

            if (strncmp((char *)nbiot_rx_line, "+CEREG: 5", 9) == 0) {
                nbiot_cmd_index = 0;
                nbiot_send_next_cmd();
            }

            if (nbiot_sent_cmd_index > -1 && strncmp((char *)nbiot_rx_line,
                nbiot_cmds[nbiot_sent_cmd_index][1].c_str(),
                nbiot_cmds[nbiot_sent_cmd_index][1].length()) == 0)
            {
                nbiot_cmd_index++;
                nbiot_send_next_cmd();
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
        std::cout << ch;
        // Can we send it back?
        // if (uart_is_writable(UART_NBIOT_ID)) {
        //     uart_putc(UART_NBIOT_ID, ch);
        // }
    }
}

int main() {
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    sleep_ms(1000);

    std::cout << "Hello World" << std::endl;

    gpio_init(GPIO_NBIOT_RST);
    gpio_set_dir(GPIO_NBIOT_RST, GPIO_OUT);
    gpio_put(GPIO_NBIOT_RST, true);

    uart_init(UART_NBIOT_ID, UART_NBIOT_BAUD_RATE);
    gpio_set_function(UART_NBIOT_TX_PIN, UART_FUNCSEL_NUM(UART_NBIOT_ID, UART_TX_PIN));
    gpio_set_function(UART_NBIOT_RX_PIN, UART_FUNCSEL_NUM(UART_NBIOT_ID, UART_RX_PIN));
    uart_set_hw_flow(UART_NBIOT_ID, false, false);
    uart_set_format(UART_NBIOT_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_NBIOT_ID, false);

    uart_init(UART_GPS_ID, UART_GPS_BAUD_RATE);
    gpio_set_function(UART_GPS_TX_PIN, UART_FUNCSEL_NUM(UART_GPS_ID, UART_TX_PIN));
    gpio_set_function(UART_GPS_RX_PIN, UART_FUNCSEL_NUM(UART_GPS_ID, UART_RX_PIN));
    uart_set_hw_flow(UART_GPS_ID, false, false);
    uart_set_format(UART_GPS_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_GPS_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_NBIOT_IRQ = UART_NBIOT_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_NBIOT_IRQ, on_nbiot_rx);
    irq_set_enabled(UART_NBIOT_IRQ, true);
    uart_set_irq_enables(UART_NBIOT_ID, true, false);

    int UART_GPS_IRQ = UART_GPS_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_GPS_IRQ, on_gps_rx);
    irq_set_enabled(UART_GPS_IRQ, true);
    uart_set_irq_enables(UART_GPS_ID, true, false);

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

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
    // while (true) {
    //     uart_puts(uart0, "Hello, World!\n");
    //     //uart_puts(UART_ID, "ATI\r\n");
    //     sleep_ms(1000);
    // }

    while (1)
        tight_loop_contents();

    return 0;
}
