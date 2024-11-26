#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define GPIO_NBIOT_RST 2

#define UART_NBIOT_ID uart0
#define UART_NBIOT_BAUD_RATE 115200
#define UART_NBIOT_TX_PIN 8
#define UART_NBIOT_RX_PIN 9
#define UART_GPS_ID uart1
#define UART_GPS_BAUD_RATE 9600
#define UART_GPS_TX_PIN 8
#define UART_GPS_RX_PIN 9

#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define MAX_LINE_LENGTH 100

void mqtt_open() {
    uart_puts(UART_NBIOT_ID, "AT+QMTOPEN=0,\"137.135.83.217\",1883\r\n");
}

void mqtt_connect() {
    uart_puts(UART_NBIOT_ID, "AT+QMTCONN=0,\"pollen-bc660\"\r\n");
}

void mqtt_pub() {
    uart_puts(UART_NBIOT_ID, "AT+QMTPUB=0,0,0,0,\"/pollen\"\r\n");
}

// RX interrupt handler
void on_nbiot_rx() {
    uint8_t line[MAX_LINE_LENGTH];
    uint8_t index = 0;

    while (uart_is_readable(UART_GPS_ID)) {
        uint8_t ch = uart_getc(UART_GPS_ID);

        if (ch == '\n' || ch == '\r') {
            // Check for newline (end of line)
            line[index] = '\0';  // Null-terminate the string

            if (strstr(reinterpret_cast<const char *>(line), "+QMTOPEN: 0,0") != nullptr) {
                mqtt_connect();
            }

            if (strstr(reinterpret_cast<const char *>(line), "+QMTCONN: 0,0,0") != nullptr) {
                mqtt_connect();
            }

            return;
        }

        if (index < MAX_LINE_LENGTH - 1) {
            line[index++] = ch;  // Store the character in the buffer
        }
    }
}

void on_gps_rx() {
    while (uart_is_readable(UART_GPS_ID)) {
        uint8_t ch = uart_getc(UART_GPS_ID);
        // Can we send it back?
        if (uart_is_writable(UART_NBIOT_ID)) {
            uart_putc(UART_NBIOT_ID, ch);
        }
    }
}

int main() {
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
    gpio_put(GPIO_NBIOT_RST, false);
    sleep_ms(500);
    gpio_put(GPIO_NBIOT_RST, true);

    sleep_ms(5000);

    uart_puts(UART_NBIOT_ID, "ATE0\r\n");
    mqtt_open();

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
