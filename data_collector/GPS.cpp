#include <iostream>
#include <string>

#include <hardware/uart.h>

#include "GPS.h"

#include <vector>
#include <hardware/gpio.h>

#define GPS_TIMEOUT 60

using namespace std;

int gps_timeout = GPS_TIMEOUT;

std::vector<std::string> split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

void GPS::on_receive(const string& line) {
    if (line.rfind("$GPRMC", 0) == 0) {
        if (oneshot && !--gps_timeout) {
            oneshot = false;
            stop();

            if (on_ready != NULL) {
                on_ready();
            }

            return;
        }

        std::vector<std::string> v = split (line, ",");

        if (v.size() < 2 || v[2] != "A") {
            gps_data.clear();
            gps_valid = false;
            gps_data_ready = false;
            return;
        }

        gps_data = line;
        gps_data += "\n";
        gps_valid = true;
        gps_data_ready = false;
    }
    if (gps_valid && line.rfind("$GPGGA", 0) == 0) {
        gps_data += line;
        gps_data_ready = true;

        if (oneshot) {
            oneshot = false;
            stop();
            callback();
        }
    }
}

void GPS::on_rx() {
    while (uart_is_readable(uart)) {
        uint8_t ch = uart_getc(uart);

        if (ch == '\n' || ch == '\r') {
            if (!rx_index) {
                return;
            }

            rx_buffer[rx_index] = '\0';

            on_receive(string(rx_buffer));

            rx_index = 0;
            return;
        }

        if (rx_index < RX_BUF_SIZE - 1) {
            rx_buffer[rx_index++] = ch;
        }
    }
}

void GPS::start() {
    gpio_put(gpio, true);
    // gpio_put(PICO_DEFAULT_LED_PIN, true);
}

void GPS::stop() {
    gpio_put(gpio, false);
    // gpio_put(PICO_DEFAULT_LED_PIN, false);
}

void GPS::get_position_once(void (*cb)()) {
    callback = cb;
    oneshot = true;
    gps_timeout = GPS_TIMEOUT;
    start();
}
