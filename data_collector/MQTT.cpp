#include <iostream>
#include <string>
#include <cstring>
#include <utility>
#include <vector>

#include <hardware/uart.h>
#include <hardware/rtc.h>
#include <pico/time.h>
#include <pico/util/datetime.h>

#include "MQTT.h"

using namespace std;

vector<vector<string>> nbiot_cmds = {
    {"AT+QSCLK=0\r\n", "OK"},
    {"AT+QIDNSCFG=0,\"8.8.8.8\"\r\n", "OK"},
    {"AT+CCLK?\r\n", "+CCLK:"},
    {"AT+CSQ\r\n", "+CSQ:"},
    // {"AT+QMTOPEN=0,\"137.135.83.217\",1883\r\n", "+QMTOPEN: 0,0"},
    // {"AT+QMTCONN=0,\"pollen-bc660\"\r\n", "+QMTCONN: 0,0"},
    // {"AT+QCFG=\"wakeupRXD\",0\r\n", "OK"},
    // {"AT+QSCLK=1\r\n", "OK"},
};

void MQTT::send_next_cmd() {
    if (mqtt_connected) {
        return;
    }
    if (cmd_index >= nbiot_cmds.size()) {
        mqtt_connected = true;
        if (on_publish_done != NULL) {
            on_publish_done(true);
        }
        return;
    }
    cout << "Sending command: " << nbiot_cmds[cmd_index][0] << endl;
    uart_puts(uart, nbiot_cmds[cmd_index][0].c_str());
    uart_tx_wait_blocking(uart);
    sent_cmd_index = cmd_index;
}

void MQTT::send_next_mqtt_cmd() {
    if (mqtt_cmd_index >= mqtt_cmds.size()) {
        if (on_publish_done != NULL) {
            on_publish_done(true);
        }
        return;
    }
    cout << "Sending command: " << mqtt_cmds[mqtt_cmd_index].cmd << endl;
    uart_puts(uart, mqtt_cmds[mqtt_cmd_index].cmd.c_str());
    uart_puts(uart, "\r\n");
    uart_tx_wait_blocking(uart);
    mqtt_sent_cmd_index = mqtt_cmd_index;
}

void MQTT::mqtt_publish_data() {
    uart_puts(uart, publish_buffer);
    uart_puts(uart, "\x1a");
    // free(publish_buffer);
    mqtt_ready_to_send = false;
}

void MQTT::publish(char *data) {
    if (on_publish_done != NULL) {
        on_publish_done(false);
    }

    publish_buffer = data;
    mqtt_ready_to_send = true;

    mqtt_cmd_index = 0;
    send_next_mqtt_cmd();
}

void MQTT::on_receive(const string& line) {
    cout << "NB-IoT: " << line << endl;

    if (line.rfind("ERROR", 0) == 0) {
        mqtt_connected = false;
        reset();
    }

    if (line.rfind("+CCLK:", 0) == 0) {
        set_rtc(line.substr(7));
    }

    if (line.rfind("+CEREG: 5", 0) == 0) {
        cmd_index = 0;
        mqtt_connected = false;
        send_next_cmd();
    }

    if (!mqtt_connected && sent_cmd_index > -1 && line.rfind(nbiot_cmds[sent_cmd_index][1]) == 0)
    {
        cmd_index++;
        send_next_cmd();
    }

    if (mqtt_sent_cmd_index > -1 && line.rfind(mqtt_cmds[mqtt_sent_cmd_index].ok_response) == 0)
    {
        mqtt_cmd_index++;
        send_next_mqtt_cmd();
    }

    if (line == ">" && mqtt_ready_to_send) {
        mqtt_publish_data();
    }
}

void MQTT::on_rx() {
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

void MQTT::reset() {
    uart_puts(uart, "AT+QRST=1\r\n");
}

void MQTT::set_rtc(string datetime) {
    datetime_t t = {
        .year  = (int16_t)stoi(datetime.substr(0,2)),
        .month = (int8_t)stoi(datetime.substr(3,2)),
        .day   = (int8_t)stoi(datetime.substr(6,2)),
        .dotw  = 0, // 0 is Sunday, so 5 is Friday
        .hour  = (int8_t)stoi(datetime.substr(9,2)),
        .min   = (int8_t)stoi(datetime.substr(12,2)),
        .sec   = (int8_t)stoi(datetime.substr(15,2)),
    };

    rtc_set_datetime(&t);
}

void MQTT::cmd(string cmd, string ok_response, void (*cb)()) {
    callback_on_resp = move(ok_response);
    callback_func = cb;
    uart_puts(uart, cmd.c_str());
    uart_puts(uart, "\r\n");
}
