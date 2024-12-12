#ifndef MQTT_H
#define MQTT_H

#include <vector>

#define RX_BUF_SIZE 128

using namespace std;

typedef struct {
    string cmd;
    string ok_response;
    void (*cb)();
} mqtt_cmd_t;

class MQTT {
    uart_inst_t *uart;
    char *publish_buffer = nullptr;
    bool mqtt_ready_to_send = false;
    int rx_index = 0;
    char rx_buffer[RX_BUF_SIZE] = {0};
    int cmd_index = 0;
    int sent_cmd_index = -1;
    bool mqtt_connected = false;
    string callback_on_resp;
    void (*callback_func)();
    int mqtt_cmd_index = 0;
    int mqtt_sent_cmd_index = -1;
    void (*on_publish_done)(bool ready);
    vector<mqtt_cmd_t> mqtt_cmds = {
        // {"AT", "OK"},
        // {"AT+QSCLK=0", "OK"},
        {"AT+QMTOPEN=0,\"137.135.83.217\",1883", "+QMTOPEN: 0,0"},
        {"AT+QMTCONN=0,\"pollen-bc660\"", "+QMTCONN: 0,0"},
        {"AT+QMTPUB=0,0,0,0,\"/pollen\"", "+QMTPUB: 0,0"},
        {"AT+QMTDISC=0", "+QMTDISC: 0,0"},
        // {"AT+QMTCLOSE=0", "+QMTCLOSE: 0,0"},
        // {"AT+QSCLK=1", "OK"},
    };

    void mqtt_publish_data();
    void send_next_cmd();
    void reset();
    void set_rtc(string datetime);
    void mqtt_connect();
    void send_next_mqtt_cmd();

public:
    bool can_sleep = true;

    explicit MQTT(uart_inst_t *uart, void (*on_publish_done)(bool ready)): uart(uart), on_publish_done(on_publish_done) {}
    void publish(char *data);
    void on_receive(const string& line);
    void on_rx();
    void cmd(string cmd, string ok_response, void (*cb)());
};

#endif //MQTT_H
