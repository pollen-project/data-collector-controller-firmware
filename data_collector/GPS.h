#ifndef GPS_H
#define GPS_H

#define RX_BUF_SIZE 128

using namespace std;

class GPS {
    uart_inst_t *uart;
    uint gpio;
    int rx_index = 0;
    char rx_buffer[RX_BUF_SIZE] = {0};
    bool gps_valid = false;
    bool oneshot = false;
    void (*callback)() = nullptr;
    void (*on_ready)();

public:
    string gps_data;
    bool gps_data_ready = false;

    explicit GPS(uart_inst_t *uart, uint gpio, void (*on_ready)()): uart(uart), gpio(gpio), on_ready(on_ready) {}
    void on_receive(const string& line);
    void on_rx();
    void start();
    void stop();
    void get_position_once(void (*cb)());
};

#endif //GPS_H
