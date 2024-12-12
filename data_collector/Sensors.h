#ifndef SENSORS_H
#define SENSORS_H

#define GPIO_DHT1 13
#define GPIO_DHT2 15

using namespace std;

typedef struct {
    float voltage;
    float current;
    float power;
} power_data_t;

typedef struct {
    power_data_t solar;
    power_data_t battery;
    string timestamp;
} power_t;

typedef struct {
    float temperature;
    float humidity;
} environment_data_t;

typedef struct {
    environment_data_t box;
    environment_data_t outside;
} environment_t;

typedef struct {
    power_t power;
    environment_t environment;
} sensor_data_t;

class Sensors {
    ina219_handle_t power_solar{};
    ina219_handle_t power_battery{};
    dht_t dht1{};
    dht_t dht2{};
    uint dht_power_pin;

public:
    sensor_data_t sensor_data{};

    explicit Sensors(uint dht_power_pin) : dht_power_pin(dht_power_pin) {}

    void init();
    void read_power();
    void read_environment();
};

#endif //SENSORS_H
