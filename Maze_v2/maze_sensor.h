#ifndef SENSOR_H
#define SENSOR_H



#include <EEPROM.h>
#include <Wire.h>
#include <VL53L0X.h>

class Maze_Sensor {
public:
    Maze_Sensor(uint8_t left_xshut, uint8_t right_xshut, uint8_t front_xshut) {
        _left_xshut = left_xshut;
        _right_xshut = right_xshut;
        _front_xshut = front_xshut;
    }

    void begin() {
        Wire.begin();
        Wire.setClock(400000);

        pinMode(_left_xshut, OUTPUT);
        pinMode(_right_xshut, OUTPUT);
        pinMode(_front_xshut, OUTPUT);

        // Turn all sensors off
        digitalWrite(_left_xshut, LOW);
        digitalWrite(_right_xshut, LOW);
        digitalWrite(_front_xshut, LOW);
        delay(50);

        // Initialize Left
        digitalWrite(_left_xshut, HIGH);
        delay(50);
        left_sensor.setTimeout(500);
        if (left_sensor.init()) {
            left_sensor.setAddress(42);
            configureSensor(left_sensor);
        } else {
            Serial.println("Left sensor failed to initialize.");
        }

        // Initialize Right
        digitalWrite(_right_xshut, HIGH);
        delay(50);
        right_sensor.setTimeout(500);
        if (right_sensor.init()) {
            right_sensor.setAddress(43);
            configureSensor(right_sensor);
        } else {
            Serial.println("Right sensor failed to initialize.");
        }

        // Initialize Front
        digitalWrite(_front_xshut, HIGH);
        delay(50);
        front_sensor.setTimeout(500);
        if (front_sensor.init()) {
            front_sensor.setAddress(44);
            configureSensor(front_sensor);
        } else {
            Serial.println("Front sensor failed to initialize.");
        }


        // Load thresholds from EEPROM
        EEPROM.get(left_add, left_threshold);
        EEPROM.get(right_add, right_threshold);
        EEPROM.get(front_add, front_threshold);
    }

    void update() {
        left_distance = left_sensor.readRangeContinuousMillimeters()/10;
        right_distance = right_sensor.readRangeContinuousMillimeters()/10;
        front_distance = front_sensor.readRangeContinuousMillimeters()/10;
    }

    bool is_left_empty(){
        return left_distance > left_threshold;
    }
    bool is_front_empty(){
        return front_distance > front_threshold;
    }
    bool is_right_empty(){
        return right_distance > right_threshold;
    }

    void update_threshold() {
        left_threshold = left_distance+10;
        right_threshold = right_distance+10;
        front_threshold = front_distance+10;

        left_threshold = 10;
        right_threshold = 10;

        Serial.println(left_threshold);
        Serial.println(right_threshold);
        EEPROM.put(left_add, left_threshold);
        EEPROM.put(right_add, right_threshold);
        EEPROM.put(front_add, front_threshold);
    }


    int left_distance = 0, right_distance = 0, front_distance=0;
    int left_threshold = 0, right_threshold = 0, front_threshold=0;

private:
    int left_add=5, right_add=6, front_add=7;

    VL53L0X left_sensor, right_sensor, front_sensor;
    uint8_t _left_xshut, _right_xshut, _front_xshut;

    void configureSensor(VL53L0X &sensor) {
        sensor.setSignalRateLimit(0.1);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
        sensor.setMeasurementTimingBudget(20000);
        sensor.startContinuous(40);
    }
};



#endif