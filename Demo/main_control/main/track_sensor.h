#ifndef TRACK_SENSOR_H
#define TRACK_SENSOR_H
#include "driver/gpio.h"

class TrackSensor {
public:
    static TrackSensor& getInstance();
    void init();
    void read(int &s1, int &s2, int &s3, int &s4, int &s5);
    bool isTarget();
};
#endif