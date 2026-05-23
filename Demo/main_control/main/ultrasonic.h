#ifndef ULTRASONIC_H
#define ULTRASONIC_H
class Ultrasonic {
public:
    static Ultrasonic& getInstance();
    void init();
    int getDistance();
    bool isObstacle(int limit = 15);
};
#endif