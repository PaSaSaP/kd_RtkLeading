#ifndef HEADER_RTKLEADING_COMPASS_H_
#define HEADER_RTKLEADING_COMPASS_H_

#include <HMC5883L.h>
#include <MPU6050.h>

class Compass {
public:
    Compass();
    void setup();
    void loop();
    void calculate();
    Vector rotateVector(Vector v);
    float meanAngle(float angle1, float angle2);
    float getHeading();
    float tiltCompensate(Vector mag, Vector normAccel);
    float correctAngle(float heading);
    void MadgwickQuaternionUpdate(float ax, float ay, float az,
        float gx, float gy, float gz,
        float mx, float my, float mz, double deltat);

private:
    HMC5883L compass;
    MPU6050 mpu;
    uint32_t timer;
    uint32_t lastTimeUpdated;
    float q[4];

    float prevYaw;
    float currentYaw;
    float meanYaw;

    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle;

    Vector mag;
    Vector acc;
    Vector gyr;

    bool firstTimeSetup;
};

extern Compass compass;

#endif /* HEADER_RTKLEADING_COMPASS_H_ */

