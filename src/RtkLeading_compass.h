#ifndef HEADER_RTKLEADING_COMPASS_H_
#define HEADER_RTKLEADING_COMPASS_H_

#include <HMC5883L.h>
#include <MPU6050.h>

class Compass {
public:
    Compass();
    void setup();
    bool setupPhase1();
    bool setupPhase2();
    void setupPhase3();
    void resetOffsets();
    void loop();
    void calculate();
    Vector rotateMpuVector(Vector v) const; // to match MPU with HMC
    Vector rotateVector(Vector v) const; // to rotate all vec
    float meanAngle(float angle1, float angle2);
    float getHeading() const;
    float tiltCompensate(Vector mag, Vector normAccel);
    float correctAngle(float heading);
    void MadgwickQuaternionUpdate(float ax, float ay, float az,
        float gx, float gy, float gz,
        float mx, float my, float mz, double deltat);
    void disable();
    void enable();
    bool isEnabled() const;

    Vector const& getMag() const { return mag; }
    Vector const& getAcc() const { return acc; }
    Vector const& getGyr() const { return gyr; }

    Vector getRawMag() const { return magnetometer.getRawMag(); }
    Vector getRawAcc() const { return rotateMpuVector(mpu.getRawAccel()); }
    Vector getRawGyr() const { return rotateMpuVector(mpu.getRawGyro()); }

private:
    HMC5883L magnetometer;
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
    bool enabled;
};

extern Compass compass;

#endif /* HEADER_RTKLEADING_COMPASS_H_ */

