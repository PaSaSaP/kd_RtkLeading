#include <Arduino.h>
#include "RtkLeading_compass.h"
#include "compass_9axis_offsets.h"
#include "RtkLeading_extern.h"

Compass compass;

Compass::Compass():
    prevYaw(0),
    currentYaw(0),
    declinationAngle((6.0 + (33.0 / 60.0)) / (180 / M_PI))
{
}

void Compass::setup() {
    while (!setupPhase1()) {
        delay(500);
    }
    while (!setupPhase2()) {
        delay(500);
    }
    setupPhase3();
}

bool Compass::setupPhase1() {
    // Initialize MPU6050
    if (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)) {
        return false;
    }
    return true;
}

bool Compass::setupPhase2() {
    // Initialize Initialize HMC5883L
    if (!magnetometer.begin()) {
        return false;
    }
    return true;
}

void Compass::setupPhase3() {
    // Set calibration offset. See HMC5883L_calibration.ino
    magnetometer.setOffset(currOffX, currOffY, currOffZ);
    magnetometer.setScale(currScaleX, currScaleY, currScaleZ);

    mpu.setAccelOffsetX(accelOffsetX);
    mpu.setAccelOffsetY(accelOffsetY);
    mpu.setAccelOffsetZ(accelOffsetZ);

    mpu.setGyroOffsetX(gyroOffsetX);
    mpu.setGyroOffsetY(gyroOffsetY);
    mpu.setGyroOffsetZ(gyroOffsetZ);
    const float gyroThreshold = 10.f;
    mpu.setManualCalibrated(gyroThreshold);

    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;

    timer = micros();
    lastTimeUpdated = RtkLeading_getCurrentTime();
    firstTimeSetup = true;
    enabled = true;
}

void Compass::resetOffsets() {
    magnetometer.resetOffsets();
    mpu.resetOffsets();
}

void Compass::loop() {
    enable();
    if (RtkLeading_getCurrentTime() - lastTimeUpdated > 25) {
        lastTimeUpdated = RtkLeading_getCurrentTime();
        calculate();
    }
}

void Compass::calculate() {
    // Read vectors
    mag = rotateVector(magnetometer.readNormalize());
    if (!magnetometer.isDataValid()) {
        RtkLeading_log("QInvalMagRead");
        return;
    }

    if (firstTimeSetup) {
        RtkLeading_log("QFIRST TIME");

        acc = rotateVector(rotateMpuVector(mpu.readScaledAccel()));

        // Calculate heading
        float heading = tiltCompensate(mag, acc);
        // RtkLeading_log("heading compensate="+String(heading));
        heading += declinationAngle;
        // RtkLeading_log("heading declination="+String(heading));

        // Correct for heading < 0deg and heading > 360deg
        heading = correctAngle(heading);
        // RtkLeading_log("heading corrected="+String(heading));

        firstTimeSetup = false;
        q[0] = cos(heading / 2);
        q[3] = cos((heading + M_PI) / 2);

        // Convert to degrees
        // heading = degrees(heading);
    }

    acc = rotateVector(rotateMpuVector(mpu.readNormalizeAccel()));
    gyr = rotateVector(rotateMpuVector(mpu.readNormalizeGyro()));

    gyr.XAxis = radians(gyr.XAxis);
    gyr.YAxis = radians(gyr.YAxis);
    gyr.ZAxis = radians(gyr.ZAxis);

    auto us = micros();
    double dt = (double)(us - timer) / 1000000; // Calculate delta time
    timer = us;

    MadgwickQuaternionUpdate(acc.XAxis, acc.YAxis, acc.ZAxis,
                            gyr.XAxis, gyr.YAxis, gyr.ZAxis,
                            mag.XAxis, mag.YAxis, mag.ZAxis, dt);

    currentYaw = atan2(-2.0f * (q[1] * q[2] + q[0] * q[3]),
                        q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    currentYaw += declinationAngle;
    currentYaw = correctAngle(currentYaw);
    currentYaw = degrees(currentYaw);
    meanYaw = meanAngle(currentYaw, prevYaw);
    prevYaw = currentYaw;

    // RtkLeading_log("delta="+String(dt));
    // RtkLeading_log("acc=("+String(acc.XAxis)+","+String(acc.YAxis)+","+String(acc.ZAxis)+"),gyr=("+String(gyr.XAxis)+","+String(gyr.YAxis)+","+String(gyr.ZAxis)+"),mag=("+String(mag.XAxis)+","+String(mag.YAxis)+","+String(mag.ZAxis)+"),q="+String(q[0])+","+String(q[1])+","+String(q[2])+","+String(q[3]));
    // RtkLeading_log("acc=("+String(acc.XAxis)+","+String(acc.YAxis)+","+String(acc.ZAxis)+")");
    // RtkLeading_log("gyr=("+String(gyr.XAxis)+","+String(gyr.YAxis)+","+String(gyr.ZAxis)+")");
    // RtkLeading_log("mag=("+String(mag.XAxis)+","+String(mag.YAxis)+","+String(mag.ZAxis)+")");
    // RtkLeading_log("q="+String(q[0])+","+String(q[1])+","+String(q[2])+","+String(q[3]));
    // RtkLeading_log("===");
}

Vector Compass::rotateMpuVector(Vector v) const {
    // accel+gyro is rotated so to match it with magnetometer so magnetometer after that is main axis system
    // acc.x = old_acc.y
    // acc.y = -old_acc.x
    auto t = v.XAxis;
    v.XAxis = v.YAxis;
    v.YAxis = -t;

    return v;
}

Vector Compass::rotateVector(Vector v) const {
    // calculations are based on axis system so magnetometer X looks backward, Y right, Z top
    // if not then that function should fix that by rotating axis
    // return v; // uncomment for calibration only

    auto temp_x = v.XAxis;
    auto temp_y = v.YAxis;
    auto temp_z = v.ZAxis;

    v.XAxis = -temp_z;
    v.YAxis = -temp_x;
    v.ZAxis = temp_y;
    return v;
}

float Compass::meanAngle(float angle1, float angle2) {
    // angle1, angle2 - in degrees
    float y_part = 0, x_part = 0;

    x_part += cos(radians(angle1));
    y_part += sin(radians(angle1));
    x_part += cos(radians(angle2));
    y_part += sin(radians(angle2));

    auto result = atan2(y_part / 2, x_part / 2);
    result = degrees(result);
    if (result < 0) {
        result += 360;
    }
    if (result >= 360) {
        result -= 360;
    }
    // RtkLeading_log("mean("+String(angle1)+","+String(angle2)+")="+String(result));
    return result;
}

float Compass::getHeading() const {
    // idiot, heading points South, so to show North add +180
    return meanYaw;
}

float Compass::tiltCompensate(Vector mag, Vector normAccel) {
    // Pitch & Roll
    float roll;
    float pitch;

    roll = asin(normAccel.YAxis);
    pitch = asin(-normAccel.XAxis);

    // RtkLeading_log("asin("+String(normAccel.YAxis)+")="+String(roll));

    if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78) {
        return -1000;
    }

    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
    float cosRoll = cos(roll);
    float sinRoll = sin(roll);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);

    // Tilt compensation
    float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
    float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

    float heading = atan2(Yh, Xh);
    return heading;
}

float Compass::correctAngle(float heading) {
    if (heading < 0) {
        heading += 2 * PI;
    }
    if (heading > 2 * PI) {
        heading -= 2 * PI;
    }
    return heading;
}

void Compass::MadgwickQuaternionUpdate(float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz, double deltat)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    //#if 0
    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    //#endif

    //#if 0
    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;
    //#endif

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    #define beta   0.9069                             // sqrt(0.75) * GyroMeasError and GyroMeasError = PI * (60.0f / 180.0f)
    //#define zeta  0.015115                           // 0.866 * GyroMeasDrift, and GyroMeasDrift = PI * (1.0f / 180.0f)
    //#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
    //#define Ki 0.0f

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void Compass::disable() {
    if (enabled) {
        mpu.setSleepEnabled(true);
        magnetometer.setMeasurementMode(HMC5883L_STANDBY);
    }
    enabled = false;
}

void Compass::enable() {
    if (!enabled) {
        mpu.setSleepEnabled(false);
        magnetometer.setMeasurementMode(HMC5883L_CONTINOUS);
        setupPhase3();
    }
    enabled = true;
}

bool Compass::isEnabled() const {
    return enabled;
}
