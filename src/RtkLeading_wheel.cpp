#include <Arduino.h>
#include "RtkLeading_wheel.h"
#include "RtkLeading_compass.h"
#include "RtkLeading_def.h"
#include "RtkLeading_extern.h"
#include <TinyGPS++.h>

Pos currentPosition;
Wheel wheel;

Wheel::Wheel() {
}

void Wheel::loop() {
    calculateBearing();
    distanceBetweenMidAndLine = distanceFromPoint(midPos);
    distanceBetweenCurPosAndLine = distanceFromPoint(currentPosition);
    // RtkLeading_logDouble("dist_mid", distanceBetweenMidAndLine, 3);
    // RtkLeading_logDouble("dist_cur", distanceBetweenCurPosAndLine, 3);
    // RtkLeading_log("distanceFromMid="+String(distanceBetweenMidAndLine, 3));
    // RtkLeading_log("distanceFromCur="+String(distanceBetweenCurPosAndLine, 3));
    distanceDiff = distanceBetweenCurPosAndLine - distanceBetweenMidAndLine;

    calculatedBearing = expectedBearing;
    auto const compassHeading = compass.getHeading();
    compassBearing = bearingNorm(compassHeading + 180); // to make it point north

    auto currentDiff = compassBearing - calculatedBearing;
    if (currentDiff < -180) {
        currentDiff += 360;
    } else if (currentDiff > 180) {
        currentDiff -= 360;
    }

    movingToSouth = false;
    // calculatedWheelDegrees = wheelDegrees; // rotate right when moving up
    if (currentDiff < -90 || currentDiff > 90) {
        expectedBearing = bearingNorm(expectedBearing + 180);
        calculatedBearing = expectedBearing;
        currentDiff = compassBearing - calculatedBearing;
        // calculatedWheelDegrees = -calculatedWheelDegrees; // rotate left when moving down
        distanceDiff = -distanceDiff;
        movingToSouth = true;
    }

    if (distanceDiff < -histeresis) {
        // wheel into right
        calculatedBearing += wheelDegrees;
        currentDiff -= wheelDegrees;
    } else if (distanceDiff > histeresis) {
        // wheel into left
        calculatedBearing -= wheelDegrees;
        currentDiff += wheelDegrees;
    } else {
    }
    if (currentDiff < -180) {
        currentDiff += 360;
    } else if (currentDiff > 180) {
        currentDiff -= 360;
    }

    calculatedBearing = bearingNorm(calculatedBearing);
    // RtkLeading_logDouble("comp_exp", calculatedBearing, 2);

    auto newWheelValue = wheel;
    if (currentDiff < -wheelHisteresis) {
        newWheelValue = 1;
    } else if (currentDiff > wheelHisteresis) {
        newWheelValue = -1;
    } else {
        newWheelValue = 0;
    }
    updateWheel(newWheelValue);
}

void Wheel::updateWheel(int8_t newWheelValue) {
    if (newWheelValue != wheel) {
        RtkLeading_log("change wheel "+String(wheel)+" -> "+String(newWheelValue));
        wheel = newWheelValue;
        RtkLeading_logInt("wheel", wheel);
    }
}

int8_t Wheel::getWheelState() const {
    return wheel;
}

void Wheel::setStartPos(double lat, double lon) {
    startPos.lat = lat;
    startPos.lon = lon;
}

void Wheel::setEndPos(double lat, double lon) {
    endPos.lat = lat;
    endPos.lon = lon;
}

void Wheel::setMidPos(double lat, double lon) {
    midPos.lat = lat;
    midPos.lon = lon;
}

void Wheel::setStartPos(Pos const& p) {
    startPos = p;
}

Pos const& Wheel::getStartPos() const {
    return startPos;
}

void Wheel::setEndPos(Pos const& p) {
    endPos = p;
}

Pos const& Wheel::getEndPos() const {
    return endPos;
}

void Wheel::setMidPos(Pos const& p) {
    midPos = p;
}

Pos const& Wheel::getMidPos() const {
    return midPos;
}

void Wheel::moveMidBy(double m) {
    auto bearing = expectedBearing;  // clockwise from north
    bearing += movingToSouth? -90: 90;
    m /= EARTH_R; // divide by Earths R

    Pos p = midPos;
    p.lat = radians(p.lat);
    p.lon = radians(p.lon);
    bearing = radians(bearing);

    auto slat = sin(p.lat);
    auto clat = cos(p.lat);
    auto sbear = sin(bearing);
    auto cbear = cos(bearing);
    auto sm = sin(m);
    auto cm = cos(m);

    midPos.lat = asin(slat * cm + clat * sm * cbear);
    midPos.lon = p.lon + atan2(sbear * sm * clat, cm - slat * sin(midPos.lat));

    midPos.lat = degrees(midPos.lat);
    midPos.lon = degrees(midPos.lon);
}

void Wheel::calculateBearing() {
    expectedBearing = TinyGPSPlus::courseTo(startPos.lat, startPos.lon, endPos.lat, endPos.lon);
}

double Wheel::distanceFromPoint(Pos const& p) const {
    auto bearingAC = TinyGPSPlus::courseTo(startPos.lat, startPos.lon, p.lat, p.lon);
    auto distanceAC = TinyGPSPlus::distanceBetween(startPos.lat, startPos.lon, p.lat, p.lon);
    auto rbearingAB = radians(expectedBearing);
    auto rbearingAC = radians(bearingAC);
    auto distance = asin(sin(distanceAC / EARTH_R) * sin(rbearingAC - rbearingAB)) * EARTH_R;
    return distance;
}

double Wheel::bearingNorm(double bearing) {
    if (bearing >= 360.0) {
        bearing -= 360.0;
    }
    return bearing;
}

