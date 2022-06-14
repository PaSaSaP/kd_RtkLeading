#ifndef HEADER_RTKLEADING_WHEEL_H_
#define HEADER_RTKLEADING_WHEEL_H_

#include "RtkLeading_def_inc.h"

class Wheel {
public:
    Wheel();
    void loop();
    void updateWheel(int8_t newWheelValue);
    int8_t getWheelState() const; // negative - left, positive - right
    void setStartPos(double lat, double lon);
    void setEndPos(double lat, double lon);
    void setMidPos(double lat, double lon);
    void setStartPos(Pos const& p);
    void setEndPos(Pos const& p);
    void setMidPos(Pos const& p);
    Pos getStartPos() const;
    Pos getEndPos() const;
    Pos getMidPos() const;
    void moveMidBy(double m);
    void calculateBearing();
    double distanceFromPoint(Pos const& p) const;

    static double bearingNorm(double bearing);

private:
    Pos startPos;
    Pos endPos;
    Pos midPos;

    double distanceBetweenMidAndLine;
    double distanceBetweenCurPosAndLine;
    double expectedBearing;
    double calculatedBearing;
    double histeresis = 0.05; // 5 cm = half of histeresis
    double wheelHisteresis = 1.0; // degree
    int8_t wheelDegrees = 30; // when rover out of 10cm histeresis then rotate wheel by that value
    int8_t wheel = 0; // negative - left, positive - right
};

extern Pos currentPosition;
extern Wheel wheel;

#endif /* HEADER_RTKLEADING_WHEEL_H_ */

