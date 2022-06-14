#include <Arduino.h>
#include "RtkLeading_extern.h"

long RtkLeading_getCurrentTime() {
    return millis();
}

void RtkLeading_log(String const& str) {
    // do nothing
}

void RtkLeading_logDouble(String const& str, double v, int dec) {
    // do nothing
}

void RtkLeading_logInt(String const& str, int v) {
    // do nothing
}

