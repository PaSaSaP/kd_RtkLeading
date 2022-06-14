#ifndef HEADER_RTKLEADING_EXTERN_H_
#define HEADER_RTKLEADING_EXTERN_H_

long RtkLeading_getCurrentTime() __attribute__((weak));
void RtkLeading_log(String const& str) __attribute__((weak));
void RtkLeading_logDouble(String const& str, double v, int dec) __attribute__((weak));
void RtkLeading_logInt(String const& str, int v) __attribute__((weak));

#endif /* HEADER_RTKLEADING_EXTERN_H_ */

