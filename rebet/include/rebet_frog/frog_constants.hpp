#ifndef rebet__FROG_CONSTS_HPP_
#define rebet__FROG_CONSTS_HPP_

#include <math.h>

inline constexpr double DETECTION_AVG_POW = 59.275; //watts

inline constexpr char* OBJECT_DETECTED_STRING = "DETECTED";
inline constexpr char* OBJECT_NOT_DETECTED_STRING = "NOT_DETECTED";

inline constexpr float WAFFLE_MAX_LIN_VEL = 0.26;
static constexpr int PIC_INCREMENT = 2;
static constexpr int START_PIC_RATE = 1;

static float calculate_power_motion(float speed) {
      return 6.25 * pow(speed, 2) + 9.79 * speed + 3.66;
    }




#endif  // rebet__FROG_CONSTS_HPP_