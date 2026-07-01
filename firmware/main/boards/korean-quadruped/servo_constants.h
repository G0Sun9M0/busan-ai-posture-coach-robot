#ifndef KQ_SERVO_CONSTANTS_H_
#define KQ_SERVO_CONSTANTS_H_

// Leg placement and mirror map for the 4-leg, 1-DOF-per-leg quadruped.
// The leg index IS the PCA9685 channel: 0=FL, 1=FR, 2=RL, 3=RR. The right side is
// mirror-mounted, so the sine output is reversed on FR and RR (index 1 and 3). These
// values are the BOM wiring; they are not runtime-tunable (there is no trim subsystem --
// legs are mounted to the physical pose the 90-degree center command produces).

namespace ServoConstants
{
    static constexpr int LEG_COUNT = 4;

    // Right side is mirror-mounted; reverse the swing on FR and RR (index 1 and 3).
    static constexpr bool REVERSE[LEG_COUNT] = { false, true, false, true };

    // Soft angle bounds; mechanical center is 90 degrees.
    static constexpr int CENTER_DEGREE = 90;
    static constexpr int MIN_DEGREE = 0;
    static constexpr int MAX_DEGREE = 180;
}

#endif // KQ_SERVO_CONSTANTS_H_
