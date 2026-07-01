#ifndef KQ_OSCILLATOR_H_
#define KQ_OSCILLATOR_H_

#include "servo_constants.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Pure sine-waveform generator for one leg's swing angle.
//
// No servo I/O and no timing source of its own: GaitController advances the phase
// once per fixed 20ms tick and reads the angle. (The otto Oscillator is welded to
// LEDC and an extern millis(); only its sine math is reused here -- the output sink
// and the clock live in GaitController.) Output is centered at 90 degrees and
// clamped to [0,180]; right-side legs set reverse to mirror the swing.
class Oscillator
{
private:
    double mAmplitudeDeg;  // peak swing in degrees about center
    double mPhase0;        // phase offset (radians)
    double mPhase;         // current phase (radians)
    double mInc;           // phase increment per tick (radians)
    bool   mReverse;       // mirror the swing for right-side legs

public:
    Oscillator()
    {
        mAmplitudeDeg = 0.0;
        mPhase0 = 0.0;
        mPhase = 0.0;
        mInc = 0.0;
        mReverse = false;
    }

    // Configure for a continuous motion; periodMs is one full sine cycle.
    void configure(int amplitudeDeg, int periodMs, double phase0, bool reverse, int tickMs)
    {
        mAmplitudeDeg = amplitudeDeg;
        mPhase0 = phase0;
        mReverse = reverse;
        mPhase = 0.0;
        mInc = (periodMs > 0) ? (2.0 * M_PI * tickMs / (double)periodMs) : 0.0;
    }

    // Angle to command this tick, centered at 90 degrees, clamped to [MIN,MAX].
    int currentAngle()
    {
        double tSwing = mAmplitudeDeg * std::sin(mPhase + mPhase0);
        if (mReverse)
        {
            tSwing = -tSwing;
        }
        int tAngle = (int)std::lround(ServoConstants::CENTER_DEGREE + tSwing);
        if (tAngle < ServoConstants::MIN_DEGREE)
        {
            tAngle = ServoConstants::MIN_DEGREE;
        }
        if (tAngle > ServoConstants::MAX_DEGREE)
        {
            tAngle = ServoConstants::MAX_DEGREE;
        }
        return tAngle;
    }

    void advance()
    {
        mPhase += mInc;
    }
};

#endif // KQ_OSCILLATOR_H_
