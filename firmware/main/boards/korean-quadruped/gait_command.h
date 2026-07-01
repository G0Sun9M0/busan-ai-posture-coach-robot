#ifndef KQ_GAIT_COMMAND_H_
#define KQ_GAIT_COMMAND_H_

// POD message passed from MCP callbacks (main thread) to the gait action task
// via a FreeRTOS queue. Plain value type by design: it is copied into the queue
// by value, so every field is primitive (no owning members).

enum class GaitType
{
    Home,
    WalkForward,
    WalkBack,
    TurnLeft,
    TurnRight,
    Dance,
    Wave,
    Sit,
    Stretch
};

struct GaitCommand
{
    GaitType type;
    int steps;
};

#endif // KQ_GAIT_COMMAND_H_
