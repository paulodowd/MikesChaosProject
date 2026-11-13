#ifndef I2C_DATATYPES_H
#define I2C_DATATYPES_H

#include <stdint.h>

// Existing core protocol: select a register in a one-byte write, then send its
// payload in a separate write or request its response. All values are packed,
// little-endian; float fields use the AVR's 32-bit IEEE-754 representation.
#define ROBOT_I2C_ADDR          0x08
typedef uint8_t RobotRegister_t;
#define REGISTER_NULL           0x00
#define REGISTER_GET_SURFACE    0x01
#define REGISTER_GET_ENCODERS   0x03
#define REGISTER_GET_POSE       0x04
#define REGISTER_SET_POSE       0x05
#define REGISTER_SET_MOTORS     0x06
#define REGISTER_SET_BUZZER     0x07
#define REGISTER_SET_WHEEL_SPEED 0x10
#define REGISTER_GET_WHEEL_SPEED 0x11
#define REGISTER_SET_HEADING     0x12
#define REGISTER_SET_GOTO        0x13
#define REGISTER_GET_GOTO_STATUS 0x14
#define REGISTER_SET_LEFT_PID_GAINS  0x15
#define REGISTER_GET_LEFT_PID_GAINS  0x16
#define REGISTER_SET_RIGHT_PID_GAINS 0x17
#define REGISTER_GET_RIGHT_PID_GAINS 0x18

enum RobotGotoState : uint8_t {
  GOTO_STATUS_IDLE = 0,
  GOTO_STATUS_RUNNING = 1,
  GOTO_STATUS_COMPLETE = 2,
  GOTO_STATUS_CANCELLED = 3,
  GOTO_STATUS_TIMEOUT = 4,
  GOTO_STATUS_ERROR = 5
};

enum RobotGotoPhase : uint8_t {
  GOTO_PHASE_NONE = 0,          // No move, or accepted but not yet evaluated.
  GOTO_PHASE_ALIGN = 1,         // Turn towards the target position.
  GOTO_PHASE_DRIVE = 2,         // Approach the target position.
  GOTO_PHASE_FINAL_HEADING = 3, // Turn to the requested final orientation.
  GOTO_PHASE_SETTLING = 4       // Within pose tolerances; wait for rest.
};

// Protocol bound, not a statement about the robot's achievable speed.
constexpr float WHEEL_SPEED_MAX_ABS_CPS = 10000.0f;

// Same configuration bounds as the serial tuning harness. Zero is allowed.
constexpr float SPEED_PID_MAX_KP = 1.0f;
constexpr float SPEED_PID_MAX_KI = 10.0f;
constexpr float SPEED_PID_MAX_KD = 0.01f;

// Former registers 0x08..0x0F are deliberately unused, not reassigned.
#pragma pack(push, 1)

struct RobotSurface_t {
  uint32_t timestamp_us;  // Acquisition start, from micros(). Wraps at 2^32.
  uint16_t reading[5];    // DN1..DN5 raw RC readings, 25 us/tick, 0..5000 us.
};

struct RobotEncoders_t {
  int32_t count[2];       // Left, right; positive is forward, modulo 2^32.
};

struct RobotPose_t {
  float x;               // mm; forward at theta=0 is +x.
  float y;               // mm.
  float theta;           // Unwrapped radians; positive is anticlockwise.
};

struct RobotMotors_t {
  int16_t pwm[2];         // Left, right; signed Timer1 compare values.
};

struct RobotBuzzer_t {
  uint16_t freq;          // Hz; zero stops the buzzer.
  uint16_t duration_ms;   // Zero duration plays continuously until stopped.
};

struct RobotWheelSpeedDemand_t {
  float counts_per_second[2];  // Left, right; positive is forward.
};

struct RobotWheelSpeed_t {
  uint32_t timestamp_us;       // End of measurement interval; wraps at 2^32.
  uint32_t interval_us;        // Actual measurement duration; 0 = not yet valid.
  float counts_per_second[2];  // Left, right; encoder delta / elapsed seconds.
};

struct RobotPIDGains_t {
  float kp; // PWM / (count/s)
  float ki; // PWM / count
  float kd; // PWM / (count/s^2)
};

struct RobotHeadingDemand_t {
  float heading_rad;   // Absolute odometry-frame orientation; positive anticlockwise.
  float forward_mm_s;  // Signed body velocity; zero requests a turn on the spot.
};

using RobotGotoCommand_t = RobotPose_t; // Absolute x/y in mm, theta in radians.

struct RobotGotoStatus_t {
  uint8_t state;                // RobotGotoState.
  uint8_t phase;                // RobotGotoPhase; retained on termination.
  uint32_t request_id;          // Incremented for each new accepted move.
  uint32_t elapsed_ms;          // Since acceptance; frozen on termination.
  float distance_error_mm;     // Distance to target x/y at last evaluation.
  float heading_error_rad;     // Shortest error to final theta, not path bearing.
  RobotPose_t target;           // Most recently accepted destination.
};

#pragma pack(pop)

static_assert(sizeof(RobotSurface_t) == 14, "Surface packet layout changed");
static_assert(sizeof(RobotEncoders_t) == 8, "Encoder packet layout changed");
static_assert(sizeof(RobotPose_t) == 12, "Pose packet layout changed");
static_assert(sizeof(RobotMotors_t) == 4, "Motor packet layout changed");
static_assert(sizeof(RobotBuzzer_t) == 4, "Buzzer packet layout changed");
static_assert(sizeof(RobotWheelSpeedDemand_t) == 8, "Speed demand layout changed");
static_assert(sizeof(RobotWheelSpeed_t) == 16, "Speed measurement layout changed");
static_assert(sizeof(RobotPIDGains_t) == 12, "PID gains packet layout changed");
static_assert(sizeof(RobotHeadingDemand_t) == 8, "Heading demand layout changed");
static_assert(sizeof(RobotGotoCommand_t) == 12, "Goto command layout changed");
static_assert(sizeof(RobotGotoStatus_t) == 30, "Goto status must fit AVR Wire's 32 bytes");

#endif
