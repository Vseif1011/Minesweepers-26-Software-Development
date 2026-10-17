#ifndef CONTROL_H
#define CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#define MOTOR_COUNT 5
#define ENCODER_COUNT 2
enum { FAULT_HARDWARE=1u, FAULT_LINK=2u, FAULT_TIMEOUT=4u,
       FAULT_ESTOP=8u, FAULT_FEEDBACK=16u, FAULT_COMMAND=32u,
       FAULT_DEADLINE=64u, FAULT_CLOCK=128u };
typedef enum { MOTOR_UNBOUND, MOTOR_DUTY, MOTOR_VELOCITY } motor_mode_t;
typedef struct {
  motor_mode_t mode;
  int8_t encoder; /* -1 for no feedback; at most two distinct feedback motors */
  float target_limit, duty_limit, kp, ki, kd, integral_limit;
} motor_config_t;
typedef struct {
  uint32_t session, sequence;
  uint64_t issued_at_ms;
  bool arm;
  float target[MOTOR_COUNT];
} motor_command_t;
typedef struct {
  bool hardware_ok, link_ok, estop, feedback_ok, clock_ok;
} safety_inputs_t;
typedef struct {
  uint32_t session, sequence, last_command_ms, faults;
  bool armed, ready, have_command, have_sequence;
  float target[MOTOR_COUNT];
} safety_t;
typedef struct { float integral, previous_error; bool have_previous; } motor_pid_t;
void safety_init(safety_t *s, uint32_t session);
void safety_trip(safety_t *s, uint32_t faults);
void safety_update(safety_t *s, safety_inputs_t in, uint32_t now_ms);
bool safety_command(safety_t *s, const motor_config_t cfg[MOTOR_COUNT],
                    const motor_command_t *cmd, safety_inputs_t in,
                    uint32_t now_ms, uint64_t epoch_ms);
void pid_reset(motor_pid_t *pid);
bool motor_feedback_required(const motor_config_t cfg[MOTOR_COUNT]);
float pid_step(motor_pid_t *pid, const motor_config_t *cfg, float target,
               float measured, float dt);
/* Unsigned modular timer delta, rejecting ambiguous half-range and bad bounds. */
bool encoder_delta(uint32_t now, uint32_t previous, uint8_t bits,
                   uint32_t max_delta, int32_t *delta);
#define COMMAND_TIMEOUT_MS 250u
#define COMMAND_MAX_AGE_MS 100u
#define COMMAND_FUTURE_TOLERANCE_MS 20u
#define CONTROL_PERIOD_MS 5u
#endif
