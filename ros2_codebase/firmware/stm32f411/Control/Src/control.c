#include "control.h"
#include <math.h>
#include <string.h>
static float clamp(float v, float limit) {
  return fmaxf(-limit, fminf(limit, v));
}
bool motor_feedback_required(const motor_config_t cfg[MOTOR_COUNT]) {
  for (unsigned i=0;i<MOTOR_COUNT;++i) if (cfg[i].mode==MOTOR_VELOCITY) return true;
  return false;
}
void safety_init(safety_t *s, uint32_t session) {
  memset(s, 0, sizeof(*s));
  s->session = session ? session : 1;
  s->faults = FAULT_HARDWARE | FAULT_LINK;
}
void safety_trip(safety_t *s, uint32_t faults) {
  /* Rotate the challenge once per new fault/active command stream. */
  if (s->armed || s->ready || s->have_command || (faults & ~s->faults)) {
    if (++s->session == 0) ++s->session;
  }
  s->armed = s->ready = s->have_command = s->have_sequence = false;
  memset(s->target, 0, sizeof(s->target));
  s->faults |= faults;
}
static uint32_t input_faults(safety_inputs_t in) {
  return (!in.hardware_ok ? FAULT_HARDWARE : 0u) |
    (!in.link_ok ? FAULT_LINK : 0u) | (in.estop ? FAULT_ESTOP : 0u) |
    (!in.feedback_ok ? FAULT_FEEDBACK : 0u) |
    (!in.clock_ok ? FAULT_CLOCK : 0u);
}
void safety_update(safety_t *s, safety_inputs_t in, uint32_t now_ms) {
  uint32_t faults = input_faults(in);
  if (s->have_command && (uint32_t)(now_ms-s->last_command_ms) >= COMMAND_TIMEOUT_MS)
    faults |= FAULT_TIMEOUT;
  if (faults) safety_trip(s, faults);
}
bool safety_command(safety_t *s, const motor_config_t cfg[MOTOR_COUNT],
                    const motor_command_t *cmd, safety_inputs_t in,
                    uint32_t now_ms, uint64_t epoch_ms) {
  safety_update(s, in, now_ms);
  if (input_faults(in)) return false;
  bool zero = true;
  for (unsigned i=0; i<MOTOR_COUNT; ++i) {
    if (cfg[i].mode == MOTOR_UNBOUND || !isfinite(cfg[i].target_limit) ||
        cfg[i].target_limit <= 0 || !isfinite(cmd->target[i]) ||
        fabsf(cmd->target[i]) > cfg[i].target_limit) {
      safety_trip(s, FAULT_COMMAND); return false;
    }
    if (cmd->target[i] != 0) zero = false;
  }
  if (cmd->session != s->session || cmd->sequence == 0 ||
      (s->have_sequence && cmd->sequence <= s->sequence) ||
      cmd->issued_at_ms == 0 || epoch_ms == 0 ||
      (epoch_ms > cmd->issued_at_ms && epoch_ms-cmd->issued_at_ms > COMMAND_MAX_AGE_MS) ||
      (cmd->issued_at_ms > epoch_ms && cmd->issued_at_ms-epoch_ms > COMMAND_FUTURE_TOLERANCE_MS)) {
    safety_trip(s, FAULT_COMMAND); return false;
  }
  if (!cmd->arm) {
    if (!zero) { safety_trip(s, FAULT_COMMAND); return false; }
    s->armed=false; s->ready=true; s->faults=0;
  } else if (!s->armed) {
    if (!s->ready || !zero) { safety_trip(s, FAULT_COMMAND); return false; }
    s->armed=true; s->ready=false;
  }
  memcpy(s->target, cmd->target, sizeof(s->target));
  s->sequence=cmd->sequence; s->have_sequence=true;
  s->last_command_ms=now_ms; s->have_command=true;
  return true;
}
void pid_reset(motor_pid_t *p) { memset(p, 0, sizeof(*p)); }
float pid_step(motor_pid_t *p, const motor_config_t *c, float target, float measured, float dt) {
  if (!isfinite(target) || !isfinite(measured) || !isfinite(dt) || dt <= 0 ||
      !isfinite(c->kp) || !isfinite(c->ki) || !isfinite(c->kd) ||
      c->duty_limit <= 0 || c->duty_limit > 1 || c->integral_limit < 0) {
    pid_reset(p); return 0;
  }
  float error=target-measured;
  float derivative=p->have_previous ? (error-p->previous_error)/dt : 0;
  float candidate=clamp(p->integral+error*dt, c->integral_limit);
  float raw=c->kp*error+c->ki*candidate+c->kd*derivative;
  /* Conditional integration: permit unwinding, inhibit winding into saturation. */
  if (fabsf(raw) <= c->duty_limit || raw*error < 0) p->integral=candidate;
  p->previous_error=error; p->have_previous=true;
  return clamp(c->kp*error+c->ki*p->integral+c->kd*derivative, c->duty_limit);
}
bool encoder_delta(uint32_t now, uint32_t previous, uint8_t bits,
                   uint32_t max_delta, int32_t *delta) {
  if (!delta || (bits != 16 && bits != 32)) return false;
  uint64_t modulus=UINT64_C(1)<<bits;
  uint32_t raw=(uint32_t)(((uint64_t)now-previous)&(modulus-1));
  if (raw == modulus/2 || max_delta >= modulus/2) return false;
  int64_t signed_delta=raw < modulus/2 ? raw : (int64_t)raw-(int64_t)modulus;
  if (signed_delta > max_delta || signed_delta < -(int64_t)max_delta) return false;
  *delta=(int32_t)signed_delta; return true;
}
