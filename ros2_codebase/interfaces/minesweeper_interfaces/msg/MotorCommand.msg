# Current boot/connection challenge from MotorStatus; not an authentication token.
uint32 session
# Strictly increasing in a session, starting at 1. No wrapping.
uint32 sequence
# Unix epoch milliseconds; clocks must be synchronized; NOT ROS simulation time.
uint64 issued_at_ms
# Zero/disarm -> zero/arm -> armed targets. Reconnection never arms.
bool arm
# Configured per channel: rad/s with encoder feedback, signed duty otherwise.
float32[5] target
