#ifndef BOARD_H
#define BOARD_H
#include "stm32f4xx_hal.h"
#include "control.h"
/* Default image remains motor-disabled. The explicit MDD10A build profile follows
 * the supplied five PWM+DIR connections, with no STM E-stop/ENABLE input. */
#ifndef BOARD_PROFILE_MDD10A
#define BOARD_PROFILE_MDD10A 0
#endif
#if BOARD_PROFILE_MDD10A
#define BOARD_HARDWARE_APPROVED 1
#define BOARD_ESTOP_PRESENT 0
#else
#define BOARD_HARDWARE_APPROVED 0
#define BOARD_ESTOP_PRESENT 1
#endif
#define BOARD_PWM_FREQUENCY_HZ 20000u /* Cytron MDD10A: active-high sign-magnitude */
#define BOARD_TRANSPORT_USB_CDC 1 /* MCU USB peripheral pins; board routing UNVERIFIED */
typedef struct { GPIO_TypeDef *port; uint16_t pin; bool active_high; } gpio_signal_t;
typedef struct {
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  gpio_signal_t direction, enable;
  bool enable_present, pwm_active_high, direction_positive_high;
  uint32_t reversal_deadtime_ms;
} motor_binding_t;
typedef struct {
  TIM_HandleTypeDef *timer;
  uint8_t bits;
  int8_t sign;
  uint32_t max_delta_per_period;
  float counts_per_revolution; /* counts AFTER configured timer decoding */
} encoder_binding_t;
extern const motor_config_t board_motor_config[MOTOR_COUNT];
extern const motor_binding_t board_motors[MOTOR_COUNT];
extern const encoder_binding_t board_encoders[ENCODER_COUNT];
extern const gpio_signal_t board_estop;
bool board_init(void);
bool board_estop_active(void);
void gpio_write(gpio_signal_t signal, bool active);
bool gpio_output_init(gpio_signal_t signal);
bool gpio_read(gpio_signal_t signal);
void motor_stop_all(void); /* also callable with interrupts disabled */
bool motor_write(unsigned index, float duty, uint32_t now_ms);
bool encoder_init(void);
bool encoder_sample(float velocity[ENCODER_COUNT], int64_t ticks[ENCODER_COUNT]);
typedef struct { float accel[3], gyro[3]; uint32_t stamp_ms; bool valid; } imu_sample_t;
bool imu_init(void);
bool imu_read(imu_sample_t *sample);
typedef enum { METAL_UNBOUND, METAL_DIGITAL, METAL_ADC } metal_mode_t;
typedef struct { metal_mode_t mode; gpio_signal_t digital; ADC_HandleTypeDef *adc;
                 uint32_t debounce_ms; } metal_binding_t;
extern const metal_binding_t board_metal;
bool metal_read(float *value, uint32_t now_ms);
void board_panic(void) __attribute__((noreturn));
#endif
