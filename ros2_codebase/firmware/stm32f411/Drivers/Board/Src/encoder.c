#include "board.h"
#include <math.h>
static uint32_t previous[ENCODER_COUNT];
static int64_t total[ENCODER_COUNT];
bool encoder_init(void) {
  for (unsigned i=0; i<ENCODER_COUNT; ++i) {
    const encoder_binding_t *e=&board_encoders[i];
    if (!e->timer || (e->bits!=16 && e->bits!=32) ||
        (e->sign!=1 && e->sign!=-1) || !isfinite(e->counts_per_revolution) ||
        e->counts_per_revolution<=0 || !e->max_delta_per_period) return false;
    if (HAL_TIM_Encoder_Start(e->timer,TIM_CHANNEL_ALL)!=HAL_OK) return false;
    previous[i]=__HAL_TIM_GET_COUNTER(e->timer); total[i]=0;
  }
  return true;
}
bool encoder_sample(float velocity[ENCODER_COUNT], int64_t ticks[ENCODER_COUNT]) {
  for (unsigned i=0; i<ENCODER_COUNT; ++i) {
    const encoder_binding_t *e=&board_encoders[i];
    if (!e->timer || e->counts_per_revolution<=0) return false;
    uint32_t now=__HAL_TIM_GET_COUNTER(e->timer); int32_t delta;
    if (!encoder_delta(now,previous[i],e->bits,e->max_delta_per_period,&delta)) return false;
    previous[i]=now;
    int64_t signed_delta=(int64_t)delta*e->sign;
    total[i]+=signed_delta; ticks[i]=total[i];
    velocity[i]=(float)signed_delta*6.28318530718f/
      (e->counts_per_revolution*(CONTROL_PERIOD_MS/1000.0f));
  }
  return true;
}
