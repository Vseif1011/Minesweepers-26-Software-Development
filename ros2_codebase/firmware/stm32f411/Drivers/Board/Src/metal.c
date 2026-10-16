#include "board.h"
static bool candidate, stable, initialized;
static uint32_t changed_at;
bool metal_read(float *value, uint32_t now) {
  if (!value) return false;
  if (board_metal.mode==METAL_DIGITAL && board_metal.digital.port) {
    bool raw=gpio_read(board_metal.digital);
    if (!initialized || raw!=candidate) { candidate=raw;changed_at=now;initialized=true; }
    if ((uint32_t)(now-changed_at)>=board_metal.debounce_ms) stable=candidate;
    *value=stable?1.0f:0.0f; return true;
  }
  if (board_metal.mode==METAL_ADC && board_metal.adc) {
    if (HAL_ADC_Start(board_metal.adc)!=HAL_OK) return false;
    bool ok=HAL_ADC_PollForConversion(board_metal.adc,1)==HAL_OK;
    if (ok) *value=(float)HAL_ADC_GetValue(board_metal.adc); /* raw ADC counts */
    HAL_ADC_Stop(board_metal.adc); return ok;
  }
  return false;
}
