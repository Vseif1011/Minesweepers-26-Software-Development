#include "board.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static GPIO_TypeDef direction,enable;
static timer_regs_t regs={.arr=1000};static TIM_HandleTypeDef timer={&regs};
const motor_binding_t board_motors[MOTOR_COUNT]={
  {&timer,0,{&direction,1,true},{&enable,1,true},true,true,true,10},
  {&timer,4,{&direction,2,true},{&enable,2,false},true,true,false,10}
};
const motor_config_t board_motor_config[MOTOR_COUNT]={
  {.duty_limit=.25f},{.duty_limit=.25f}
};
void gpio_write(gpio_signal_t s,bool active) {
  if(s.port) HAL_GPIO_WritePin(s.port,s.pin,active==s.active_high);
}
int main(void) {
  motor_stop_all();assert(regs.ccr[0]==0 && !(enable.value&1) && (enable.value&2));
  assert(motor_write(0,.8f,100));assert(regs.ccr[0]==250 && (enable.value&1) && (direction.value&1));
  assert(motor_write(0,-.8f,101));assert(regs.ccr[0]==0 && !(enable.value&1));
  assert(motor_write(0,-.8f,110));assert(regs.ccr[0]==0); /* deadtime not elapsed */
  assert(motor_write(0,-.8f,111));assert(regs.ccr[0]==250 && !(direction.value&1));
  assert(motor_write(1,.2f,100));assert(regs.ccr[1]==200 && !(enable.value&2) && !(direction.value&2));
  motor_stop_all();assert(regs.ccr[0]==0 && regs.ccr[1]==0 && (enable.value&2));
  assert(!motor_write(5,.1f,200));assert(!motor_write(0,NAN,200));
  puts("motor driver tests: PWM limits, direction/enable polarity, reversal deadtime, stop and invalid inputs passed");
}
