#include "control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static const safety_inputs_t healthy={true,true,false,true,true};
static motor_config_t cfg[MOTOR_COUNT];
static unsigned assertions;
#define CHECK(x) do { ++assertions; assert(x); } while (0)
static void configuration(void) {
  for (unsigned i=0;i<MOTOR_COUNT;++i) cfg[i]=(motor_config_t){
    .mode=i<2?MOTOR_VELOCITY:MOTOR_DUTY,.encoder=i<2?(int8_t)i:-1,
    .target_limit=i<2?20:1,.duty_limit=.3f,.kp=.2f,.ki=.1f,.kd=0,.integral_limit=2};
}
static motor_command_t command(safety_t *s,uint32_t sequence,bool arm,uint64_t epoch) {
  motor_command_t c={.session=s->session,.sequence=sequence,.arm=arm,.issued_at_ms=epoch};return c;
}
static void arm(safety_t *s,uint32_t now,uint64_t epoch) {
  motor_command_t c=command(s,1,false,epoch);
  CHECK(safety_command(s,cfg,&c,healthy,now,epoch));CHECK(s->ready && !s->armed);
  c.arm=true;c.sequence=2;
  CHECK(safety_command(s,cfg,&c,healthy,now,epoch));CHECK(s->armed && !s->ready);
}
static void safe(const safety_t *s) {
  CHECK(!s->armed);for(unsigned i=0;i<MOTOR_COUNT;++i) CHECK(s->target[i]==0);
}
int main(void) {
  configuration();safety_t s;motor_command_t c;
  CHECK(motor_feedback_required(cfg));
  for (unsigned i=0;i<MOTOR_COUNT;++i) cfg[i].mode=MOTOR_DUTY;
  CHECK(!motor_feedback_required(cfg));
  configuration();
  safety_init(&s,100);safe(&s);
  c=command(&s,1,true,10000);
  CHECK(!safety_command(&s,cfg,&c,healthy,0,10000));safe(&s); /* no arm on boot */
  arm(&s,0,10000);
  c=command(&s,3,true,10005);c.target[0]=5;c.target[3]=.2f;
  CHECK(safety_command(&s,cfg,&c,healthy,5,10005));CHECK(s.target[0]==5);
  safety_update(&s,healthy,254);CHECK(s.armed);
  uint32_t session=s.session;safety_update(&s,healthy,255);safe(&s);
  CHECK(s.faults&FAULT_TIMEOUT);CHECK(s.session!=session);
  safety_update(&s,healthy,500);safe(&s); /* no auto restart */
  c=command(&s,1,true,10500);CHECK(!safety_command(&s,cfg,&c,healthy,500,10500));safe(&s);
  arm(&s,501,10501);
  safety_inputs_t failure=healthy;failure.link_ok=false;
  safety_update(&s,failure,502);safe(&s);
  safety_update(&s,healthy,503);safe(&s);
  arm(&s,504,10504);
  failure=healthy;failure.estop=true;safety_update(&s,failure,505);safe(&s);
  safety_update(&s,healthy,506);safe(&s);arm(&s,507,10507);
  failure=healthy;failure.hardware_ok=false;safety_update(&s,failure,508);safe(&s);
  c=command(&s,1,false,10509);CHECK(!safety_command(&s,cfg,&c,failure,509,10509));safe(&s);
  const float invalid[]={NAN,INFINITY,-INFINITY,20.001f,-20.001f};
  for (unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) {
    arm(&s,600,10600);c=command(&s,3,true,10600);c.target[0]=invalid[i];
    CHECK(!safety_command(&s,cfg,&c,healthy,600,10600));safe(&s);
  }
  arm(&s,600,10600);c=command(&s,3,true,10499);
  CHECK(!safety_command(&s,cfg,&c,healthy,600,10600));safe(&s); /* stale reliable backlog */
  arm(&s,600,10600);c=command(&s,3,true,10621);
  CHECK(!safety_command(&s,cfg,&c,healthy,600,10600));safe(&s); /* future clock */
  arm(&s,600,10600);c=command(&s,2,true,10600);
  CHECK(!safety_command(&s,cfg,&c,healthy,600,10600));safe(&s); /* duplicate */
  arm(&s,600,10600);c=command(&s,3,true,10600);--c.session;
  CHECK(!safety_command(&s,cfg,&c,healthy,600,10600));safe(&s); /* old session */
  arm(&s,UINT32_MAX-100,10600);
  safety_update(&s,healthy,148);CHECK(s.armed);
  safety_update(&s,healthy,149);safe(&s); /* HAL tick wrap */
  arm(&s,600,10600);failure=healthy;failure.feedback_ok=false;
  safety_update(&s,failure,601);safe(&s);
  arm(&s,600,10600);failure=healthy;failure.clock_ok=false;
  safety_update(&s,failure,601);safe(&s);
  arm(&s,600,10600);safety_trip(&s,FAULT_DEADLINE);safe(&s);
  arm(&s,600,10600);c=command(&s,3,false,10600);c.target[2]=.1f;
  CHECK(!safety_command(&s,cfg,&c,healthy,600,10600));safe(&s);
  arm(&s,600,10600);c=command(&s,3,false,10600);
  CHECK(safety_command(&s,cfg,&c,healthy,600,10600));safe(&s);
  c=command(&s,4,true,10600);c.target[2]=.1f;
  CHECK(!safety_command(&s,cfg,&c,healthy,600,10600));safe(&s); /* arm must be zero */
  motor_pid_t pid={0};
  for (unsigned i=0;i<10000;++i) CHECK(fabsf(pid_step(&pid,&cfg[0],20,0,.005f))<=.3f);
  CHECK(fabsf(pid.integral)<.001f); /* saturated integral did not wind up */
  CHECK(pid_step(&pid,&cfg[0],0,NAN,.005f)==0);CHECK(!pid.have_previous);
  CHECK(pid_step(&pid,&cfg[0],1,0,0)==0);
  int32_t delta;
  CHECK(encoder_delta(2,65534,16,10,&delta) && delta==4);
  CHECK(encoder_delta(65534,2,16,10,&delta) && delta==-4);
  CHECK(encoder_delta(2,UINT32_MAX-1,32,10,&delta) && delta==4);
  CHECK(encoder_delta(UINT32_MAX-1,2,32,10,&delta) && delta==-4);
  CHECK(!encoder_delta(32768,0,16,32767,&delta));
  CHECK(!encoder_delta(100,0,16,99,&delta));
  CHECK(!encoder_delta(1,0,8,10,&delta));
  CHECK(!encoder_delta(1,0,16,10,NULL));
  printf("control tests: %u assertions passed\n",assertions);
  return 0;
}
