#include <Arduino.h>
#include <fan_controller.hpp>
using namespace arduino;

#define TARGET_RPM 0
#define MAX_RPM 1500

#define PWM_PIN 23
#define TACH_PIN 22

static void pwm_set(uint8_t value, void* state) {
  // input is 8-bit
  // write a 4-bit duty 
  ledcWrite(0,value>>4);
}
// four pin fan:
fan_controller fan(pwm_set,nullptr,TACH_PIN,MAX_RPM);
// three pin fan:
//fan_controller fan(pwm_set,nullptr, MAX_RPM);
#define PERIOD_SECS 1
static volatile int ticks;
static volatile uint32_t tick_ts;
static volatile uint32_t tick_ts_old;
static uint32_t ts=0;
static void tick_counter() {
  tick_ts = millis();
  ++ticks;
  tick_ts_old = tick_ts;
}
void setup() {
  Serial.begin(115200);
  ledcAttachPin(PWM_PIN,0);
  ledcSetup(0,25*1000,4);
  if(!fan.initialize()) {
    Serial.println("Could not initialize fan");
    while(true);
  }
  Serial.println("Booted");
  
  fan.rpm(TARGET_RPM);
  Serial.print("Fan set to ");
  Serial.print(TARGET_RPM);
  Serial.println(" RPM");
  delay(5000);
  //fan.pwm_duty(255);
  fan.rpm(1500);
}

void loop() {
  fan.update();
  if(millis()>ts+1000) {
    ts = millis();
    Serial.print(fan.pwm_duty());
    Serial.println(" duty");
    Serial.print(fan.rpm());
    Serial.println(" RPM");
  }
}