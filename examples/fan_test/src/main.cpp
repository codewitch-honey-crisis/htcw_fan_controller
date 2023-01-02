#include <Arduino.h>
#include <fan_controller.hpp>
using namespace arduino;

static void pwm_set(uint8_t value, void* state) {
  ledcWrite(0,value>>4);
}

fan_controller fan(pwm_set,nullptr,22,1500,2,2);
//fan_controller fan(pwm_set,nullptr, 1500);
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
  ledcAttachPin(23,0);
  ledcSetup(0,25*1000,4);
  if(!fan.initialize()) {
    Serial.println("Could not initialize fan");
    while(true);
  }
  Serial.println("Booted");
  
  //fan.rpm(0);
  //delay(5000);
 // fan.rpm(200);
 // delay(5000);
  fan.rpm(500);
  Serial.println("Fan set to 1500 RPM");
}

void loop() {
  fan.update();
  if(millis()>ts+1000) {
    ts = millis();
    Serial.print(fan.rpm());
    Serial.println(" RPM");
  }
}