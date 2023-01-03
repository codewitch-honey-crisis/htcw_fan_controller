#include <Arduino.h>
#include <fan_controller.hpp>
using namespace arduino;
#define PWM_PIN 23
#define TACH_PIN 22

#define MAX_RPM 1500

static uint32_t ts;
static void pwm_set(uint8_t value, void *state)
{
  // input is 8-bit
  // write a 4-bit duty
  ledcWrite(0, value >> 4);
}

fan_controller fan(pwm_set,nullptr,TACH_PIN,MAX_RPM);
void setup()
{
  ts = 0;
  Serial.begin(115200);
  ledcAttachPin(PWM_PIN, 0);
  ledcSetup(0, 25 * 1000, 4);
  fan.initialize();
  Serial.println("Booted");
  fan.rpm(MAX_RPM/2);
}

void loop()
{
  fan.update();
  if(millis()>ts+250) {
    ts=millis();
    Serial.print(fan.rpm());
    Serial.println(" RPM");
    Serial.print(fan.pwm_duty());
    Serial.println(" duty");
  }
}