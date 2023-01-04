#include <Arduino.h>
#include <fan_controller.hpp>
using namespace arduino;
#define PWM_PIN 23
#define TACH_PIN 22
#define MAX_RPM NAN
static void pwm_set(uint8_t value, void *state)
{
  // input is 8-bit
  ledcWrite(0, value);
}
static fan_controller fan(pwm_set,nullptr,TACH_PIN,MAX_RPM);
static int state = 0;
static uint32_t ts=0;
static float min_rpm=NAN;
void setup()
{
  Serial.begin(115200);
  
  ledcAttachPin(PWM_PIN, 0);
  ledcSetup(0, 25 * 1000, 8);
  Serial.println("Finding minimum RPM...");
  min_rpm = fan_controller::find_min_rpm(pwm_set,nullptr,TACH_PIN);
  Serial.println("Finding maximum RPM...");
  // since we passed NAN for max RPM it will be detected on init
  fan.initialize();
  Serial.print("RPM range: ");
  Serial.print(min_rpm);
  Serial.print(" - ");
  Serial.println(fan.max_rpm());
  delay(2000);
  Serial.print("Setting to ");
  Serial.print(fan.max_rpm()/2.0);
  Serial.println(" RPM");
  fan.rpm(fan.max_rpm()/2);
}
void loop() {
  fan.update();
  if(millis()>ts+250) {  
      ts=millis();
      switch(state) {
        case 50:
          Serial.print("Setting to ");
          Serial.print(fan.max_rpm());
          Serial.println(" RPM");
          fan.rpm(fan.max_rpm());
          break;
        case 100:
          Serial.print("Setting to ");
          Serial.print(min_rpm);
          Serial.println(" RPM");
          fan.rpm(min_rpm);
          break;
        case 150:
          Serial.print("Setting to ");
          Serial.print(fan.max_rpm()/2.0);
          Serial.println(" RPM");
          fan.rpm(fan.max_rpm()/2.0);
          state = 0;
          break;
      }
      ++state;
      Serial.print(fan.rpm());
      Serial.print(" RPM / ");
      Serial.print(fan.pwm_duty());
      Serial.println(" duty");
      
    }
  
}
