#include <Arduino.h>
#include <fan_controller.hpp>
using namespace arduino;
#define PWM_PIN 23
#define TACH_PIN 22
#define MAX_RPM NAN
#ifdef ESP32
static void pwm_set(uint16_t value, void *state)
{
  // output is 8-bit
  ledcWrite(0, value>>8);
}
static fan_controller fan(pwm_set,nullptr,TACH_PIN,MAX_RPM);
#else
static void pwm_set(uint16_t value, void *state)
{
  Serial.print("TODO: Write ");
  Serial.print((value/65535.0)*100);
  Serial.println("% duty");
}
using fan_ctrl = fan_controller<TACH_PIN>;
static fan_ctrl fan(pwm_set,nullptr,MAX_RPM);
#endif

static int state = 0;
static uint32_t ts=0;
static float min_rpm=NAN;
void setup()
{
#ifdef ESP32
  ledcAttachPin(PWM_PIN, 0);
  ledcSetup(0, 25 * 1000, 8);
#endif
  Serial.begin(115200);
  Serial.println("Finding minimum RPM...");
  min_rpm = 0;
#if TACH_PIN > -1
#ifdef ESP32
  min_rpm = fan_controller::find_min_rpm(pwm_set,nullptr,TACH_PIN);
#else
  min_rpm = fan_ctrl::find_min_rpm(pwm_set,nullptr);
#endif
#endif
  Serial.println("Finding maximum RPM...");
  // since we passed NAN for max RPM it will be detected on init
  fan.initialize();
  Serial.print("RPM range: ");
  Serial.print(min_rpm);
  Serial.print(" - ");
  Serial.println(fan.max_rpm());
  delay(2000);
  Serial.print("Setting to ");
  Serial.print((((int)fan.max_rpm())/100)*50);
  Serial.println(" RPM");
  fan.rpm((((int)fan.max_rpm())/100)*50);
}
void loop() {
  int new_rpm;
  fan.update();
  if(millis()>ts+250) {  
      ts=millis();
      switch(state) {
        case 50:
          Serial.print("Setting to ");
          new_rpm = (((int)fan.max_rpm())/100)*100;
          Serial.print(new_rpm);
          Serial.println(" RPM");
          fan.rpm(new_rpm);
          break;
        case 100:
          Serial.print("Setting to ");
          new_rpm = (((int)min_rpm)/10)*10;
          Serial.print(new_rpm);
          Serial.println(" RPM");
          fan.rpm(new_rpm);
          break;
        case 150:
          Serial.print("Setting to ");
          new_rpm = (((int)fan.max_rpm())/100)*50;
          Serial.print(new_rpm);
          Serial.println(" RPM");
          fan.rpm(new_rpm);
          state = 0;
          break;
      }
      ++state;
      Serial.print(fan.rpm());
      Serial.print(" RPM / ");
      Serial.print((fan.pwm_duty()/65535.0)*100.0);
      Serial.println("% duty");
      
    }  
}
