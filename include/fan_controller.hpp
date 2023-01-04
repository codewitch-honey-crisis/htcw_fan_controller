#pragma once
#include <Arduino.h>
#include <pid.h>

/*
Tuning
In general the gains of P, I, and D will need to be adjusted by the user in order to best servo the system. While there is not a static set of rules for what the values should be for any specific system, following the general procedures should help in tuning a circuit to match one’s system and environment. In general a PID circuit will typically overshoot the SP value slightly and then quickly damp out to reach the SP value.

Manual tuning of the gain settings is the simplest method for setting the PID controls. However, this procedure is done actively (the PID controller turned on and properly attached to the system) and requires some amount of experience to fully integrate. To tune your PID controller manually, first the integral and derivative gains are set to zero. Increase the proportional gain until you observe oscillation in the output. Your proportional gain should then be set to roughly half this value. After the proportional gain is set, increase the integral gain until any offset is corrected for on a time scale appropriate for your system. If you increase this gain too much, you will observe significant overshoot of the SP value and instability in the circuit. Once the integral gain is set, the derivative gain can then be increased. Derivative gain will reduce overshoot and damp the system quickly to the SP value. If you increase the derivative gain too much, you will see large overshoot (due to the circuit being too slow to respond). By playing with the gain settings, you can maximize the performance of your PID circuit, resulting in a circuit that quickly responds to changes in the system and effectively damps out oscillation about the SP value.

Control Type    Kp      Ki          Kd
P               0.50 Ku	-	        -
PI              0.45 Ku 1.2 Kp/Pu   -
PID	            0.60 Ku	2 Kp/Pu     KpPu/8
While manual tuning can be very effective at setting a PID circuit for your specific system, it does require some amount of experience and understanding of PID circuits and response. The Ziegler-Nichols method for PID tuning offers a bit more structured guide to setting PID values. Again, you’ll want to set the integral and derivative gain to zero. Increase the proportional gain until the circuit starts to oscillate. We will call this gain level Ku. The oscillation will have a period of Pu. Gains are for various control circuits are then given below in the chart.
*/

namespace arduino {
    typedef void (*fan_controller_pwm_callback)(uint8_t duty,void* state);
    class fan_controller final {
        struct tick_data {
            unsigned int ticks_per_revolution;
            volatile uint32_t last_update_ts;
            volatile uint32_t last_update_ts_old;
            volatile int ticks;
        };
        epid_t m_pid_ctx;
        bool m_first;
        float m_rpm;
        float m_target_rpm;
        float m_max_rpm;
        float m_kp;
        float m_ki;
        uint8_t m_pwm_duty;
        fan_controller_pwm_callback m_pwm_callback;
        void* m_pwm_callback_state;
        int16_t m_tach_pin;
        tick_data m_tick_data;
        bool m_initialized;
        #ifdef ESP32
        IRAM_ATTR
        #endif
        static void tick_counter(void* state);
        void do_move(fan_controller& rhs);
        fan_controller(const fan_controller& rhs)=delete;
        fan_controller& operator=(const fan_controller& rhs)=delete;
    public:
        // configure for a 3-pin fan (no tach)
        fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, float max_rpm);
        // configure for a 4-pin fan (with tach)
        fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, float max_rpm, unsigned int ticks_per_revolution = 2, float kp = 0.4f,float ki=0.4f);
        fan_controller(fan_controller&& rhs);
        fan_controller& operator=(fan_controller&& rhs);
        // initialize the library
        bool initialize();
        // retrieve the maximum RPM
        float max_rpm() const;
        // retrieve the RPM (NaN if not available)
        float rpm() const;
        // set the RPM
        void rpm(float value);
        // retrieve the PWM duty
        uint8_t pwm_duty() const;
        // set the PWM duty
        void pwm_duty(uint8_t value);
        // call in a loop to keep the fan updating
        void update();

        // Find the maximum effective stable RPM. Must be called before initialize()
        static float find_max_rpm(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, unsigned int ticks_per_revolution = 2);
        // Find the minimum effective stable RPM. Must be called before initialize()
        static float find_min_rpm(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, unsigned int ticks_per_revolution = 2,float response_delay_secs = .1);
    };
}