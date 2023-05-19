#pragma once
#include <Arduino.h>

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
typedef void (*fan_controller_pwm_callback)(uint16_t duty, void* state);
#ifdef ESP32
    class fan_controller final {
        struct tick_data {
            unsigned int ticks_per_revolution;
            volatile uint32_t last_update_ts;
            volatile uint32_t last_update_ts_old;
            volatile int ticks;
        };
        bool m_first;
        float m_rpm;
        float m_target_rpm;
        float m_max_rpm;
        float m_kp;
        float m_ki;
        float m_xk1;
        float m_xk2;
        float m_y_out;
        float m_p_term;
        float m_i_term;
        float m_max_update_period_secs;
        uint16_t m_pwm_duty;
        uint32_t m_last_update_ts;
        fan_controller_pwm_callback m_pwm_callback;
        void* m_pwm_callback_state;
        int16_t m_tach_pin;
        tick_data m_tick_data;
        bool m_initialized;
        #if defined(IRAM_ATTR)
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
        fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, float max_rpm, unsigned int ticks_per_revolution = 2, float kp = 0.4f,float ki=0.4f, float max_update_period_secs = .25);
        fan_controller(fan_controller&& rhs);
        fan_controller& operator=(fan_controller&& rhs);
        // initialize the library
        bool initialize();
        // deinitialize the library
        void deinitialize();
        // retrieve the maximum RPM
        float max_rpm() const;
        // retrieve the RPM (NaN if not available)
        float rpm() const;
        // set the RPM
        void rpm(float value);
        // retrieve the PWM duty
        uint16_t pwm_duty() const;
        // set the PWM duty
        void pwm_duty(uint16_t value);
        // reports the RPM currently being targeted, or NAN
        float target_rpm() const;
        // call in a loop to keep the fan updating
        void update();
        // gets the kp value. Used for tuning the adaptive PI(D) algorithm
        float kp() const;
        // sets the kp value. Used for tuning the adaptive PI(D) algorithm
        void kp(float value);
        // gets the ki value. Used for tuning the adaptive PI(D) algorithm
        float ki() const;
        // sets the ki value. Used for tuning the adaptive PI(D) algorithm
        void ki(float value);
        // Find the maximum effective stable RPM. Must be called before initialize()
        static float find_max_rpm(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, unsigned int ticks_per_revolution = 2);
        // Find the minimum effective stable RPM. Must be called before initialize()
        static float find_min_rpm(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, unsigned int ticks_per_revolution = 2,float response_delay_secs = .1);
};
#else // !ESP32
template <int16_t TachPin = -1, unsigned TicksPerRevolution = 2>
struct fan_controller final {
    static constexpr const int16_t tach_pin = TachPin;
    static constexpr const unsigned ticks_per_revolution = TicksPerRevolution;

   private:
    struct tick_data final {
        volatile uint32_t last_update_ts;
        volatile uint32_t last_update_ts_old;
        volatile int ticks;
    };
    bool m_first;
    float m_rpm;
    float m_target_rpm;
    float m_max_rpm;
    float m_kp;
    float m_ki;
    float m_xk1;
    float m_xk2;
    float m_y_out;
    float m_p_term;
    float m_i_term;
    float m_max_update_period_secs;
    uint16_t m_pwm_duty;
    uint32_t m_last_update_ts;
    fan_controller_pwm_callback m_pwm_callback;
    void* m_pwm_callback_state;
    static tick_data m_tick_data;
    bool m_initialized;
#if defined(IRAM_ATTR)
    IRAM_ATTR
#endif
    static void tick_counter() {
        if (++m_tick_data.ticks == ticks_per_revolution) {
            m_tick_data.last_update_ts_old = m_tick_data.last_update_ts;
            m_tick_data.last_update_ts = micros();
            m_tick_data.ticks = 0;
        }
    }
    void do_move(fan_controller& rhs) {
        m_first = rhs.m_first;
        m_rpm = rhs.m_rpm;
        m_target_rpm = rhs.m_target_rpm;
        m_max_rpm = rhs.m_max_rpm;
        m_kp = rhs.m_kp;
        m_ki = rhs.m_ki;
        m_xk1 = rhs.m_xk1;
        m_xk2 = rhs.m_xk2;
        m_y_out = rhs.m_y_out;
        m_p_term = rhs.m_p_term;
        m_i_term = rhs.m_i_term;
        m_max_update_period_secs = rhs.m_max_update_period_secs;
        m_pwm_duty = rhs.m_pwm_duty;
        m_last_update_ts = rhs.m_last_update_ts;
        m_pwm_callback = rhs.m_pwm_callback;
        m_pwm_callback_state = rhs.m_pwm_callback_state;
        m_initialized = rhs.m_initialized;
    }
    fan_controller(const fan_controller& rhs) = delete;
    fan_controller& operator=(const fan_controller& rhs) = delete;

   public:
    // configure for a 4-pin fan (with tach)
    fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, float max_rpm, unsigned int ticks_per_revolution = 2, float kp = 0.4f, float ki = 0.4f, float max_update_period_secs = .25) : m_rpm(0), m_target_rpm(NAN), m_max_rpm(max_rpm), m_kp(kp), m_ki(ki), m_max_update_period_secs(max_update_period_secs), m_pwm_duty(0), m_pwm_callback(pwm_callback), m_pwm_callback_state(pwm_callback_state), m_initialized(false) {
        m_tick_data.last_update_ts = 0;
        m_tick_data.last_update_ts_old = 0;
        m_tick_data.ticks = 0;
    }
    fan_controller(fan_controller&& rhs) {
        do_move(rhs);
    }
    fan_controller& operator=(fan_controller&& rhs) {
        do_move(rhs);
        return *this;
    }
    // initialize the library
    bool initialize() {
        if (!m_initialized) {
            if(m_kp!=m_kp || m_ki!=m_ki) {
                return false;
            }
            if (m_max_rpm != m_max_rpm || 0 == m_max_rpm) {
                m_max_rpm = find_max_rpm(m_pwm_callback, m_pwm_callback_state);
                if (m_max_rpm == 0) {
                    return false;
                }
            }
            m_first = true;
            m_tick_data.ticks = 0;
            m_rpm = 0;
            m_last_update_ts = 0;
            attachInterrupt(tach_pin, tick_counter, RISING);
            m_tick_data.ticks = 0;
            m_initialized = true;
        }
        return true;
    }
    // deinitialize the library
    void deinitialize() {
        if (m_initialized) {
            detachInterrupt(tach_pin);
            m_initialized = false;
        }
    }
    // retrieve the maximum RPM
    float max_rpm() const {
        return m_max_rpm;
    }
    // retrieve the RPM (NaN if not available)
    float rpm() const {
        return m_rpm;
    }
    // set the RPM
    void rpm(float value) {
        if (value > m_max_rpm) {
            value = m_max_rpm;
        }
        m_target_rpm = value;
    }
    // retrieve the PWM duty
    uint16_t pwm_duty() const {
        return m_pwm_duty;
    }
    // set the PWM duty
    void pwm_duty(uint16_t value) {
        m_pwm_duty = value;
        m_target_rpm = NAN;
        if (m_pwm_callback != nullptr) {
            m_pwm_callback(m_pwm_duty, m_pwm_callback_state);
        }
    }
    // reports the RPM currently being targeted, or NAN
    float target_rpm() const {
        return m_target_rpm;
    }
    // call in a loop to keep the fan updating
    void update() {
        if (m_tick_data.last_update_ts >= m_tick_data.last_update_ts_old && m_tick_data.last_update_ts_old != 0) {
            m_rpm = (60 * 1000.0 * 1000.0) / (m_tick_data.last_update_ts - m_tick_data.last_update_ts_old);
            if (micros() - m_tick_data.last_update_ts > 500 * 1000) {
                m_rpm = 0.0;
            }
        }
        if (0 != m_max_update_period_secs && millis() < m_last_update_ts + (1000 * m_max_update_period_secs)) {
            return;
        }
        m_last_update_ts = millis();
        if (m_first) {
            float pwm = (m_rpm / (float)m_max_rpm) * 65535;
            m_xk1 = m_xk2 = pwm;
            m_y_out = 0;
            m_first = false;
        }
        if (m_target_rpm == m_target_rpm) {
            float target_pwm = (m_target_rpm / (float)m_max_rpm) * 65535;
            if (target_pwm > 65535)
                target_pwm = 65535;
            float pwm = (m_rpm / (float)m_max_rpm) * 65535;
            if (pwm > 65535)
                pwm = 65535;
            m_p_term = m_xk1 - pwm;
            m_p_term = m_kp * m_p_term;
            m_i_term = m_ki * (target_pwm - pwm);
            m_xk2 = m_xk1;
            m_xk1 = pwm;        
            float deadband_delta = m_p_term + m_i_term;
            if ((deadband_delta != deadband_delta) || (fabsf(deadband_delta) >= 0)) {
                const float y_prev = m_y_out;
                m_y_out += m_p_term + m_i_term;
                if(m_y_out!=m_y_out || m_p_term!=m_p_term||m_i_term!=m_i_term) {
                    m_y_out = y_prev;
                }
                if(m_y_out>65535) {
                    m_y_out = 65535;
                } else if(m_y_out<0) {
                    m_y_out = 0;
                }
                m_pwm_duty = lround(m_y_out);
                if (m_pwm_callback != nullptr) {
                    m_pwm_callback(m_pwm_duty, m_pwm_callback_state);
                }
            }
        }
    }

    // Find the maximum effective stable RPM. Must be called before initialize()
    static float find_max_rpm(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state) {
        if (pwm_callback != nullptr) {
            pwm_callback(65535, nullptr);
        }
        delay(5000);
        m_tick_data.last_update_ts = 0;
        m_tick_data.last_update_ts_old = 0;
        m_tick_data.ticks = 0;
        attachInterrupt(tach_pin, tick_counter, RISING);
        float rpm = 0, rpm_old = 0;
        static const int max_count = 20;
        float max_rpm = NAN;
        int zero_count = 0, count = 0;
        for (int tries = 0; tries < 20; ++tries) {
            if (m_tick_data.last_update_ts >= m_tick_data.last_update_ts_old && m_tick_data.last_update_ts_old != 0) {
                float prev = rpm_old;
                rpm_old = rpm;

                rpm = (60 * 1000.0 * 1000.0) / (m_tick_data.last_update_ts - m_tick_data.last_update_ts_old);
                if (micros() - m_tick_data.last_update_ts > 500 * 1000) {
                    rpm = 0;
                }
                if (prev != rpm) {
                    continue;
                }
                if (rpm == 0) {
                    // exit if we're just getting zeroes
                    if (++zero_count >= max_count * 5) {
                        break;
                    }
                    continue;
                }
                if (++count = max_count) {
                    detachInterrupt(tach_pin);
                    return rpm;
                }
            }
            delay(100);
        }
        detachInterrupt(tach_pin);
        return 0;
    }
    // Find the minimum effective stable RPM. Must be called before initialize()
    static float find_min_rpm(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, float response_delay_secs = .1) {
        if (pwm_callback != nullptr) {
            pwm_callback(0, nullptr);
        } else {
            return 0;
        }
        delay(1000);
        float rpm = NAN;
        m_tick_data.last_update_ts = 0;
        m_tick_data.last_update_ts_old = 0;
        m_tick_data.ticks = 0;
        
        attachInterrupt(tach_pin, tick_counter, RISING);
        int pwm_duty;
        for (pwm_duty = 1 << 8; pwm_duty < 65536; pwm_duty += (1 << 8)) {
            if (pwm_callback != nullptr) {
                pwm_callback(pwm_duty, nullptr);
            }
            for (int i = 0; i < 10; ++i) {
                if (m_tick_data.last_update_ts >= m_tick_data.last_update_ts_old && m_tick_data.last_update_ts_old != 0) {
                    rpm = (60 * 1000.0 * 1000.0) / (m_tick_data.last_update_ts - m_tick_data.last_update_ts_old);
                    if (micros() - (m_tick_data.last_update_ts) > 500 * 1000) {
                        rpm = 0;
                    }
                    if (rpm != 0) {
                        detachInterrupt(tach_pin);
                        return rpm;
                    }
                }
                delay(1000 * response_delay_secs);
            }
        }
        detachInterrupt(tach_pin);
        return 0;
    }
};
template <unsigned TicksPerRevolution> 
struct fan_controller<-1,TicksPerRevolution> final {
    
   private:
    float m_rpm;
    float m_target_rpm;
    float m_max_rpm;
    uint16_t m_pwm_duty;
    fan_controller_pwm_callback m_pwm_callback;
    void* m_pwm_callback_state;
    bool m_initialized;
    void do_move(fan_controller& rhs) {
        m_rpm = rhs.m_rpm;
        m_target_rpm = rhs.m_target_rpm;
        m_max_rpm = rhs.m_max_rpm;
        m_pwm_duty = rhs.m_pwm_duty;
        m_pwm_callback = rhs.m_pwm_callback;
        m_pwm_callback_state = rhs.m_pwm_callback_state;
        m_initialized = rhs.m_initialized;
    }
    fan_controller(const fan_controller& rhs) = delete;
    fan_controller& operator=(const fan_controller& rhs) = delete;

   public:
    // configure for a 3-pin fan (with no tach)
    fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, float max_rpm) : m_rpm(0), m_target_rpm(NAN), m_max_rpm(max_rpm), m_pwm_duty(0), m_pwm_callback(pwm_callback), m_pwm_callback_state(pwm_callback_state), m_initialized(false) {
    }
    fan_controller(fan_controller&& rhs) {
        do_move(rhs);
    }
    fan_controller& operator=(fan_controller&& rhs) {
        do_move(rhs);
        return *this;
    }
    // initialize the library
    bool initialize() {
        if (!m_initialized) {
            m_rpm = 0;
            m_pwm_duty = 0;
            m_initialized = true;
        }
        return true;
    }
    // deinitialize the library
    void deinitialize() {
        m_initialized = false;
    }
    // retrieve the maximum rated RPM
    float max_rpm() const {
        return m_max_rpm;
    }
    // retrieve the approx RPM
    float rpm() const {
        return (m_pwm_duty/65535.0)*m_max_rpm;
    }
    // set the approx RPM
    void rpm(float value) {
        if (value > m_max_rpm) {
            value = m_max_rpm;
        }
        m_target_rpm = value;
    }
    // retrieve the PWM duty
    uint16_t pwm_duty() const {
        return m_pwm_duty;
    }
    // set the PWM duty
    void pwm_duty(uint16_t value) {
        m_pwm_duty = value;
        m_target_rpm = NAN;
        if (m_pwm_callback != nullptr) {
            m_pwm_callback(m_pwm_duty, m_pwm_callback_state);
        }
    }
    // reports the RPM currently being targeted, or NAN
    float target_rpm() const {
        return m_target_rpm;
    }
    // call in a loop to keep the fan updating
    void update() {
        if (m_target_rpm == m_target_rpm) {
            m_pwm_duty = ((float)m_target_rpm / (float)m_max_rpm) * 65535 + .5;
            if (m_pwm_callback != nullptr) {
                m_pwm_callback(m_pwm_duty, m_pwm_callback_state);
            }
            m_target_rpm = NAN;
        }
    }
    // gets the kp value. Used for tuning the adaptive PI(D) algorithm
    float kp() const {
        if(tach_pin>-1) {
            return m_kp;
        }
        return NAN;
    }
    // sets the kp value. Used for tuning the adaptive PI(D) algorithm
    void kp(float value) {
        if(tach_pin>-1) {
            m_kp=value;
        }
    }
    // gets the ki value. Used for tuning the adaptive PI(D) algorithm
    float ki() const {
        if(tach_pin>-1) {
            return m_ki;
        }
        return NAN;
    }
    // sets the ki value. Used for tuning the adaptive PI(D) algorithm
    void ki(float value) {
        if(tach_pin>-1) {
            m_ki=value;
        }
    }
};
template <int16_t TachPin, unsigned TicksPerRevolution>
typename fan_controller<TachPin,TicksPerRevolution>::tick_data
fan_controller<TachPin,TicksPerRevolution>::m_tick_data = {0};
#endif
}
