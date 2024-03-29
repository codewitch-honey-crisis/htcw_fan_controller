#ifdef ESP32
#include <fan_controller.hpp>
using namespace arduino;
#if defined(IRAM_ATTR)
IRAM_ATTR
#endif
void fan_controller::tick_counter(void* state) {
    tick_data* pdata = (tick_data*)state;
    if(++pdata->ticks==pdata->ticks_per_revolution) {
        pdata->last_update_ts_old = pdata->last_update_ts;
        pdata->last_update_ts = micros();
        pdata->ticks = 0;
    }
}

void fan_controller::do_move(fan_controller& rhs) {
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
    m_tach_pin = rhs.m_tach_pin;
    m_tick_data = rhs.m_tick_data;
    m_initialized = rhs.m_initialized;
}
fan_controller::fan_controller(fan_controller&& rhs) {
    do_move(rhs);
}
fan_controller& fan_controller::operator=(fan_controller&& rhs) {
    do_move(rhs);
    return *this;
}
fan_controller::fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, float max_rpm) : m_rpm(NAN), m_target_rpm(NAN), m_max_rpm(max_rpm),m_pwm_duty(0), m_pwm_callback(pwm_callback),m_pwm_callback_state(pwm_callback_state),m_tach_pin(-1),m_initialized(false) {

}
// configure for a 4-pin fan (with tach)
fan_controller::fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, float max_rpm, unsigned int ticks_per_revolution, float kp,float ki, float max_update_period_secs) : m_rpm(0), m_target_rpm(NAN), m_max_rpm(max_rpm), m_kp(kp),m_ki(ki),m_max_update_period_secs(max_update_period_secs), m_pwm_duty(0), m_pwm_callback(pwm_callback),m_pwm_callback_state(pwm_callback_state),m_tach_pin(tach_pin),m_initialized(false) {
    m_tick_data.last_update_ts = 0;
    m_tick_data.last_update_ts_old = 0;
    m_tick_data.ticks = 0;
    m_tick_data.ticks_per_revolution = ticks_per_revolution;
}
// initialize the library
bool fan_controller::initialize() {
    if(!m_initialized) {
        if(m_kp!=m_kp || m_ki!=m_ki) {
            return false;
        }
        if(m_max_rpm!=m_max_rpm || 0==m_max_rpm && m_tach_pin>-1) {
            m_max_rpm=find_max_rpm(m_pwm_callback,m_pwm_callback_state,m_tach_pin,m_tick_data.ticks_per_revolution);
            if(m_max_rpm==0) {
                return false;
            }
        }
        m_first = true;
        if(-1<m_tach_pin) {
            m_tick_data.ticks=0;
            m_rpm =0;
            m_last_update_ts = 0;
            attachInterruptArg(m_tach_pin,tick_counter,&m_tick_data,RISING);
            
        }
        m_tick_data.ticks=0;
        m_initialized = true;
    }
    return true;
}
// indicates whether or not the fan is initialized
bool fan_controller::initialized() const {
    return m_initialized;
}
        
// deinitalize the library
void fan_controller::deinitialize() {
    if(m_initialized) {
        if(m_tach_pin>-1) {
            detachInterrupt(m_tach_pin);
        }
        m_initialized = false;
    }
}
// retrieve the max RPM
float fan_controller::max_rpm() const {
    return m_max_rpm;
}
// retrieve the RPM
float fan_controller::rpm() const {
    return m_rpm;
}
// set the RPM
void fan_controller::rpm(float value) {
    if(value>m_max_rpm) {
        value = m_max_rpm;
    }
    m_target_rpm = value;
}
// retrieve the PWM duty
uint16_t fan_controller::pwm_duty() const {
    return m_pwm_duty;
}
// set the current PWM duty
void fan_controller::pwm_duty(uint16_t value) {
    m_pwm_duty = value;
    m_target_rpm = NAN;
    if(m_pwm_callback!=nullptr) {
        m_pwm_callback(m_pwm_duty,m_pwm_callback_state);
    }
}
// reports the RPM currently being targeted, or NAN
float fan_controller::target_rpm() const {
    return m_target_rpm;
}

// call in a loop to keep the fan updating
void fan_controller::update() {
    if(0>m_tach_pin) {
        if(m_target_rpm==m_target_rpm) {
            m_pwm_duty = ((float)m_target_rpm/(float)m_max_rpm)*65535+.5;
            if(m_pwm_callback!=nullptr) {
                m_pwm_callback(m_pwm_duty,m_pwm_callback_state);
            }
            m_target_rpm = NAN;
        }
        return;
    }
    if(m_tick_data.last_update_ts>=m_tick_data.last_update_ts_old && m_tick_data.last_update_ts_old!=0) {
        m_rpm = (60*1000.0*1000.0)/(m_tick_data.last_update_ts-m_tick_data.last_update_ts_old);
        if(micros()-m_tick_data.last_update_ts>(500*1000)) {
            m_rpm = 0.0;
        }
    }
    if(0!=m_max_update_period_secs && millis()<m_last_update_ts+(1000*m_max_update_period_secs)) {
        return;
    }
    m_last_update_ts = millis();
    if(m_first) {
        float pwm = (m_rpm/(float)m_max_rpm)*65535 ;
        m_xk1 = m_xk2 = pwm;
        m_y_out = 0;
        m_first = false;
    }
    if(m_target_rpm==m_target_rpm) {
        float target_pwm = (m_target_rpm/(float)m_max_rpm)*65535 ;
        if(target_pwm>65535) target_pwm = 65535;
        float pwm = (m_rpm/(float)m_max_rpm)*65535 ;
        if(pwm>65535) pwm = 65535;
        m_p_term = m_xk1 - pwm;
        m_p_term = m_kp * m_p_term;
        m_i_term = m_ki * (target_pwm - pwm);
        m_xk2 = m_xk1;
        m_xk1 = pwm;
        float deadband_delta= m_p_term + m_i_term;
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
            if(m_pwm_callback!=nullptr) {
                m_pwm_callback(m_pwm_duty,m_pwm_callback_state);
            }
        }
    }
}
// gets the kp value. Used for tuning the adaptive PI(D) algorithm
float fan_controller::kp() const {
    if(m_tach_pin>-1) {
        return m_kp;
    }
    return NAN;
}
// sets the kp value. Used for tuning the adaptive PI(D) algorithm
void fan_controller::kp(float value) {
    if(m_tach_pin>-1) {
        m_kp=value;
    }
}
// gets the ki value. Used for tuning the adaptive PI(D) algorithm
float fan_controller::ki() const {
    if(m_tach_pin>-1) {
        return m_ki;
    }
    return NAN;
}
// sets the ki value. Used for tuning the adaptive PI(D) algorithm
void fan_controller::ki(float value) {
    if(m_tach_pin>-1) {
        m_ki=value;
    }
}
// Find the maximum effective stable RPM. Must be called before initialize()
float fan_controller::find_max_rpm(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, unsigned int ticks_per_revolution) {
    if(pwm_callback!=nullptr) {
        pwm_callback(65535,nullptr);
    }
    delay(5000);
    tick_data data;
    data.last_update_ts = 0;
    data.last_update_ts_old = 0;
    data.ticks = 0;
    data.ticks_per_revolution = ticks_per_revolution;
    attachInterruptArg(tach_pin,tick_counter,&data,RISING);
    float rpm = 0, rpm_old = 0;
    static const int max_count = 20;
    float max_rpm = NAN;
    int zero_count = 0, count = 0;
    for(int tries = 0;tries<20;++tries) {
        if(data.last_update_ts>=data.last_update_ts_old && data.last_update_ts_old!=0) {
            float prev = rpm_old;
            rpm_old = rpm;
            
            rpm = (60*1000.0*1000.0)/(data.last_update_ts-data.last_update_ts_old);
            if(micros()-data.last_update_ts>(500*1000.0)) {
                rpm=0;
            }
            if(prev!=rpm) {
                continue;
            }
            if(rpm==0) {
                // exit if we're just getting zeroes
                if(++zero_count>=max_count*5) {
                    break;
                }
                continue;
            }
            if(++count=max_count) {
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
float fan_controller::find_min_rpm(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, unsigned int ticks_per_revolution, float response_delay_secs) {
    if(pwm_callback!=nullptr) {
        pwm_callback(0,nullptr);
    } else {
        return 0;
    }
    delay(1000);
    float rpm = NAN;
    tick_data data;
    data.last_update_ts = 0;
    data.last_update_ts_old = 0;
    data.ticks = 0;
    data.ticks_per_revolution = ticks_per_revolution;
    attachInterruptArg(tach_pin,tick_counter,&data,RISING);
    int pwm_duty;
    for(pwm_duty = 1;pwm_duty<65536;pwm_duty+=256) {
        if(pwm_callback!=nullptr) {
            pwm_callback(pwm_duty,nullptr);
        }
        for(int i = 0;i<10;++i) {
            if(data.last_update_ts>=data.last_update_ts_old && data.last_update_ts_old!=0) {
                rpm = (60*1000.0*1000.0)/(data.last_update_ts-data.last_update_ts_old);
                if(micros()-data.last_update_ts>(500*1000.0)) {
                    rpm=0;
                }
                if(rpm!=0) {
                    /*float old_rpm = 0;
                    for(--pwm_duty;pwm_duty>=0;--pwm_duty) {
                        if(pwm_callback!=nullptr) {
                            pwm_callback(pwm_duty,nullptr);
                        }
                        delay(1000*response_delay_secs);
                        for(int i = 0;i<10;++i) {
                            if(data.last_update_ts>=data.last_update_ts_old && data.last_update_ts_old!=0) {
                                old_rpm = rpm;
                                rpm = (60*1000.0)/(data.last_update_ts-data.last_update_ts_old);
                                if(millis()-data.last_update_ts>500) {
                                    rpm=0;
                                }
                                if(rpm==0) {
                                    detachInterrupt(tach_pin);
                                    return old_rpm;
                                }
                            }
                            delay(1000*response_delay_secs);
                        }
                    }
                    */
                    detachInterrupt(tach_pin);
                    return rpm;
                }
            }
            delay(1000*response_delay_secs);
        }
    }
    detachInterrupt(tach_pin);
    return 0;
}
#endif
