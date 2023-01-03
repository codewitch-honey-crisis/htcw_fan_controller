#include <fan_controller.hpp>
using namespace arduino;
#ifdef ESP32
IRAM_ATTR
#endif
void fan_controller::tick_counter(void* state) {
    fan_controller* pthis = (fan_controller*)state;
    if(++pthis->m_ticks==pthis->m_ticks_per_revolution) {
        pthis->m_last_update_ts_old = pthis->m_last_update_ts;
        pthis->m_last_update_ts = millis();
        pthis->m_ticks = 0;
    }
}

void fan_controller::do_move(fan_controller& rhs) {
    m_pid_ctx = rhs.m_pid_ctx;
    m_first = rhs.m_first;
    m_rpm = rhs.m_rpm;
    m_target_rpm = rhs.m_target_rpm;
    m_last_update_ts = rhs.m_last_update_ts;
    m_last_update_ts_old = rhs.m_last_update_ts_old;
    m_max_rpm = rhs.m_max_rpm;
    m_ticks_per_revolution = rhs.m_ticks_per_revolution;
    m_kp = rhs.m_kp;
    m_ki = rhs.m_ki;
    m_kd = rhs.m_kd;
    m_pwm_duty = rhs.m_pwm_duty;
    m_pwm_callback = rhs.m_pwm_callback;
    m_pwm_callback_state = rhs.m_pwm_callback_state;
    m_tach_pin = rhs.m_tach_pin;
    m_ticks = rhs.m_ticks;
    m_initialized = rhs.m_initialized;
}
fan_controller::fan_controller(fan_controller&& rhs) {
    do_move(rhs);
}
fan_controller& fan_controller::operator=(fan_controller&& rhs) {
    do_move(rhs);
    return *this;
}
fan_controller::fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, unsigned int max_rpm) : m_rpm(NAN), m_target_rpm(NAN), m_last_update_ts(0), m_last_update_ts_old(0), m_max_rpm(max_rpm),m_ticks_per_revolution(0), m_kp(0),m_ki(0),m_kd(0),m_pwm_duty(0), m_pwm_callback(pwm_callback),m_pwm_callback_state(pwm_callback_state),m_tach_pin(-1),m_initialized(false) {

}
// configure for a 4-pin fan (with tach)
fan_controller::fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, unsigned int max_rpm, unsigned int ticks_per_revolution, float kp,float ki,float kd) : m_rpm(0), m_target_rpm(NAN), m_last_update_ts(0),m_last_update_ts_old(0), m_max_rpm(max_rpm),m_ticks_per_revolution(ticks_per_revolution), m_kp(kp),m_ki(ki),m_kd(kd),m_pwm_duty(0), m_pwm_callback(pwm_callback),m_pwm_callback_state(pwm_callback_state),m_tach_pin(tach_pin),m_initialized(false) {
}
// initialize the library
bool fan_controller::initialize() {
    m_first = true;
    if(!m_initialized) {
        if(-1<m_tach_pin) {
            m_ticks=0;
            m_rpm =0;
            attachInterruptArg(m_tach_pin,tick_counter,this,RISING);
        }
        m_ticks=0;
        m_initialized = true;
    }
    return true;
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
uint8_t fan_controller::pwm_duty() const {
    return m_pwm_duty;
}
// set the current PWM duty
void fan_controller::pwm_duty(uint8_t value) {
    m_pwm_duty = value;
    m_target_rpm = NAN;
    if(m_pwm_callback!=nullptr) {
        m_pwm_callback(m_pwm_duty,m_pwm_callback_state);
    }
}
// call in a loop to keep the fan updating
void fan_controller::update() {
    if(0>m_tach_pin) {
        if(m_target_rpm==m_target_rpm) {
            m_pwm_duty = ((float)m_target_rpm/(float)m_max_rpm)*255+.5;
            if(m_pwm_callback!=nullptr) {
                m_pwm_callback(m_pwm_duty,m_pwm_callback_state);
            }
            m_target_rpm = NAN;
        }
        return;
    }
    
    if(m_last_update_ts>=m_last_update_ts_old && m_last_update_ts_old!=0) {
        m_rpm = (60*1000.0)/(m_last_update_ts-m_last_update_ts_old);
        if(millis()-m_last_update_ts>125) {
            m_rpm = 0.0;
        }
    }
    if(m_first) {
        float pwm = (m_rpm/(float)m_max_rpm)*255 ;
        if(EPID_ERR_NONE!=epid_init(&m_pid_ctx,pwm,pwm,0,m_kp,m_ki,m_kd)) {
            return;
        }
        m_first = false;
    }
    if(m_target_rpm==m_target_rpm) {
        float target_pwm = (m_target_rpm/(float)m_max_rpm)*255 ;
        if(target_pwm>255) target_pwm = 255;
        float pwm = (m_rpm/(float)m_max_rpm)*255 ;
        if(pwm>255) pwm = 255;
        
        epid_pid_calc(&m_pid_ctx,target_pwm, pwm);
        float deadband_delta= m_pid_ctx.p_term + m_pid_ctx.i_term + m_pid_ctx.d_term;
        if ((deadband_delta != deadband_delta) || (fabsf(deadband_delta) >= 0)) {
            epid_pid_sum(&m_pid_ctx, 0, 255);
            m_pwm_duty = lround(m_pid_ctx.y_out);
            if(m_pwm_callback!=nullptr) {
                m_pwm_callback(m_pwm_duty,m_pwm_callback_state);
            }
        }
    }
}
