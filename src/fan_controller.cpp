#include <fan_controller.hpp>
using namespace arduino;
#ifdef ESP32
IRAM_ATTR
#endif
void fan_controller::tick_counter(void* state) {
    fan_controller* pthis = (fan_controller*)state;
    ++pthis->m_ticks;
}
void fan_controller::do_move(fan_controller& rhs) {
    m_pid_ctx = rhs.m_pid_ctx;
    m_rpm = rhs.m_rpm;
    m_target_rpm = rhs.m_target_rpm;
    m_last_update_ts = rhs.m_last_update_ts;
    m_max_rpm = rhs.m_max_rpm;
    m_ticks_per_revolution = rhs.m_ticks_per_revolution;
    m_period_secs = rhs.m_period_secs;
    m_kp = rhs.m_kp;
    m_ki = rhs.m_ki;
    m_kd = rhs.m_kd;
    m_pwm_callback = rhs.m_pwm_callback;
    m_pwm_callback_state = rhs.m_pwm_callback_state;
    m_tach_pin = rhs.m_tach_pin;
    m_ticks = rhs.m_ticks;
}
fan_controller::fan_controller(fan_controller&& rhs) {
    do_move(rhs);
}
fan_controller& fan_controller::operator=(fan_controller&& rhs) {
    do_move(rhs);
    return *this;
}
fan_controller::fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, unsigned int max_rpm) : m_rpm(-1), m_target_rpm(-1), m_last_update_ts(0), m_max_rpm(max_rpm),m_ticks_per_revolution(0), m_period_secs(0), m_kp(0),m_ki(0),m_kd(0),m_pwm_callback(pwm_callback),m_pwm_callback_state(pwm_callback_state),m_tach_pin(-1), m_ticks(-1) {

}
// configure for a 4-pin fan (with tach)
fan_controller::fan_controller(fan_controller_pwm_callback pwm_callback, void* pwm_callback_state, uint8_t tach_pin, unsigned int max_rpm, unsigned int ticks_per_revolution, float period_secs, float kp,float ki,float kd) : m_rpm(-1), m_target_rpm(-1), m_last_update_ts(0), m_max_rpm(max_rpm),m_ticks_per_revolution(ticks_per_revolution), m_period_secs(period_secs), m_kp(kp),m_ki(ki),m_kd(kd),m_pwm_callback(pwm_callback),m_pwm_callback_state(pwm_callback_state),m_tach_pin(tach_pin), m_ticks(-1) {
}
// initialize the library
bool fan_controller::initialize() {
    if(0>m_ticks) {
        if(-1<m_tach_pin) {
            if(EPID_ERR_NONE!=epid_init(&m_pid_ctx,0,0,0,m_kp,m_ki,m_kd)) {
                return false;
            }
            pinMode(m_tach_pin,INPUT_PULLDOWN);
            m_ticks=0;
            attachInterruptArg(m_tach_pin,tick_counter,this,RISING);
        }
        m_ticks=0;
    }
    return true;
}
// retrieve the RPM
int fan_controller::rpm() const {
    return m_rpm;
}
// set the RPM
void fan_controller::rpm(unsigned int value) {
    if(value>m_max_rpm) {
        value = m_max_rpm;
    }
    m_target_rpm = value;
}
// call in a loop to keep the fan updating
void fan_controller::update() {
    if(0>m_tach_pin) {
        if(m_target_rpm!=-1) {
            if(m_pwm_callback!=nullptr) {
                m_pwm_callback(((float)m_target_rpm/(float)m_max_rpm)*255+.5,m_pwm_callback_state);
            }
            m_target_rpm = -1;
        }
        return;
    }
    uint32_t ms = millis();
    if(ms>m_last_update_ts+(m_period_secs*1000+.5)) {
        m_last_update_ts = ms;
        m_rpm = (60.0/m_period_secs) *(m_ticks/m_ticks_per_revolution);
        m_ticks = 0;
        if(m_target_rpm!=-1) {
            epid_pid_calc(&m_pid_ctx, m_target_rpm, m_rpm);
            float deadband_delta= m_pid_ctx.p_term + m_pid_ctx.i_term + m_pid_ctx.d_term;
            if ((deadband_delta != deadband_delta) || (fabsf(deadband_delta) >= 0)) {
                epid_pid_sum(&m_pid_ctx, m_target_rpm, m_max_rpm);
                if(m_pwm_callback!=nullptr) {
                    const int tmp = lroundf(m_pid_ctx.y_out);
                    const uint8_t duty = (uint8_t)(((float)tmp/(float)m_max_rpm)*255+.5);
                    m_pwm_callback(duty,m_pwm_callback_state);
                }
            }
        }
    }
}