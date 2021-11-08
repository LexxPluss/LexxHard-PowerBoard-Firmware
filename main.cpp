#include "mbed.h"
#include "serial_message.hpp"

namespace {

#undef DEBUG
#ifdef DEBUG
BufferedSerial debugserial{PA_2, PA_3};
FILE *debugout{fdopen(&debugserial, "r+")};
#define LOG(...) fprintf(debugout, __VA_ARGS__)
#else
#define LOG(...) [](){}() // lambda empty function
#endif

EventQueue globalqueue;

class can_callback {
public:
    void register_callback(uint32_t msgid, Callback<void(const CANMessage &msg)> func) {
        if (count < NUM) {
            callbacks[count].msgid = msgid;
            callbacks[count].func = func;
            ++count;
        }
    }
    bool call_callbacks(const CANMessage &msg) const {
        for (int i{0}; i < count; ++i) {
            if (msg.id == callbacks[i].msgid) {
                callbacks[i].func(msg);
                return true;
            }
        }
        return false;
    }
private:
    int count{0};
    static const int NUM{8};
    struct {
        uint32_t msgid;
        Callback<void(const CANMessage &msg)> func;
    } callbacks[NUM];
};

class can_driver {
public:
    can_driver() {
        can.frequency(500000);
    }
    void poll() {
        CANMessage msg;
        if (can.read(msg) != 0 && msg.type == CANData)
            callback.call_callbacks(msg);
    }
    void register_callback(uint32_t msgid, Callback<void(const CANMessage &msg)> func) {
        can.filter(msgid, 0x000007ffu, CANStandard, filter_handle);
        callback.register_callback(msgid, func);
        filter_handle = (filter_handle + 1) % 14; // STM CAN filter size
    }
    void send(const CANMessage &msg) {
        can.write(msg);
    }
private:
    CAN can{PA_11, PA_12};
    can_callback callback;
    int filter_handle{0};
};

class power_switch {
public:
    enum class STATE {
        RELEASED, PUSHED, LONG_PUSHED,
    };
    void poll() {
        int now{sw.read()};
        if (prev != now) {
            LOG("power_switch change to %d\n", now);
            prev = now;
            timer.reset();
            timer.start();
        } else if (now == 0) {
            auto elapsed{timer.elapsed_time()};
            if (elapsed > 10s) {
                if (state != STATE::LONG_PUSHED)
                    state = STATE::LONG_PUSHED;
            } else if (elapsed > 3s) {
                if (state == STATE::RELEASED)
                    state = STATE::PUSHED;
            }
        }
    }
    void reset_state() {
        timer.stop();
        timer.reset();
        state = STATE::RELEASED;
    }
    STATE get_state() const {return state;}
    bool get_raw_state() {
        return sw.read() == 0;
    }
    void set_led(bool enabled) {
        led.write(enabled ? 1 : 0);
    }
private:
    DigitalIn sw{PB_0};
    DigitalOut led{PB_13, 0};
    Timer timer;
    STATE state{STATE::RELEASED};
    int prev{-1};
};

class bumper_switch {
public:
    void poll() {
        if (left.read() == 0 || right.read() == 0) {
            asserted = true;
            timeout.attach(callback(this, &bumper_switch::assert_timeout), 1s);
        }
    }
    void get_raw_state(bool &left, bool &right) const {
        left = right = asserted;
#ifdef DEBUG
        static bool prev{false};
        if (prev != asserted) {
            prev = asserted;
            LOG("BUMPER %s!\n", asserted ? "ON" : "OFF");
        }
#endif
    }
private:
    void assert_timeout() {asserted = false;}
    DigitalIn left{PA_4}, right{PA_5};
    Timeout timeout;
    bool asserted{false};
};

class emergency_switch {
public:
    bool asserted() {return left.read() == 1 || right.read() == 1;}
    void get_raw_state(bool &left, bool &right) {
        left = this->left.read() == 1;
        right = this->right.read() == 1;
    }
private:
    DigitalIn left{PA_6}, right{PA_7};
};

class wheel_switch {
public:
    void set_disable(bool disable) {
        if (disable) {
            left.write(1);
            right.write(1);
        } else {
            left.write(0);
            right.write(0);
        }
    }
    void get_raw_state(bool &left, bool &right) {
        left = this->left.read() == 1;
        right = this->right.read() == 1;
    }
private:
    DigitalOut left{PB_8, 0}, right{PB_9, 0};
};

class manual_charger {
public:
    void init() {
        setup_first_state();
    }
    void poll() {
        int now{din.read()};
        if (prev != now) {
            prev = now;
            timer.reset();
            timer.start();
        } else {
            auto elapsed{timer.elapsed_time()};
            if (elapsed > 100ms) {
                plugged = now == 0;
                timer.stop();
                timer.reset();
            }
        }
    }
    bool is_plugged() {return plugged;}
private:
    void setup_first_state() {
        int plugped_count{0};
        for (int i{0}; i < 10; ++i) {
            if (din.read() == 0)
                ++plugped_count;
            ThisThread::sleep_for(5ms);
        }
        plugged = plugped_count > 5;
    }
    DigitalIn din{PB_10};
    Timer timer;
    int prev{1};
    bool plugged{false};
};

class auto_charger {
public:
    auto_charger(I2C &i2c) : i2c(i2c) {}
    void init() {
#ifndef DEBUG
        serial.set_baud(4800);
        serial.set_format(8, SerialBase::None, 1);
        serial.set_blocking(false);
#endif
        globalqueue.call_every(1s, this, &auto_charger::poll_1s);
        heartbeat_timer.start();
        serial_timer.start();
    }
    bool is_docked() const {
        return is_connected() && heartbeat_timer.elapsed_time() < 5s;
    }
    void set_enable(bool enable) {
        if (enable) {
            sw.write(1);
            voltage_timer.reset();
            voltage_timer.start();
        } else {
            sw.write(0);
            voltage_timer.stop();
            voltage_timer.reset();
        }
    }
    bool is_connector_overheat() const {
        return connector_temp[0] > 80.0f || connector_temp[1] > 80.0f;
    }
    bool is_charger_stopped() const {
        return voltage_timer.elapsed_time() > 10s;
    }
    void get_connector_temperature(int &positive, int &negative) const {
        positive = connector_temp[0];
        negative = connector_temp[1];
    }
    void poll() {
        connector_v = connector.read_voltage();
        if (connector_v > CHARGING_VOLTAGE * 0.5f)
            voltage_timer.reset();
#ifndef DEBUG
        while (serial.readable()) {
            if (serial_timer.elapsed_time() > 1s)
                msg.reset();
            serial_timer.reset();
            uint8_t data;
            serial.read(&data, 1);
            if (msg.decode(data)) {
                uint8_t param[3];
                uint8_t command{msg.get_command(param)};
                if (command == serial_message::HEARTBEAT && param[0] == heartbeat_counter)
                    heartbeat_timer.reset();
            }
        }
#endif
        adc_ticktock();
    }
private:
    void adc_ticktock() {
        if (adc_measure_mode) {
            adc_read();
            adc_ch = adc_ch == 0 ? 1 : 0;
            adc_measure_mode = false;
        } else {
            adc_measure();
            adc_measure_mode = true;
        }
    }
    void adc_read() {
        uint8_t buf[2];
        buf[0] = 0b00000000; // Conversion Register
        if (i2c.write(ADDR, reinterpret_cast<const char*>(buf), 1) == 0 &&
            i2c.read(ADDR, reinterpret_cast<char*>(buf), 2) == 0) {
            int16_t value{static_cast<int16_t>((buf[0] << 8) | buf[1])};
            float voltage{static_cast<float>(value) / 32768.0f * 4.096f};
            calculate_temperature(voltage);
        }
    }
    void adc_measure() const {
        uint8_t buf[3];
        buf[0] = 0b00000001; // Config Register
        buf[1] = 0b10000011; // Start, FSR4.096V, Single
        buf[2] = 0b10000011; // 128SPS
        switch (adc_ch) {
        default:
        case 0: buf[1] |= 0b01000000; break;
        case 1: buf[1] |= 0b01010000; break;
        case 2: buf[1] |= 0b01100000; break;
        case 3: buf[1] |= 0b01110000; break;
        }
        i2c.write(ADDR, reinterpret_cast<const char*>(buf), sizeof buf);
    }
    void calculate_temperature(float adc_voltage) {
        if (adc_voltage > 3.29999f)
            adc_voltage = 3.29999f;
        if (adc_voltage < 0.0f)
            adc_voltage = 0.0f;
        // see https://lexxpluss.esa.io/posts/459
        static constexpr float R0{3300.0f}, B{70.0f}, T0{373.0f};
        float Rpu{adc_ch < 2 ? 27000.0f : 10000.0f};
        float R{Rpu * adc_voltage / (3.3f - adc_voltage)};
        float T{1.0f / (logf(R / R0) / B + 1.0f / T0)};
        connector_temp[adc_ch] = T - 273.0f;
    }
    bool is_connected() const {
        return connector_v > CONNECT_VOLTAGE * 0.5f;
    }
    void poll_1s() {
        if (is_connected())
            send_heartbeat();
    }
    void send_heartbeat() {
        uint8_t buf[8], param[3]{++heartbeat_counter, static_cast<uint8_t>(sw.read())};
        serial_message::compose(buf, serial_message::HEARTBEAT, param);
#ifndef DEBUG
        serial.write(buf, sizeof buf);
#endif
    }
    I2C &i2c;
#ifndef DEBUG
    BufferedSerial serial{PA_2, PA_3};
#endif
    AnalogIn connector{PB_1, 3.3f};
    DigitalOut sw{PB_2, 0};
    Timer heartbeat_timer, voltage_timer, serial_timer;
    serial_message msg;
    uint8_t heartbeat_counter{0};
    float connector_v{0.0f}, connector_temp[2]{0.0f, 0.0f};
    int adc_ch{0};
    bool adc_measure_mode{false};
    static constexpr int ADDR{0b10010010};
    static constexpr float CHARGING_VOLTAGE{30.0f * 1000.0f / (9100.0f + 1000.0f)},
                           CONNECT_VOLTAGE{3.3f * 1000.0f / (9100.0f + 1000.0f)};
};

class bmu_controller {
public:
    bmu_controller(can_driver &can) : can(can) {}
    void init() {
        for (auto i : {0x100, 0x101, 0x113})
            can.register_callback(i, callback(this, &bmu_controller::handle_can));
    }
    void set_enable(bool enable) {main_sw = enable ? 1 : 0;}
    bool is_ok() const {
        return ((data.mod_status1 & 0b10111111) == 0 ||
                (data.mod_status2 & 0b11100001) == 0 ||
                (data.bmu_alarm1  & 0b11111111) == 0 ||
                (data.bmu_alarm2  & 0b00000001) == 0);
    }
    void get_fet_state(bool &c_fet, bool &d_fet, bool &p_dsg) {
        c_fet = this->c_fet.read() == 1;
        d_fet = this->d_fet.read() == 1;
        p_dsg = this->p_dsg.read() == 1;
    }
    bool is_full_charge() const {
        return (data.mod_status1 & 0b01000000) != 0;
    }
    bool is_chargable() const {
        return (data.mod_status1 & 0b01000000) == 0 && data.rsoc < 95;
    }
private:
    void handle_can(const CANMessage &msg) {
        switch (msg.id) {
        case 0x100:
            data.mod_status1 = msg.data[0];
            data.asoc = msg.data[2];
            data.rsoc = msg.data[3];
            // LOG("asoc:%u rsoc:%u soh:%u st:0x%02x\n", msg.data[2], msg.data[3], msg.data[4], msg.data[0]);
            break;
        case 0x101:
            data.mod_status2 = msg.data[6];
#if 0
            {
                int16_t pack_a{(msg.data[0]) << 8 | msg.data[1]};
                uint16_t charge_a{(msg.data[2]) << 8 | msg.data[3]};
                uint16_t pack_v{(msg.data[4]) << 8 | msg.data[5]};
                LOG("pack_a:%d charge_a:%u v:%u\n", pack_a, charge_a, pack_v);
            }
#endif
            break;
        case 0x113:
            data.bmu_alarm1 = msg.data[4];
            data.bmu_alarm2 = msg.data[5];
            break;
        }
    }
    can_driver &can;
    DigitalOut main_sw{PB_11, 0};
    DigitalIn c_fet{PB_14}, d_fet{PB_15}, p_dsg{PA_9};
    struct {
        uint8_t mod_status1{0xff}, mod_status2{0xff}, bmu_alarm1{0xff}, bmu_alarm2{0xff};
        uint8_t asoc{0}, rsoc{0};
    } data;
};

class temperature_sensor {
public:
    temperature_sensor(I2C &i2c) : i2c(i2c) {}
    void init() {
        uint8_t buf[2];
        buf[0] = 0x0b; // ID Register
        if (i2c.write(ADDR, reinterpret_cast<const char*>(buf), 1, true) == 0 &&
            i2c.read(ADDR, reinterpret_cast<char*>(buf), 1) == 0 &&
            (buf[0] & 0b11111000) == 0b11001000) {
            buf[0] = 0x03; // Configuration Register
            buf[1] = 0b10000000; // 16bit
            i2c.write(ADDR, reinterpret_cast<const char*>(buf), 2);
        }
    }
    bool is_ok() const {
        return temperature < 80.0f;
    }
    int get_temperature() const {
        if (temperature > 127.0f)
            return 127;
        else if (temperature < -50.0f)
            return -50;
        else
            return temperature;
    }
    void poll() {
        uint8_t buf[2];
        buf[0] = 0x00; // Temperature Value MSB Register
        if (i2c.write(ADDR, reinterpret_cast<const char*>(buf), 1, true) == 0 &&
            i2c.read(ADDR, reinterpret_cast<char*>(buf), 2) == 0) {
            int16_t value{static_cast<int16_t>((buf[0] << 8) | buf[1])};
            temperature = value / 128.0f;
        }
    }
private:
    I2C &i2c;
    float temperature{0.0f};
    static constexpr int ADDR{0b10010000};
};

class dcdc_converter {
public:
    void set_enable(bool enable) {
        if (enable) {
            control[0].write(1); // 5V must be turned on first.
            control[1].write(1);
        } else {
            control[1].write(0); // 16V must be turned off first.
            control[0].write(0);
        }
    }
    bool is_ok() {
        return fail[0].read() != 0 && fail[1].read() != 0;
    }
    void get_failed_state(bool &v5, bool &v16) {
        v5 = fail[0].read() == 0;
        v16 = fail[1].read() == 0;
    }
private:
    DigitalOut control[2]{{PA_10, 0}, {PB_3, 0}};
    DigitalIn fail[2]{{PA_15}, {PB_4}};
};

class fan_driver {
public:
    void init() {
        pwm.period_us(1000000 / CONTROL_HZ);
        pwm.pulsewidth_us(0);
    }
    void control_by_temperature(float temperature) {
        static constexpr float temp_min{15.0f}, temp_l{30.0f}, temp_h{50.0f};
        static constexpr int duty_l{10}, duty_h{100};
        static constexpr float A{(duty_h - duty_l) / (temp_h - temp_l)};
        int duty_percent;
        if (temperature < temp_min)
            duty_percent = 0;
        else if (temperature < temp_l)
            duty_percent = duty_l;
        else if (temperature > temp_h)
            duty_percent = duty_h;
        else
            duty_percent = A * (temperature - temp_l) + duty_l; // Linearly interpolate between temp_l and temp_h.
        int pulsewidth{duty_percent * 1000000 / 100 / CONTROL_HZ};
        pwm.pulsewidth_us(pulsewidth);
    }
    int get_duty_percent() {
        return pwm.read_pulsewitdth_us() * CONTROL_HZ * 100 / 1000000;
    }
private:
    PwmOut pwm{PA_8};
    static constexpr int CONTROL_HZ{5000};
};

class mainboard_controller {
public:
    mainboard_controller(can_driver &can) : can(can) {}
    void init() {
        can.register_callback(0x201, callback(this, &mainboard_controller::handle_can));
    }
    void poll() {
        if (timer.elapsed_time() > 1s) {
            heartbeat_timeout = true;
            timer.stop();
            timer.reset();
        }
    }
    bool emergency_stop_from_ros() const {
        return emergency_stop;
    }
    bool power_off_from_ros() const {
        return power_off;
    }
    bool is_dead() const {
        if (heartbeat_detect)
            return heartbeat_timeout || ros_heartbeat_timeout;
        else
            return false;
    }
    bool is_ready() const {
        return heartbeat_detect;
    }
private:
    void handle_can(const CANMessage &msg) {
        heartbeat_timeout = false;
        timer.reset();
        timer.start();
        emergency_stop = msg.data[0] != 0;
        power_off = msg.data[1] != 0;
        ros_heartbeat_timeout = msg.data[2] != 0;
        if (!ros_heartbeat_timeout)
            heartbeat_detect = true;
    }
    can_driver &can;
    Timer timer;
    bool heartbeat_timeout{true}, heartbeat_detect{false}, ros_heartbeat_timeout{false}, emergency_stop{true}, power_off{false};
};

class state_controller {
public:
    void init() {
        i2c.frequency(100000);
        mc.init();
        ac.init();
        bmu.init();
        temp.init();
        fan.init();
        mbd.init();
        globalqueue.call_every(20ms, this, &state_controller::poll);
        globalqueue.call_every(100ms, this, &state_controller::poll_100ms);
        globalqueue.call_every(1s, this, &state_controller::poll_1s);
    }
private:
    enum class POWER_STATE {
        OFF,
        WAIT_SW,
        POST,
        STANDBY,
        NORMAL,
        AUTO_CHARGE,
        MANUAL_CHARGE,
    };
    void poll() {
        can.poll();
        psw.poll();
        bsw.poll();
        mc.poll();
        ac.poll();
        temp.poll();
        mbd.poll();
        switch (state) {
        case POWER_STATE::OFF:
            set_new_state(mc.is_plugged() ? POWER_STATE::POST : POWER_STATE::WAIT_SW);
            break;
        case POWER_STATE::WAIT_SW:
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                poweron_by_switch = true;
                psw.reset_state();
                set_new_state(POWER_STATE::POST);
            }
            break;
        case POWER_STATE::POST:
            if (!poweron_by_switch && !mc.is_plugged())
                set_new_state(POWER_STATE::OFF);
            if (bmu.is_ok() && temp.is_ok())
                set_new_state(POWER_STATE::STANDBY);
            else if (timer_post.elapsed_time() > 3s)
                set_new_state(POWER_STATE::OFF);
            break;
        case POWER_STATE::STANDBY: {
            auto psw_state{psw.get_state()};
            if (!dcdc.is_ok() || psw_state == power_switch::STATE::LONG_PUSHED || mbd.is_dead())
                set_new_state(POWER_STATE::OFF);
            if (psw_state == power_switch::STATE::PUSHED || mbd.power_off_from_ros() ||
                !bmu.is_ok() || !temp.is_ok()) {
                if (wait_shutdown) {
                    if (timer_shutdown.elapsed_time() > 60s)
                        set_new_state(POWER_STATE::OFF);
                } else {
                    LOG("wait shutdown\n");
                    wait_shutdown = true;
                    bat_out.write(0);
                    timer_shutdown.reset();
                    timer_shutdown.start();
                }
            } else if (!esw.asserted() && !mbd.emergency_stop_from_ros() && mbd.is_ready()) {
                set_new_state(POWER_STATE::NORMAL);
            }
            break;
        }
        case POWER_STATE::NORMAL:
            if (psw.get_state() != power_switch::STATE::RELEASED || mbd.power_off_from_ros() ||
                !bmu.is_ok() || !temp.is_ok() || !dcdc.is_ok() ||
                esw.asserted() || mbd.emergency_stop_from_ros() || mbd.is_dead())
                set_new_state(POWER_STATE::STANDBY);
            if (mc.is_plugged())
                set_new_state(POWER_STATE::MANUAL_CHARGE);
            if (ac.is_docked() && bmu.is_chargable())
                set_new_state(POWER_STATE::AUTO_CHARGE);
            break;
        case POWER_STATE::AUTO_CHARGE:
            if (psw.get_state() != power_switch::STATE::RELEASED || mbd.power_off_from_ros() ||
                !bmu.is_ok() || !temp.is_ok() || !dcdc.is_ok() ||
                esw.asserted() || mbd.emergency_stop_from_ros() || mbd.is_dead())
                set_new_state(POWER_STATE::STANDBY);
            if (bmu.is_full_charge() || ac.is_charger_stopped() || !ac.is_docked() || ac.is_connector_overheat())
                set_new_state(POWER_STATE::NORMAL);
            break;
        case POWER_STATE::MANUAL_CHARGE:
            if (psw.get_state() != power_switch::STATE::RELEASED)
                psw.reset_state();
            if (!mc.is_plugged())
                set_new_state(POWER_STATE::NORMAL);
            break;
        }
    }
    void set_new_state(POWER_STATE newstate) {
        switch (newstate) {
        case POWER_STATE::OFF:
            LOG("enter OFF\n");
            poweron_by_switch = false;
            psw.set_led(false);
            dcdc.set_enable(false);
            bmu.set_enable(false);
            bat_out.write(0);
            while (true) // wait power off
                continue;
            break;
        case POWER_STATE::WAIT_SW:
            LOG("enter WAIT_SW\n");
            break;
        case POWER_STATE::POST:
            LOG("enter POST\n");
            psw.set_led(true);
            bmu.set_enable(true);
            bat_out.write(0);
            timer_post.reset();
            timer_post.start();
            break;
        case POWER_STATE::STANDBY:
            LOG("enter STANDBY\n");
            dcdc.set_enable(true);
            wsw.set_disable(true);
            bat_out.write(1);
            ac.set_enable(false);
            wait_shutdown = false;
            break;
        case POWER_STATE::NORMAL:
            LOG("enter NORMAL\n");
            wsw.set_disable(false);
            bat_out.write(1);
            ac.set_enable(false);
            break;
        case POWER_STATE::AUTO_CHARGE:
            LOG("enter AUTO_CHARGE\n");
            ac.set_enable(true);
            break;
        case POWER_STATE::MANUAL_CHARGE:
            LOG("enter MANUAL_CHARGE\n");
            wsw.set_disable(true);
            bat_out.write(0);
            ac.set_enable(false);
            break;
        }
        state = newstate;
    }
    void poll_100ms() {
        auto temperature{temp.get_temperature()};
        fan.control_by_temperature(temperature);
        uint8_t buf[8]{0};
        if (psw.get_raw_state())
            buf[0] |= 0b00000001;
        bool st0, st1, st2;
        esw.get_raw_state(st0, st1);
        if (st0)
            buf[0] |= 0b00000010;
        if (st1)
            buf[0] |= 0b00000100;
        bsw.get_raw_state(st0, st1);
        if (st0)
            buf[0] |= 0b00001000;
        if (st1)
            buf[0] |= 0b00010000;
        if (mc.is_plugged())
            buf[1] |= 0b00000001;
        if (ac.is_docked())
            buf[1] |= 0b00000010;
        dcdc.get_failed_state(st0, st1);
        if (st0)
            buf[2] |= 0b00000001;
        if (st1)
            buf[2] |= 0b00000010;
        bmu.get_fet_state(st0, st1, st2);
        if (st0)
            buf[2] |= 0b00010000;
        if (st1)
            buf[2] |= 0b00100000;
        if (st2)
            buf[2] |= 0b01000000;
        wsw.get_raw_state(st0, st1);
        if (st0)
            buf[3] |= 0b00000001;
        if (st1)
            buf[3] |= 0b00000010;
        int t0, t1;
        ac.get_connector_temperature(t0, t1);
        buf[4] = fan.get_duty_percent();
        buf[5] = t0;
        buf[6] = t1;
        buf[7] = temperature;
        can.send(CANMessage{0x200, buf});
    }
    void poll_1s() {
        heartbeat_led = !heartbeat_led;
    }
    I2C i2c{PB_7, PB_6};
    can_driver can;
    power_switch psw;
    bumper_switch bsw;
    emergency_switch esw;
    wheel_switch wsw;
    manual_charger mc;
    auto_charger ac{i2c};
    bmu_controller bmu{can};
    temperature_sensor temp{i2c};
    dcdc_converter dcdc;
    fan_driver fan;
    mainboard_controller mbd{can};
    DigitalOut bat_out{PB_5, 0}, heartbeat_led{PB_12, 0};
    POWER_STATE state{POWER_STATE::OFF};
    Timer timer_post, timer_shutdown;
    bool poweron_by_switch{false}, wait_shutdown{false};
};

}

int main()
{
    LOG("RUN!\n");
    state_controller ctrl;
    ctrl.init();
    globalqueue.dispatch_forever();
    return 0;
}

// vim: set expandtab shiftwidth=4:
