#include "mbed.h"
#include "serial_message.hpp"

namespace {

#undef DEBUG
#ifdef DEBUG
BufferedSerial debugserial{PA_2, PA_3};
FILE *debugout = fdopen(&debugserial, "r+");
#define LOG(...) fprintf(debugout, __VA_ARGS__)
#else
#define LOG(...) [](){}() // lambda empty function
#endif

EventQueue globalqueue;

class can_callback {
public:
    void register_callback(uint32_t msgid, uint32_t len, Callback<void(const CANMessage &msg)> func) {
        if (count < NUM) {
            callbacks[count].msgid = msgid;
            callbacks[count].len = len;
            callbacks[count].func = func;
            ++count;
        }
    }
    bool call_callbacks(const CANMessage &msg) {
        for (int i = 0; i < count; ++i) {
            if (msg.id == callbacks[i].msgid && msg.len == callbacks[i].len) {
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
        uint32_t msgid, len;
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
    void register_callback(uint32_t msgid, uint32_t len, Callback<void(const CANMessage &msg)> func) {
        can.filter(msgid, 0x000007ffu, CANStandard, filter_handle);
        callback.register_callback(msgid, len, func);
        filter_handle = (filter_handle + 1) % 14; // STM CAN filter size
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
        int now = din.read();
        if (prev != now) {
            LOG("power_switch change to %d\n", now);
            prev = now;
            timer.reset();
            timer.start();
        } else if (now == 0) {
            auto elapsed = timer.elapsed_time();
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
private:
    DigitalIn din{PB_0, PullUp};
    Timer timer;
    STATE state{STATE::RELEASED};
    int prev{-1};
};

class switch_base {
public:
    bool asserted() {return sw0.read() == 0 || sw1.read() == 0;}
protected:
    switch_base(PinName sw0_pin, PinName sw1_pin) : sw0(sw0_pin, PullUp), sw1(sw1_pin, PullUp) {}
private:
    DigitalIn sw0, sw1;
};

struct bumber_switch : public switch_base {
    bumber_switch() : switch_base(PA_4, PA_5) {}
};

struct emergency_switch : public switch_base {
    emergency_switch() : switch_base(PA_6, PA_7) {}
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
private:
    DigitalOut left{PB_8, 0}, right{PB_9, 0};
};

class manual_charger {
public:
    bool get_plugged() {return din.read() == 0;}
private:
    DigitalIn din{PB_10, PullUp};
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
        msg.init();
    }
    bool get_docked() {
        return is_connected() && heartbeat_timer.elapsed_time() < 5s;
    }
    void set_enable(bool enable) {
        uint8_t command;
        if (enable) {
            sw.write(1);
            command = serial_message::POWERON;
        } else {
            sw.write(0);
            command = serial_message::POWEROFF;
        }
        uint8_t buf[8];
        serial_message::compose(buf, command, nullptr);
#ifndef DEBUG
        serial.write(buf, sizeof buf);
#endif
    }
    bool get_connector_overheat() {
        return connector_temp[0] > 80.0f || connector_temp[1] > 80.0f;
    }
    void poll() {
        connector_v = connector.read_voltage();
#ifndef DEBUG
        while (serial.readable()) {
            uint8_t data;
            serial.read(&data, 1);
            if (msg.decode(data)) {
                uint8_t param[3];
                uint8_t command = msg.get_command(param);
                if (command == serial_message::HEARTBEAT && param[0] == heartbeat_counter)
                    heartbeat_timer.reset();
            }
        }
#endif
        adc_ticktock();
    }
private:
    void adc_ticktock() {
        if (adc_measure_mode)
            adc_read();
        else
            adc_measure();
    }
    void adc_read() {
        uint8_t buf[2];
        buf[0] = 0b00000000; // Conversion Register
        if (i2c.write(ADDR, reinterpret_cast<const char*>(buf), 1) == 0 &&
            i2c.read(ADDR, reinterpret_cast<char*>(buf), 2) == 0) {
            int16_t value = (buf[1] << 8) | buf[0];
            float voltage = static_cast<float>(value) / 32768.0f * 4.096f;
            calculate_temperature(voltage);
        }
        adc_ch = adc_ch == 0 ? 1 : 0;
        adc_measure_mode = false;
    }
    void adc_measure() {
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
        adc_measure_mode = true;
    }
    void calculate_temperature(float adc_voltage) {
        if (adc_voltage > 3.3f)
            adc_voltage = 3.3f;
        if (adc_voltage < 0.0f)
            adc_voltage = 0.0f;
        // see https://lexxpluss.esa.io/posts/459
        static constexpr float Rpu = 10000.0f, R0 = 3300.0f, B = 3970.0f, T0 = 373.0f;
        float R = Rpu * adc_voltage / (3.3f - adc_voltage);
        float T = 1.0f / (logf(R / R0) / B + 1.0f / T0);
        connector_temp[adc_ch] = T - 273.0f;
    }
    bool is_connected() const {
        return connector_v > 1.0f;
    }
    void poll_1s() {
        if (is_connected())
            send_heartbeat();
    }
    void send_heartbeat() {
        uint8_t buf[8], param[3]{++heartbeat_counter};
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
    Timer heartbeat_timer;
    serial_message msg;
    uint8_t heartbeat_counter{0};
    float connector_v{0.0f}, connector_temp[2]{0.0f, 0.0f};
    int adc_ch{0};
    bool adc_measure_mode{false};
    static constexpr int ADDR{0b10010000};
};

class bmu_control {
public:
    bmu_control(can_driver &can) : can(can) {}
    void init() {
        for (auto i : {0x100, 0x101, 0x113})
            can.register_callback(i, 8, callback(this, &bmu_control::handle_can));
    }
    void set_enable(bool enable) {main_sw = enable ? 1 : 0;}
    bool is_ok() const {
        return ((data.mod_status1 & 0xbf) == 0 ||
                (data.mod_status2 & 0xe1) == 0 ||
                (data.bmu_alarm1  & 0xff) == 0 ||
                (data.bmu_alarm2  & 0x01) == 0);
    }
private:
    void handle_can(const CANMessage &msg) {
        switch (msg.id) {
        case 0x100:
            data.mod_status1 = msg.data[0];
            break;
        case 0x101:
            data.mod_status2 = msg.data[6];
            break;
        case 0x113:
            data.bmu_alarm1 = msg.data[4];
            data.bmu_alarm2 = msg.data[5];
            break;
        }
    }
    can_driver &can;
    DigitalOut main_sw{PB_11, 0};
    struct {
        uint8_t mod_status1{0xff}, mod_status2{0xff}, bmu_alarm1{0xff}, bmu_alarm2{0xff};
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
    void poll() {
        uint8_t buf[2];
        buf[0] = 0x00; // Temperature Value MSB Register
        if (i2c.write(ADDR, reinterpret_cast<const char*>(buf), 1, true) == 0 &&
            i2c.read(ADDR, reinterpret_cast<char*>(buf), 2) == 0) {
            int16_t value = (buf[0] << 8) | buf[1];
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
            ThisThread::sleep_for(1ms);
            control[1].write(1);
        } else {
            control[1].write(0); // 16V must be turned off first.
            ThisThread::sleep_for(1ms);
            control[0].write(0);
        }
    }
    bool is_ok() {
        return fail[0].read() != 0 || fail[1].read() != 0;
    }
private:
    DigitalOut control[2]{{PA_10, 0}, {PB_3, 0}};
    DigitalIn fail[2]{{PA_15, PullNone}, {PB_4, PullNone}};
};

class state_controller {
public:
    void init() {
        i2c.frequency(100000);
        ac.init();
        bmu.init();
        temp.init();
        globalqueue.call_every(20ms, this, &state_controller::poll);
    }
private:
    enum class POWER_STATE {
        OFF,
        WAIT_SW,
        POST,
        DISCHARGE_LOW,
        NORMAL,
        AUTO_CHARGE,
        MANUAL_CHARGE,
    };
    void poll() {
        can.poll();
        psw.poll();
        ac.poll();
        temp.poll();
        switch (state) {
        case POWER_STATE::OFF:
            set_new_state(mc.get_plugged() ? POWER_STATE::POST : POWER_STATE::WAIT_SW);
            break;
        case POWER_STATE::WAIT_SW:
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                poweron_by_switch = true;
                psw.reset_state();
                set_new_state(POWER_STATE::POST);
            }
            break;
        case POWER_STATE::POST:
            if (!poweron_by_switch && !mc.get_plugged())
                set_new_state(POWER_STATE::OFF);
            if (bmu.is_ok() && temp.is_ok())
                set_new_state(POWER_STATE::DISCHARGE_LOW);
            else if (timer_post.elapsed_time() > 3s)
                set_new_state(POWER_STATE::OFF);
            break;
        case POWER_STATE::DISCHARGE_LOW: {
            auto psw_state = psw.get_state();
            if (!dcdc.is_ok() || psw_state == power_switch::STATE::LONG_PUSHED)
                set_new_state(POWER_STATE::OFF);
            if (psw_state == power_switch::STATE::PUSHED || !bmu.is_ok() || !temp.is_ok()) {
                if (wait_shutdown) {
                    if (timer_shutdown.elapsed_time() > 60s)
                        set_new_state(POWER_STATE::OFF);
                } else {
                    wait_shutdown = true;
                    timer_shutdown.reset();
                    timer_shutdown.start();
                }
            }
            if (!bsw.asserted() && !esw.asserted())
                set_new_state(POWER_STATE::NORMAL);
            break;
        }
        case POWER_STATE::NORMAL:
            if (psw.get_state() != power_switch::STATE::RELEASED || !bmu.is_ok() || !temp.is_ok() || !dcdc.is_ok() || bsw.asserted() || esw.asserted())
                set_new_state(POWER_STATE::DISCHARGE_LOW);
            if (mc.get_plugged())
                set_new_state(POWER_STATE::MANUAL_CHARGE);
            if (ac.get_docked())
                set_new_state(POWER_STATE::AUTO_CHARGE);
            break;
        case POWER_STATE::AUTO_CHARGE:
            if (psw.get_state() != power_switch::STATE::RELEASED || !bmu.is_ok() || !temp.is_ok() || !dcdc.is_ok() || bsw.asserted() || esw.asserted())
                set_new_state(POWER_STATE::DISCHARGE_LOW);
            if (!ac.get_docked() || ac.get_connector_overheat())
                set_new_state(POWER_STATE::NORMAL);
            break;
        case POWER_STATE::MANUAL_CHARGE:
            if (psw.get_state() != power_switch::STATE::RELEASED)
                psw.reset_state();
            if (!bmu.is_ok() || !temp.is_ok() || !dcdc.is_ok() || bsw.asserted() || esw.asserted())
                set_new_state(POWER_STATE::DISCHARGE_LOW);
            if (!mc.get_plugged())
                set_new_state(POWER_STATE::NORMAL);
            break;
        }
    }
    void set_new_state(POWER_STATE newstate) {
        switch (newstate) {
        case POWER_STATE::OFF:
            LOG("enter OFF\n");
            poweron_by_switch = false;
            bmu.set_enable(false);
            while (true) // wait power off
                continue;
            break;
        case POWER_STATE::WAIT_SW:
            LOG("enter WAIT_SW\n");
            break;
        case POWER_STATE::POST:
            LOG("enter POST\n");
            bmu.set_enable(true);
            timer_post.reset();
            timer_post.start();
            break;
        case POWER_STATE::DISCHARGE_LOW:
            LOG("enter DISCHARGE_LOW\n");
            dcdc.set_enable(true);
            wsw.set_disable(true);
            bat_out.write(0);
            ac.set_enable(false);
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
    I2C i2c{PB_7, PB_6};
    can_driver can;
    power_switch psw;
    bumber_switch bsw;
    emergency_switch esw;
    wheel_switch wsw;
    manual_charger mc;
    auto_charger ac{i2c};
    bmu_control bmu{can};
    temperature_sensor temp{i2c};
    dcdc_converter dcdc;
    DigitalOut bat_out{PB_5, 0};
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
