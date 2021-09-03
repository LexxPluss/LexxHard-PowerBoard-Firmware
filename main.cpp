#include "mbed.h"

namespace {

enum class POWER_STATE {
    OFF,
    WAIT_SW,
    POST,
    DISCHARGE_LOW,
    NORMAL,
    AUTO_CHARGE,
    MANUAL_CHARGE,
};

class can_callback {
public:
    void register_callback(int msgid, int len, Callback<void(const CANMessage &msg)> func) {
        if (count < NUM) {
            callbacks[count].msgid = msgid;
            callbacks[count].len = len;
            callbacks[count].func = func;
            ++count;
        }
    }
    bool call_callbacks(const CANMessage &msg) {
        for (int i = 0; i < count; ++i) {
            uint16_t msgid = (msg.id >> 8) & 0xffff;
            if (msgid == callbacks[i].msgid && msg.len == callbacks[i].len) {
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
        int msgid, len;
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
    void register_callback(int msgid, int len, Callback<void(const CANMessage &msg)> func) {
        can.filter(msgid << 8, 0x00ffff00u, CANExtended, filter_handle);
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

class manual_charger {
public:
    bool get_plugged() {return din.read() == 0;}
private:
    DigitalIn din{PB_10, PullUp};
};

class auto_charger {
public:
    bool get_docked() {return din.read() == 0;}
    void set_enable(bool enable) {dout = enable ? 1 : 0;}
    bool get_connector_overheat() {return false;}
    void poll() {}
private:
    DigitalIn din{PB_1, PullUp};
    DigitalOut dout{PB_2, 0};
};

class bmu_control {
public:
    bmu_control(can_driver &can) : can(can) {
        for (auto i : {0x100, 0x101, 0x113})
            can.register_callback(i, 8, callback(this, &bmu_control::handle_can));
    }
    void set_enable(bool enable) {main_sw = enable ? 0 : 1;}
    void set_discharge(bool enable) {fet_active = enable ? 0 : 1;}
    bool is_ok() const {
        return ((data.mod_status1 & 0xbf) == 0 ||
                (data.mod_status2 & 0xe1) == 0 ||
                (data.bmu_alarm1  & 0xff) == 0 ||
                (data.bmu_alarm2  & 0x01) == 0);
    }
private:
    void handle_can(const CANMessage &msg) {
        uint16_t msgid = (msg.id >> 8) & 0xffff;
        switch (msgid) {
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
    DigitalOut main_sw{PB_11, 1}, fet_active{PB_12, 1};
    struct {
        uint8_t mod_status1{0xff}, mod_status2{0xff}, bmu_alarm1{0xff}, bmu_alarm2{0xff};
    } data;
};

class temperature_sensor {
public:
    bool is_ok() const {return true;} //@@
private:
};

class dcdc_base {
public:
    void set_enable(bool enable) {control = enable ? 1 : 0;}
    bool is_ok() {return fail.read() != 0;}
protected:
    dcdc_base(PinName control_pin, PinName fail_pin) : control(control_pin, 0), fail(fail_pin, PullUp) {}
private:
    DigitalOut control;
    DigitalIn fail;
};

struct dcdc5 : public dcdc_base {
    dcdc5() : dcdc_base(PA_10, PA_15) {}
};

struct dcdc16 : public dcdc_base {
    dcdc16() : dcdc_base(PB_3, PB_4) {}
};

struct dcbatout : public dcdc_base {
    dcbatout() : dcdc_base(PB_5, PB_8) {}
};

class state_controller {
public:
    void handler() {
        can.poll();
        psw.poll();
        ac.poll();
        poll();
    }
private:
    void poll() {
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
            if (!dc5.is_ok() || !dc16.is_ok() || psw_state == power_switch::STATE::LONG_PUSHED)
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
            if (psw.get_state() != power_switch::STATE::RELEASED || !bmu.is_ok() || !temp.is_ok() || !dc5.is_ok() || !dc16.is_ok() || bsw.asserted() || esw.asserted())
                set_new_state(POWER_STATE::DISCHARGE_LOW);
            if (mc.get_plugged())
                set_new_state(POWER_STATE::MANUAL_CHARGE);
            if (ac.get_docked())
                set_new_state(POWER_STATE::AUTO_CHARGE);
            break;
        case POWER_STATE::AUTO_CHARGE:
            if (psw.get_state() != power_switch::STATE::RELEASED || !bmu.is_ok() || !temp.is_ok() || !dc5.is_ok() || !dc16.is_ok() || bsw.asserted() || esw.asserted())
                set_new_state(POWER_STATE::DISCHARGE_LOW);
            if (!ac.get_docked() || ac.get_connector_overheat())
                set_new_state(POWER_STATE::NORMAL);
            break;
        case POWER_STATE::MANUAL_CHARGE:
            if (psw.get_state() != power_switch::STATE::RELEASED)
                psw.reset_state();
            if (!bmu.is_ok() || !temp.is_ok() || !dc5.is_ok() || !dc16.is_ok() || bsw.asserted() || esw.asserted())
                set_new_state(POWER_STATE::DISCHARGE_LOW);
            if (!mc.get_plugged())
                set_new_state(POWER_STATE::NORMAL);
            break;
        }
    }
    void set_new_state(POWER_STATE newstate) {
        switch (newstate) {
        case POWER_STATE::OFF:
            bmu.set_discharge(false);
            bmu.set_enable(false);
            while (true) // wait power off
                continue;
            break;
        case POWER_STATE::WAIT_SW:
            break;
        case POWER_STATE::POST:
            bmu.set_enable(true);
            bmu.set_discharge(true);
            timer_post.reset();
            timer_post.start();
            break;
        case POWER_STATE::DISCHARGE_LOW:
            dc5.set_enable(true);
            dc16.set_enable(true);
            break;
        case POWER_STATE::NORMAL:
            dcout.set_enable(true);
            ac.set_enable(false);
            break;
        case POWER_STATE::AUTO_CHARGE:
            ac.set_enable(true);
            break;
        case POWER_STATE::MANUAL_CHARGE:
            ac.set_enable(false);
            break;
        }
        state = newstate;
    }
    can_driver can;
    power_switch psw;
    bumber_switch bsw;
    emergency_switch esw;
    manual_charger mc;
    auto_charger ac;
    bmu_control bmu{can};
    temperature_sensor temp;
    dcdc5 dc5;
    dcdc16 dc16;
    dcbatout dcout;
    POWER_STATE state{POWER_STATE::OFF};
    Timer timer_post, timer_shutdown;
    bool poweron_by_switch{false}, wait_shutdown{false};
};

}

int main()
{
    state_controller ctrl;
    EventQueue queue;
    queue.call_every(10ms, &ctrl, &state_controller::handler);
    queue.dispatch_forever();
    return 0;
}

/* vim: set expandtab shiftwidth=4: */
