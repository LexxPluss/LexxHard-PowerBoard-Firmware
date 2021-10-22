#pragma once

class serial_message {
public:
    bool decode(uint8_t data) {
        return decode_single(data);
    }
    bool decode(uint8_t *buf, int length) {
        while (length--) {
            if (decode_single(*buf++))
                return true;
        }
        return false;
    }
    uint8_t get_command(uint8_t param[3]) const {
        param[0] = request.detail.param[0];
        param[1] = request.detail.param[1];
        param[2] = request.detail.param[2];
        return request.detail.command;
    }
    static void compose(uint8_t buf[8], uint8_t command, uint8_t *param) {
        buf[0] = 'L';
        buf[1] = 'P';
        buf[2] = command;
        if (param != nullptr) {
            buf[3] = param[0];
            buf[4] = param[1];
            buf[5] = param[2];
        } else {
            buf[3] = buf[4] = buf[5] = 0;
        }
        uint16_t sum{calc_check_sum(buf, 6)};
        buf[6] = sum;
        buf[7] = sum >> 8;
    }
    void reset() {state = STATE::HEAD0;}
    static constexpr uint8_t HEARTBEAT{0x01};
    static constexpr uint8_t POWERON{0x02};
    static constexpr uint8_t POWEROFF{0x03};
private:
    bool decode_single(uint8_t data) {
        bool result{false};
        switch (state) {
        case STATE::HEAD0:
            request.raw[0] = data;
            if (data == 'L')
                state = STATE::HEAD1;
            break;
        case STATE::HEAD1:
            request.raw[1] = data;
            if (data == 'P') {
                state = STATE::DATA;
                data_count = 2;
            } else {
                reset();
            }
            break;
        case STATE::DATA:
            request.raw[data_count] = data;
            if (++data_count >= sizeof request.raw) {
                reset();
                uint16_t sum{calc_check_sum(request.raw, 6)};
                if (((sum >> 0) & 0xff) == request.detail.sum[0] &&
                    ((sum >> 8) & 0xff) == request.detail.sum[1])
                    result = true;
            }
            break;
        }
        return result;
    }
    static uint16_t calc_check_sum(const uint8_t *buf, uint32_t length) {
        uint16_t value{0};
        while (length--)
            value += *buf++;
        uint8_t l{static_cast<uint8_t>(255 - value % 256)};
        uint8_t h{static_cast<uint8_t>(~l)};
        return (h << 8) | l;
    }
    union {
        uint8_t raw[8];
        struct {
            uint8_t head[2];
            uint8_t command;
            uint8_t param[3];
            uint8_t sum[2];
        } detail;
    } request;
    uint32_t data_count{0};
    enum class STATE {
        HEAD0, HEAD1, DATA
    } state{STATE::HEAD0};
};

// vim: set expandtab shiftwidth=4:
