/*
 * Copyright (c) 2022, LexxPluss Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
