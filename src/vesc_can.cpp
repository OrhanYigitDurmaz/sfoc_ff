#include "vesc_can.h"
#include <algorithm>

enum VescState : uint32_t {
    VESC_STATE_UNKNOWN = 0,
    VESC_STATE_INCOMPATIBLE = 1,
    VESC_STATE_PONG = 2,
    VESC_STATE_COMPATIBLE = 3,
    VESC_STATE_READY = 4,
    VESC_STATE_ERROR = 5
};

enum VescCANMsg : uint8_t {
    CAN_PACKET_FILL_RX_BUFFER = 5,
    CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
    CAN_PACKET_PROCESS_RX_BUFFER = 7,
    CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_PING = 17,
    CAN_PACKET_PONG = 18,
    CAN_PACKET_POLL_ROTOR_POS = 56
};

enum VescCmd : uint8_t {
    COMM_FW_VERSION = 0,
    COMM_ROTOR_POSITION = 22,
    COMM_GET_VALUES_SELECTIVE = 50
};

void CanInterface::linkCan(BaseCAN *_can) {
    this->can = _can;
}

void CanInterface::linkMotor(FOCMotor *_motor) {
    this->motor = _motor;
}

void CanInterface::setCanSpeed(uint32_t _speed) {
    this->can_speed = _speed;
}

void CanInterface::setCanAddr(uint8_t _addr) {
    this->can_address = _addr;
}

void CanInterface::begin() {
    this->can->logTo(&Serial);
    this->can->disableInternalLoopback();
    CanFilter filter = CanFilter(MASK_EXTENDED, this->can_address, 0xFF, FILTER_ANY_FRAME);
    this->can->filter(filter);
    this->can->begin(this->can_speed);
}

void CanInterface::enableRemote(bool enable) {
    this->remote_enable = enable;
}

void CanInterface::buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void CanInterface::buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
    this->buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void CanInterface::run() {
    if (!this->can) return;
    if (!this->motor) return;

    uint32_t current_time = micros();
    if ((current_time - this->time_of_last_comm > 9e5f) && this->motor->enabled && this->remote_enable) {
        this->motor->disable();
        Serial.printf("No Comms: Motor Off\n");
    }

    while (this->can->available() > 0) {
        CanMsg const rxMsg = this->can->read();
        if (!rxMsg.isExtendedId())
            continue;

        uint32_t ext_id = rxMsg.getExtendedId();
        uint8_t id = ext_id & 0xFF;
        uint8_t cmd = (ext_id >> 8) & 0xFF;

        if (!(id == this->can_address || id == 0xFF)) 
            continue;
        
        CanMsg txMsg;

        switch(cmd) {
            case VescCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER:
                this->process_short_buffer(rxMsg);
                break;

            case VescCANMsg::CAN_PACKET_SET_CURRENT_REL:
                if (rxMsg.data_length < 4 || !this->remote_enable) break;
                int32_t int_tq;
                memcpy(&int_tq, rxMsg.data, 4);
                int_tq = __builtin_bswap32(int_tq);
                this->motor->target = this->motor->current_limit * ((float)int_tq / 100000.0f);
                break;

            case VescCANMsg::CAN_PACKET_PING: 
                txMsg = CanMsg(CanExtendedId(this->can_address | (VescCANMsg::CAN_PACKET_PONG << 8)), 0, nullptr);
                this->can->write(txMsg);
                break;

            case VescCANMsg::CAN_PACKET_POLL_ROTOR_POS: {
                float mech_angle = fmodf(this->motor->shaft_angle, _2PI) * 180.0f / _PI;
                int32_t pos = static_cast<int32_t>(mech_angle * 100000.0f);
                std::array<uint8_t, 4> buffer;
                memcpy(buffer.data(), &pos, 4);
                std::reverse(buffer.begin(), buffer.end());
                txMsg = CanMsg(CanExtendedId(this->can_address | (VescCANMsg::CAN_PACKET_POLL_ROTOR_POS << 8)), 4, buffer.data());
                this->can->write(txMsg);
                break;
            }
        }
    }
}

void CanInterface::process_short_buffer(CanMsg rxMsg) {
    if (rxMsg.data_length < 3) return;

    uint8_t sendTo = rxMsg.data[0];
    uint8_t action = rxMsg.data[1];
    uint8_t comm_cmd = rxMsg.data[2];
    std::array<uint8_t, BUFFER_RX_SIZE> buffer;
    size_t ind = 0;

    CanMsg txMsg;

    switch (comm_cmd) {
        case VescCmd::COMM_FW_VERSION:
            buffer[ind++] = this->can_address;
            buffer[ind++] = 0;
            buffer[ind++] = VescCmd::COMM_FW_VERSION;
            buffer[ind++] = VESC_FW_MAJOR;
            buffer[ind++] = VESC_FW_MINOR;
            txMsg = CanMsg(CanExtendedId(sendTo | (VescCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER << 8)), ind, buffer.data());
            this->can->write(txMsg);
            break;

        case VescCmd::COMM_GET_VALUES_SELECTIVE: {

            if (rxMsg.data_length < 7) return;

            this->time_of_last_comm = micros();
            if (!this->motor->enabled && this->remote_enable) {
                this->motor->enable();
            }

            uint32_t request;
            memcpy(&request, &rxMsg.data[3], 4);
            request = __builtin_bswap32(request);

            buffer[ind++] = sendTo;
            buffer[ind++] = action;
            buffer[ind++] = VescCmd::COMM_GET_VALUES_SELECTIVE;


            if (request & (1 << 8)) { // VOLTAGE_IN
                this->buffer_append_float16(buffer.data(), this->voltage, 1e1, reinterpret_cast<int32_t*>(&ind));
            }

            if (request & (1 << 15)) { // FAULT_CODE
                buffer[ind++] = this->error_state;
            }

            this->respond(buffer.data(), ind);
            break;
        }
    }
}

void CanInterface::respond(uint8_t* data, size_t len) {
    if (len <= 8) {
        CanMsg txMsg(CanExtendedId(this->can_address | (VescCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER << 8)), len, data);
        this->can->write(txMsg);
    } else {
        uint8_t send_buffer[8];
        size_t send_len;

        unsigned int i;
        for (i = 0; i < len; i += 7) {
            if (i > 255) break;
            send_len = std::min<size_t>(7, len - i);
            send_buffer[0] = static_cast<uint8_t>(i);
            memcpy(send_buffer + 1, data + i, send_len);
            CanMsg txMsg(CanExtendedId(this->can_address | (VescCANMsg::CAN_PACKET_FILL_RX_BUFFER << 8)), send_len + 1, send_buffer);
            this->can->write(txMsg);
        }

        for (; i < len; i += 6) {
            send_len = std::min<size_t>(6, len - i);
            send_buffer[0] = static_cast<uint8_t>(i >> 8);
            send_buffer[1] = static_cast<uint8_t>(i);
            memcpy(send_buffer + 2, data + i, send_len);
            CanMsg txMsg(CanExtendedId(this->can_address | (VescCANMsg::CAN_PACKET_FILL_RX_BUFFER_LONG << 8)), send_len + 2, send_buffer);
            this->can->write(txMsg);
        }

        uint8_t process_buffer[3];
        process_buffer[0] = data[2]; 
        process_buffer[1] = (len >> 8) & 0xFF;
        process_buffer[2] = len & 0xFF;
        CanMsg txMsg(CanExtendedId(this->can_address | (VescCANMsg::CAN_PACKET_PROCESS_RX_BUFFER << 8)), 3, process_buffer);
        this->can->write(txMsg);
    }
}

void CanInterface::setVBus(float _voltage) {
    this->voltage = _voltage;
}

void CanInterface::setErrorState(int _error) {
    this->error_state = _error;
}