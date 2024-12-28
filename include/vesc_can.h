#pragma once

#include <stdint.h>
#include "SimpleCAN.h"
#include "SimpleFOC.h"

// https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can


constexpr size_t BUFFER_RX_SIZE = 64;
constexpr uint8_t VESC_FW_MAJOR = 0xBE;
constexpr uint8_t VESC_FW_MINOR = 0xEF;


class CanInterface {
public:
    void linkMotor(FOCMotor *motor);
    void linkCan(BaseCAN *can);
    
    void setCanAddr(uint8_t address);
    void setCanSpeed(uint32_t speed);
    void setVBus(float voltage);
    void setErrorState(int err);
    void begin();
    void run();
    void enableRemote(bool enable);


private:
    void process_short_buffer(CanMsg rxMsg);
    BaseCAN *can = nullptr;
    FOCMotor *motor = nullptr;

    float voltage=0.0;
    int error_state = 0;
    uint32_t time_of_last_comm = 0;
    int32_t can_speed = 500000;
    uint8_t can_address = 0xFF;
    uint8_t remote_enable = 1;

    // uint8_t buffer_rx[BUFFER_RX_SIZE]; // may be used to store multi-frame messages, currently unused

};

