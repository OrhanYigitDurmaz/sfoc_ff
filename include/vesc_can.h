#pragma once

#include <stdint.h>
#include "SimpleCAN.h"
#include "SimpleFOC.h"

// https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can


constexpr size_t BUFFER_RX_SIZE = 64;

class CanInterface {
public:
    void linkMotor(FOCMotor *motor);
    void linkCan(BaseCAN *can);
    void setCanAddr(uint8_t address);

    void begin();

    void run();

    uint8_t can_address = 0xFF;
    int32_t can_baud = 500000;
private:
    BaseCAN *can = nullptr;
    FOCMotor *motor = nullptr;
    // uint8_t buffer_rx[BUFFER_RX_SIZE]; // may be used to store multi-frame messages, currently unused

    void process_short_buffer(CanMsg rxMsg);
};

