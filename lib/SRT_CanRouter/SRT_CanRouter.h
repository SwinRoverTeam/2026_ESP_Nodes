#ifndef SRT_CANROUTER_H
#define SRT_CANROUTER_H

#include <Arduino.h>
#include "driver/twai.h"
#include "SRT_Odrive.h"
#include "SRT_OpenCan.h"

enum class Proto { ODRIVE, OPENCAN, UNKNOWN };

struct ParsedId {
    Proto   proto;
    uint8_t node_id;     // 0..31
    uint8_t cmd_or_func; // reserved
};

ParsedId parse_can_id(uint16_t can_id);

void process_can_frame(uint16_t id, uint8_t len, uint8_t* data,
                       SRT_OdriveMtr& odrive,
                       SRT_CanOpenMtr& opencan);

// optional helper class if you want it to own TWAI
class SRT_CanRouter {
public:
    SRT_CanRouter(gpio_num_t tx, gpio_num_t rx,
                  const twai_timing_config_t& timing,
                  SRT_OdriveMtr* odrv,
                  SRT_CanOpenMtr* open);

    bool begin();
    void poll(); // call regularly from loop()

private:
    gpio_num_t _tx;
    gpio_num_t _rx;
    twai_timing_config_t _timing;
    SRT_OdriveMtr* _odrv;
    SRT_CanOpenMtr* _open;
};

#endif
