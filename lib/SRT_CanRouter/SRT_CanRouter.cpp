#include "SRT_CanRouter.h"
#include <Arduino.h>
#include "driver/twai.h"

ParsedId parse_can_id(uint16_t can_id) {
    ParsedId p{Proto::UNKNOWN, 0, 0};

    uint8_t type    = (can_id >> 5) & 0x3;   // bits 6:5
    uint8_t node_id =  can_id        & 0x1F; // bits 4:0

    switch (type) {
        case 0b00: // Lichuan / CANopen
            p.proto   = Proto::OPENCAN;
            p.node_id = node_id;            // 0..31 valid
            break;

        case 0b01: // GIM / ODrive, 1+4n
            if (node_id >= 1 && ((node_id - 1) % 4 == 0)) {
                p.proto   = Proto::ODRIVE;
                p.node_id = node_id;
            }
            break;

        case 0b10: // future proto 2
        case 0b11: // future proto 3
        default:
            break;
    }

    return p;
}

void process_can_frame(uint16_t id, uint8_t len, uint8_t* data,
                       SRT_OdriveMtr& odrive,
                       SRT_CanOpenMtr& opencan) {
    ParsedId p = parse_can_id(id);

    switch (p.proto) {
        case Proto::ODRIVE:
            odrive.process_msg(id, len, data);
            break;
        case Proto::OPENCAN:
            opencan.process_msg(id, len, data);
            break;
        case Proto::UNKNOWN:
        default:
            // ignore
            break;
    }
}

// ---- optional router class that also owns TWAI ----

SRT_CanRouter::SRT_CanRouter(gpio_num_t tx, gpio_num_t rx,
                             const twai_timing_config_t& timing,
                             SRT_OdriveMtr* odrv,
                             SRT_CanOpenMtr* open)
    : _tx(tx), _rx(rx), _timing(timing),
      _odrv(odrv), _open(open) {}

bool SRT_CanRouter::begin() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(_tx, _rx, TWAI_MODE_NORMAL);
    g.rx_queue_len = 20;

    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &_timing, &f) != ESP_OK) {
        Serial.println("[CanRouter] driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CanRouter] start failed");
        return false;
    }
    Serial.println("[CanRouter] TWAI started");
    return true;
}

void SRT_CanRouter::poll() {
    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(2)) == ESP_OK) {
        if (msg.rtr) continue;
        process_can_frame(msg.identifier, msg.data_length_code, msg.data, *_odrv, *_open);
    }
}