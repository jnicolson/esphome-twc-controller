#pragma once

#include <string>

#include "esphome/core/helpers.h"

namespace esphome {
    namespace twc_controller {
        uint8_t hexCharacterStringToBytes(uint8_t*, const uint8_t*, size_t);
        uint8_t nibble(uint8_t);
        long random(long, long);
    }
}
