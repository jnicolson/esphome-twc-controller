/*
TWC Manager for ESP32
Copyright (C) 2023 Jarl Nicolson
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include <cstring>
#include <stdint.h>

#include "esphome/core/log.h"

namespace esphome {
    namespace twc_controller {
        // Encapsulate some information specific to a single wall connector.  This is to allow multiple connectors to be added
        class TeslaConnector {
            public:
                TeslaConnector(uint16_t twcid, uint8_t max_charge_rate);
                void SetVin(uint8_t* upperVin);
                uint8_t* GetVin();
                void SetActualCurrent(uint8_t current);
                uint8_t GetActualCurrent();
                uint8_t GetPhaseCurrent(uint8_t phase);
                
                uint16_t twcid;
                uint8_t state;
                uint8_t firmware_version[4];
                
                uint8_t serial_number[11];
                
                uint32_t total_kwh;
                uint16_t phase1_voltage;
                uint16_t phase2_voltage;
                uint16_t phase3_voltage;
                
                uint8_t phase1_current;
                uint8_t phase2_current;
                uint8_t phase3_current;

                uint8_t max_allowable_current;
            private:
                uint8_t vin_[18];
                uint8_t actual_current_;
                
        };
    }
}