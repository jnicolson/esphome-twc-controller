/*
TWC Manager for ESP32
Copyright (C) 2020 Jarl Nicolson
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



#include "twc_connector.h"


namespace esphome {
    namespace twc_controller {
        static const char *TAG = "twc-controller.connector";

        TeslaConnector::TeslaConnector(uint16_t twcid, uint8_t max_allowable_current) :
            twcid(twcid),
            max_allowable_current(max_allowable_current)
        {
            // Fill array with null.  That way when the VIN
            // has been fully populated (by 3 different calls),
            // strlen(vin_) will return 17.
            memset(vin_, '\0', sizeof(uint8_t) * 18);
        }

        uint8_t * TeslaConnector::GetVin() {
            return vin_;
        }

        void TeslaConnector::SetActualCurrent(uint8_t current) {
            actual_current_ = current;
        }

        uint8_t TeslaConnector::GetActualCurrent() {
            return actual_current_;
        }

        uint8_t TeslaConnector::GetPhaseCurrent(uint8_t phase) {
            switch (phase) {
                case 1:
                    return phase1_current;
                    break;
                case 2:
                    return phase2_current;
                    break;
                case 3:
                    return phase3_current;
                    break;
                default:
                    ESP_LOGE(TAG, "Request for phase >3 - ignored");
                    return 0;
            }
        }
    }
}