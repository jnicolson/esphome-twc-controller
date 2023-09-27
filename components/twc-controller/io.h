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

namespace esphome {
    namespace twc_controller {
        class TeslaControllerIO {
            public:
                virtual ~TeslaControllerIO() {};
                virtual void resetIO(uint16_t);
                virtual void writeActualCurrent(uint8_t) = 0;
                virtual void writeCharger(uint16_t, uint8_t) = 0;
                virtual void writeChargerCurrent(uint16_t, uint8_t, uint8_t) = 0;
                virtual void writeChargerSerial(uint16_t, std::string) = 0;
                virtual void writeChargerTotalKwh(uint16_t, uint32_t) = 0;
                virtual void writeChargerVoltage(uint16_t, uint16_t, uint8_t) = 0;
                virtual void writeTotalConnectedChargers(uint8_t) = 0;
                virtual void writeChargerFirmware(uint16_t, std::string) = 0;
                virtual void writeChargerActualCurrent(uint16_t, uint8_t) = 0;
                virtual void writeChargerTotalPhaseCurrent(uint8_t, uint8_t) = 0;
                virtual void writeChargerConnectedVin(uint16_t, std::string) = 0;
                virtual void writeChargerState(uint16_t, uint8_t) = 0;
                virtual void writeTotalConnectedCars(uint8_t) = 0;
                virtual void writeRaw(uint8_t*, size_t) = 0;
                virtual void writeRawPacket(uint8_t *data, size_t length) = 0;
                virtual void onCurrentMessage(std::function<void(uint8_t)>) = 0;
        };
    }
}
