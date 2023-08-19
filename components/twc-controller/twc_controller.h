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

#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

#include "twc_protocol.h"
#include "io.h"

namespace esphome {
    namespace twc_controller {

        class TWCController : public number::Number, public uart::UARTDevice, public Component, public TeslaControllerIO {
            public:
                void setup() override;
                void loop() override;
                void dump_config() override;

                float get_setup_priority() const override { 
                    return esphome::setup_priority::AFTER_CONNECTION; 
                }

                void set_min_current(uint8_t current);
                void set_max_current(uint8_t current);
                void set_twcid(uint16_t);

                void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
               
                SUB_SENSOR(current)
                SUB_SENSOR(max_allowable_current)
                SUB_SENSOR(total_kwh_delivered)
                SUB_SENSOR(phase_1_voltage)
                SUB_SENSOR(phase_2_voltage)
                SUB_SENSOR(phase_3_voltage)
                SUB_SENSOR(phase_1_current)
                SUB_SENSOR(phase_2_current)
                SUB_SENSOR(phase_3_current)
                SUB_SENSOR(actual_current)
                SUB_SENSOR(state)
                
                SUB_TEXT_SENSOR(serial)
                SUB_TEXT_SENSOR(firmware_version)
                SUB_TEXT_SENSOR(connected_vin)

/* IO Functions */                
                void writeActualCurrent(uint8_t actualCurrent);
                void writeCharger(uint16_t, uint8_t);
                void writeChargerCurrent(uint16_t, uint8_t, uint8_t);
                void writeChargerSerial(uint16_t, std::string);
                void writeChargerTotalKwh(uint16_t, uint32_t);
                void writeChargerVoltage(uint16_t, uint16_t, uint8_t);
                void writeTotalConnectedChargers(uint8_t);
                void writeChargerFirmware(uint16_t, std::string);
                void writeChargerActualCurrent(uint16_t, uint8_t);
                void writeChargerTotalPhaseCurrent(uint8_t, uint8_t);
                void writeChargerConnectedVin(uint16_t, std::string);
                void writeChargerState(uint16_t, uint8_t);
                void writeTotalConnectedCars(uint8_t);
                void writeRaw(uint8_t*, size_t);
                void writeRawPacket(uint8_t *data, size_t length);
                void onCurrentMessage(std::function<void(uint8_t)>);

/* End IO Functions */
            protected:
                GPIOPin *flow_control_pin_{nullptr};
                void print_params_();
                void control(float value);
                
                TeslaController *teslaController_;
                std::function<void(uint8_t)> onCurrentMessageCallback_=nullptr;
                uint8_t min_current_;
                uint8_t max_current_;
                uint16_t twcid_;
        };

    }
}