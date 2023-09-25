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

#include "esphome/core/log.h"

#include "twc_controller.h"

namespace esphome {
    namespace twc_controller {
        static const char *TAG = "twc";

        void TWCController::setup() {
            if (this->flow_control_pin_ != nullptr) {
                this->flow_control_pin_->setup();
            }

            this->publish_state(this->min_current_);

            teslaController_ = new TeslaController(this->parent_, this, twcid_, flow_control_pin_, passive_mode_);

            teslaController_->SetMinCurrent(this->min_current_);
            teslaController_->SetMaxCurrent(this->max_current_);

            teslaController_->Begin();
            teslaController_->Startup();
        }

        void TWCController::loop() {
            teslaController_->Handle();

        }

/* IO Functions */
        void TWCController::resetIO(uint16_t twcid) {
            // Write 0's to MQTT for each topic which has 0 as a valid value.  This is because
            // we compare the old and new values and by default everything is 0 so it never writes
            // anything.  This way we start at 0 and immediately update to the real value if there is
            // one, or stay at 0 (which is correct) if there isn't.
            writeChargerVoltage(twcid, 0, 1);
            writeChargerVoltage(twcid, 0, 2);
            writeChargerVoltage(twcid, 0, 3);

            writeChargerCurrent(twcid, 0, 1);
            writeChargerCurrent(twcid, 0, 2);
            writeChargerCurrent(twcid, 0, 3);

            writeChargerActualCurrent(twcid, 0);

            writeChargerConnectedVin(twcid, "0");

            writeChargerState(twcid, 0);
            writeTotalConnectedCars(0);
        }

        void TWCController::writeActualCurrent(uint8_t actualCurrent) {
            this->current_sensor_->publish_state((float)actualCurrent);
        }

        void TWCController::writeCharger(uint16_t twcid, uint8_t max_allowable_current) {
            this->max_allowable_current_sensor_->publish_state((float)max_allowable_current);
        }

        void TWCController::writeChargerCurrent(uint16_t twcid, uint8_t current, uint8_t phase) {
            switch (phase) {
                case 1:
                    this->phase_1_current_sensor_->publish_state((float)current);
                    break;
                case 2:
                    this->phase_2_current_sensor_->publish_state((float)current);
                    break;
                case 3:
                    this->phase_3_current_sensor_->publish_state((float)current);
                    break;
                default:
                    ESP_LOGE(TAG, "Phase should be 3 or less");
                    return;
            }
        };

        void TWCController::writeChargerSerial(uint16_t twcid, std::string serial) {
            this->serial_text_sensor_->publish_state(serial);
        }

        void TWCController::writeChargerTotalKwh(uint16_t twcid, uint32_t total_kwh) {
            this->total_kwh_delivered_sensor_->publish_state((float)total_kwh);
        }

        void TWCController::writeChargerVoltage(uint16_t twcid, uint16_t voltage, uint8_t phase) {
            switch (phase) {
                case 1:
                    this->phase_1_voltage_sensor_->publish_state((float)voltage);
                    break;
                case 2:
                    this->phase_2_voltage_sensor_->publish_state((float)voltage);
                    break;
                case 3:
                    this->phase_3_voltage_sensor_->publish_state((float)voltage);
                    break;
                default:
                    ESP_LOGE(TAG, "Phase should be 3 or less");
                    return;
            }
        }

        void TWCController::writeTotalConnectedChargers(uint8_t connected_chargers) {

        };

        void TWCController::writeChargerFirmware(uint16_t twcid, std::string firmware_version) {
            this->firmware_version_text_sensor_->publish_state(firmware_version);
        };

        void TWCController::writeChargerActualCurrent(uint16_t twcid, uint8_t current) {
            this->actual_current_sensor_->publish_state((float)current);
        }

        void TWCController::writeChargerTotalPhaseCurrent(uint8_t current, uint8_t phase) {

        }

        void TWCController::writeChargerConnectedVin(uint16_t twcid, std::string vin) {
            this->connected_vin_text_sensor_->publish_state(vin);
        }

        void TWCController::writeChargerState(uint16_t twcid, uint8_t state) {
            this->state_sensor_->publish_state((float)state);
        }

        void TWCController::writeTotalConnectedCars(uint8_t connected_cars) {

        }

        void TWCController::writeRaw(uint8_t *data, size_t length) {

        }

        void TWCController::writeRawPacket(uint8_t *data, size_t length) {

        }

        void TWCController::onCurrentMessage(std::function<void(uint8_t)> callback) {
            onCurrentMessageCallback_ = callback;
        }


/* End IO Functions */

        void TWCController::dump_config() {
            ESP_LOGCONFIG(TAG, "TWC Controller:");
            this->print_params_();
        }

        void TWCController::print_params_() {
            ESP_LOGCONFIG(TAG,"  TWC ID: 0x%s", format_hex(this->twcid_).c_str());
            ESP_LOGCONFIG(TAG,"  Min Current: %d", this->min_current_);
            ESP_LOGCONFIG(TAG,"  Max Current: %d", this->max_current_);
            LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
        }

        void TWCController::set_min_current(uint8_t current) {
            ESP_LOGD(TAG, "Set min current");
            this->min_current_ = current;
        }

        void TWCController::set_max_current(uint8_t current) {
            ESP_LOGD(TAG, "Set max current");
            this->max_current_ = current;
        }

        void TWCController::set_twcid(uint16_t twcid) {
            ESP_LOGD(TAG, "Set TWC ID");
            this->twcid_ = twcid;
        }

        void TWCController::control(float value) {
            this->onCurrentMessageCallback_(round(value));
            this->publish_state(value);
        }
    }
}
