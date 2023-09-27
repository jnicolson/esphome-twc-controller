/*
TWC Manager for ESP32
Copyright (C) 2020 Craig Peacock
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
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#pragma once

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "esphome/components/uart/uart_component.h"

#include "twc_connector.h"
#include "io.h"

#define MAX_PACKET_LENGTH 32

#define GET_SERIAL_NUMBER_OLD	0xFB19
#define GET_MODEL_NUMBER	    0xFB1A
#define GET_FIRMWARE_VER 	    0xFB1B
#define GET_PLUG_STATE		    0xFBB4

#define PRIMARY_HEARTBEAT		0xFBE0
#define PRIMARY_PRESENCE2	    0xFBE2

#define GET_PWR_STATE           0xFBEB
#define GET_FIRMWARE_VER_EXT    0xFBEC
#define GET_SERIAL_NUMBER       0xFBED
#define GET_VIN_FIRST		    0xFBEE
#define GET_VIN_MIDDLE		    0xFBEF
#define GET_VIN_LAST		    0xFBF1

// The next two commands are ** DANGEROUS ** !
// DO NOT USE THESE!  They are defined so that
// they can be blocked.
#define WRITE_ID_DATE           0xFC19
#define WRITE_MODEL_NO          0xFC1A

// Commands without responses (0xFC)
#define IDLE_MESSAGE            0xFC1D
#define START_CHARGING		    0xFCB1
#define STOP_CHARGING		    0xFCB2
#define PRIMARY_PRESENCE	    0xFCE1

// Responses (0xFD)
#define RESP_SERIAL_NUMBER_OLD  0xFD19
#define RESP_MODEL_NUMBER	    0xFD1A
#define RESP_FIRMWARE_VER	    0xFD1B
#define RESP_PLUG_STATE		    0xFDB4

#define SECONDARY_HEARTBEAT		0xFDE0

#define SECONDARY_PRESENCE	    0xFDE2		// Sent by secondary on reset
#define RESP_PWR_STATUS         0xFDEB		// Sent by primary on reset
#define RESP_FIRMWARE_VER_EXT   0xFDEC
#define RESP_SERIAL_NUMBER      0xFDED
#define RESP_VIN_FIRST	        0xFDEE
#define RESP_VIN_MIDDLE         0xFDEF
#define RESP_VIN_LAST	        0xFDF1

// Commands are SLIP encoded on the wire - need the SLIP constants
#define SLIP_END        0xC0
#define SLIP_ESC        0xDB
#define SLIP_ESC_END    0xDC
#define SLIP_ESC_ESC    0xDD

namespace esphome {
    namespace twc_controller {

        struct PACKET_T {
            uint16_t command;
            uint16_t twcid;
            uint16_t secondary_twcid;
            uint8_t payload[6];
            uint8_t checksum;
        } PACKED;

        struct RESP_PACKET_T {
            uint16_t command;
            uint16_t twcid;
            uint8_t payload[11];
            uint8_t checksum;
        } PACKED;

        struct EXTENDED_RESP_PACKET_T {
            uint16_t command;
            uint16_t twcid;
            uint8_t payload[15];
            uint8_t checksum;
        } PACKED;

        struct S_HEARTBEAT_T {
            uint16_t command;
            uint16_t src_twcid;
            uint16_t dst_twcid;
            uint8_t state;
            uint16_t max_current;
            uint16_t actual_current;
            uint8_t padding[4];
            uint8_t checksum;
        } PACKED;

        struct P_HEARTBEAT_T {
            uint16_t command;
            uint16_t src_twcid;
            uint16_t dst_twcid;
            uint8_t state;
            uint16_t max_current;
            uint8_t plug_inserted;
            uint8_t padding[5];
            uint8_t checksum;
        } PACKED;

        // Standard response packet payload
        struct PRESENCE_PAYLOAD_T {
            uint8_t     sign;
            uint16_t    max_allowable_current;
            uint8_t     padding[8];
        } PACKED;

        // Extended response packet payload
        struct POWERSTATUS_PAYLOAD_T {
            uint32_t total_kwh;
            uint16_t phase1_voltage;
            uint16_t phase2_voltage;
            uint16_t phase3_voltage;
            uint8_t phase1_current;
            uint8_t phase2_current;
            uint8_t phase3_current;
            uint8_t padding[2];
        } PACKED;

        // Extended response packet payload
        struct VIN_PAYLOAD_T {
            uint8_t vin[7];
            uint8_t padding[8];
        } PACKED;

        // Extended response packet payload
        struct SERIAL_PAYLOAD_T {
            uint8_t serial[11];
            uint8_t padding[4];
        } PACKED;

        // Standard response packet Payload
        struct EXT_FIRMWARE_PAYLOAD_T {
            uint8_t major;
            uint8_t minor;
            uint8_t revision;
            uint8_t extended;
            uint8_t padding[7];
        } PACKED;

        class TeslaController {
            public:
                TeslaController(uart::UARTComponent* serial, TeslaControllerIO *io, uint16_t twcid, GPIOPin *flow_control_pin, int passive_mode);

                void Begin();
                void GetPowerStatus(uint16_t secondary_twcid);
                void GetFirmware(uint16_t secondary_twcid);
                void GetSerial(uint16_t secondary_twcid);
                void GetFirmwareVer(uint16_t secondary_twcid);
                void GetVin(uint16_t secondary_twcid);
                void Handle();
                void SendCommand(uint16_t command, uint16_t send_to);
                void SendPresence(bool presence2 = false);
                void SendPresence2();
                void SendHeartbeat(uint16_t secondary_twcid);
                void SendIdle();
                void Startup();
                void SendData(uint8_t *packet, size_t length);
                void DecodePowerState(EXTENDED_RESP_PACKET_T *power_state);
                void DecodePrimaryPresence(RESP_PACKET_T *presence, uint8_t num);
                void DecodePrimaryHeartbeat(P_HEARTBEAT_T *heartbeat);
                void DecodeSecondaryPresence(RESP_PACKET_T *presence);
                void DecodeSecondaryHeartbeat(S_HEARTBEAT_T *heartbeat);
                void DecodeVin(EXTENDED_RESP_PACKET_T *vin);
                void DecodeExtFirmwareVerison(RESP_PACKET_T *firmware_ver);
                void DecodeSerialNumber(EXTENDED_RESP_PACKET_T *serial);
                void SetCurrent(uint8_t current);
                void SetMaxCurrent(uint8_t maxCurrent);
                void SetMinCurrent(uint8_t mincurrent);
                void SetStopStartDelay(uint16_t stopstart_delay);
                uint8_t ChargersConnected();
                TeslaConnector * GetConnector(uint16_t twcid);
                void UpdateTotalActualCurrent();
                void UpdateTotalPhaseCurrent(uint8_t phase);
                void UpdateTotalConnectedCars();
                void StopCharging(uint16_t twcid);
                void StartCharging(uint16_t twcid);
                bool IsCharging();

            private:
                uint8_t CalculateChecksum(uint8_t *buffer, size_t length);
                bool VerifyChecksum(uint8_t *buffer, size_t length);
                void DecodeLinkReady();
                void DecodePrimaryHeartbeat();
                void DecodeSecondaryHeartbeat();
                void ProcessPacket(uint8_t *packet, size_t length);
                static void startupTask_(void *pvParameter);
                void Debug(bool enabled);
                void SendDataFromString(const uint8_t* dataString, size_t length);

            private:
                uart::UARTComponent *serial_;
                GPIOPin *flow_control_pin_{nullptr};
                TeslaControllerIO *controller_io_;
                uint8_t num_connected_chargers_;
                uint16_t twcid_;
                uint8_t sign_;
                uint8_t receive_buffer_[MAX_PACKET_LENGTH];
                uint8_t receive_index_;
                bool message_started_ = false;
                uint8_t available_current_;
                uint8_t max_current_;
                uint8_t min_current_;
                uint16_t stopstart_delay_;
                bool current_changed_;
                bool debug_;
                uint8_t passive_mode_;
                uint8_t total_current_;
                TeslaConnector* chargers[3];
        };
    }
}
