/*
   Copyright (C) 2022 Kraus Hamdani Aerospace Inc. All rights reserved.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Author: Tom Pittenger
 */

#pragma once

#include "AP_ADSB_Backend.h"
#include <AP_ADSB/sagetech-sdk/sagetech_mxs.h>


#ifndef HAL_ADSB_SAGETECH_MXS_ENABLED
#define HAL_ADSB_SAGETECH_MXS_ENABLED HAL_ADSB_ENABLED
#endif


#if HAL_ADSB_SAGETECH_MXS_ENABLED
class AP_ADSB_Sagetech_MXS : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    // init - performs any required initialisation for this instance
    bool init() override;

    // update - should be called periodically
    void update() override;

    // static detection function
    static bool detect();

private:

    static const uint32_t PAYLOAD_MXS_MAX_SIZE  = 255;
    static const uint8_t  START_BYTE = 0xAA;

    enum class MsgType {
        Installation            = SG_MSG_TYPE_HOST_INSTALL,
        FlightID                = SG_MSG_TYPE_HOST_FLIGHT,
        Operating               = SG_MSG_TYPE_HOST_OPMSG,
        GPS_Data                = SG_MSG_TYPE_HOST_GPS,
        Data_Request            = SG_MSG_TYPE_HOST_DATAREQ,
        // RESERVED 0x06 - 0x0A
        Target_Request          = SG_MSG_TYPE_HOST_TARGETREQ,
        Mode                    = SG_MSG_TYPE_HOST_MODE,
        // RESERVED 0x0D - 0xC1

        ACK                     = SG_MSG_TYPE_XPNDR_ACK,
        Installation_Response   = SG_MSG_TYPE_XPNDR_INSTALL,
        FlightID_Response       = SG_MSG_TYPE_XPNDR_FLIGHT,
        Status_Response         = SG_MSG_TYPE_XPNDR_STATUS,
        RESERVED_0x84           = 0x84,
        RESERVED_0x85           = 0x85,
        Mode_Settings           = SG_MSG_TYPE_XPNDR_MODE,
        RESERVED_0x8D           = 0x8D,
        Version_Response        = SG_MSG_TYPE_XPNDR_VERSION,
        Serial_Number_Response  = SG_MSG_TYPE_XPNDR_SERIALNUM,
        Target_Summary_Report   = SG_MSG_TYPE_ADSB_TSUMMARY,

        ADSB_StateVector_Report = SG_MSG_TYPE_ADSB_SVR,
        ADSB_ModeStatus_Report  = SG_MSG_TYPE_ADSB_MSR,
        TISB_StateVector_Report = 0x93,
        TISB_ModeStatus_Report  = 0x94,
        TISB_CorasePos_Report   = 0x95,
        TISB_ADSB_Mgr_Report    = 0x96,

        ADSB_Target_State_Report= SG_MSG_TYPE_ADSB_TSTATE,
        ADSB_Air_Ref_Vel_Report = SG_MSG_TYPE_ADSB_ARVR,
    };

    enum class ParseState {
        WaitingFor_Start,
        WaitingFor_MsgType,
        WaitingFor_MsgId,
        WaitingFor_PayloadLen,
        WaitingFor_PayloadContents,
        WaitingFor_Checksum,
    };

    enum class emit_T {
        aUnknown       = 0x00,
        aLight,
        aSmall,
        aLarge,
        aHighVortex,
        aHeavy,
        aPerformance,
        aRotorCraft,
        bUnknown,
        bGlider,
        bAir,
        bParachutist,
        bUltralight,
        bUAV,
        bSpace,
        cUnknown,
        cEmergency,
        cService,
        cPoint
    };

    struct __attribute__((packed)) Packet {
        const uint8_t   start = SG_MSG_START_BYTE;
        MsgType         type;
        uint8_t         id;
        uint8_t         payload_length;
        uint8_t         payload[PAYLOAD_MXS_MAX_SIZE];
        // uint8_t         checksum;
    };

    struct {
        ParseState      state;
        uint8_t         index;
        Packet          packet;
        uint8_t         checksum;
    } message_in;

    // handling inbound byte and process it in the state machine
    bool parse_byte(const uint8_t data);

    // handle inbound packet
    void handle_packet(const Packet &msg);

    // send message to serial port
    void send_msg(Packet &msg);

    // handle inbound msgs
    void handle_adsb_in_msg(const Packet &msg);
    void handle_ack(const Packet &msg);

    // send messages to to transceiver
    void sendInstallationMessage();
    void sendFlightIdMessage();
    void sendOperatingMessage();
    void sendGpsDataMessage();

    // send packet by type
    void send_packet(const MsgType type);

    // send msg to request a packet by type
    void request_packet(const MsgType type);

    // Convert base 8 or 16 to decimal. Used to convert an octal/hexadecimal value
    // stored on a GCS as a string field in different format, but then transmitted
    // over mavlink as a float which is always a decimal.
    uint32_t convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber);

    /**
     * @brief Convert APInt8_t emitterType to Sagetech Emitter Type)
     * 
     * @param emitterType 
     * @return sg_emitter_t 
     */
    sg_emitter_t convert_to_sg_emitter_type(AP_Int8 emitterType);

    /**
     * @brief Convert float max airspeed configuration to sagetech installation
     * max airspeed type.
     * 
     * @param maxAirSpeed 
     * @return sg_airspeed_t 
     */
    sg_airspeed_t convert_to_sg_airspeed_type(float maxAirSpeed);

    /**
     * @brief Writes message of len bytes to the serial device.
     * 
     * @param data 
     * @param len 
     */
    void msgWrite(uint8_t *data, uint16_t len);

    /**
     * @brief Encodes and sends a Status Data Request message.
     * 
     */
    void sendDataReq(sg_datatype_t dataReqType);


    // timers for each out-bound packet
    uint32_t        last_packet_initialize_ms;
    uint32_t        last_packet_PreFlight_ms;
    uint32_t        last_packet_GPS_ms;
    uint32_t        last_packet_Operating_ms;

    // cached variables to compare against params so we can send msg on param change.
    uint16_t        last_operating_squawk;
    int32_t         last_operating_alt;
    uint8_t         last_operating_rf_select;

    // track status changes in acks
    uint8_t         last_ack_transponder_mode;
    // Transponder_Type transponder_type = Transponder_Type::Unknown;
};
#endif // HAL_ADSB_SAGETECH_MXS_ENABLED

