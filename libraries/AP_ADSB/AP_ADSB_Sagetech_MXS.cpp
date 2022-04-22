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


#include "AP_ADSB_Sagetech_MXS.h"

#if HAL_ADSB_SAGETECH_MXS_ENABLED
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <math.h>

#define ADSB_COM_TX_BUFFER_LEN 128

// #define SAGETECH_SCALER_LATLNG              (1.0f/2.145767E-5f)     //   180/(2^23)
// #define SAGETECH_SCALER_KNOTS_TO_CMS        ((KNOTS_TO_M_PER_SEC/0.125f) * 100.0f)
// #define SAGETECH_SCALER_ALTITUDE            (1.0f/0.015625f)
// #define SAGETECH_SCALER_HEADING_CM          ((360.0f/256.0f) * 100.0f)

// #define SAGETECH_VALIDFLAG_LATLNG           (1U<<0)
// #define SAGETECH_VALIDFLAG_ALTITUDE         (1U<<1)
// #define SAGETECH_VALIDFLAG_VELOCITY         (1U<<2)
// #define SAGETECH_VALIDFLAG_GND_SPEED        (1U<<3)
// #define SAGETECH_VALIDFLAG_HEADING          (1U<<4)
// #define SAGETECH_VALIDFLAG_V_RATE_GEO       (1U<<5)
// #define SAGETECH_VALIDFLAG_V_RATE_BARO      (1U<<6)
// #define SAGETECH_VALIDFLAG_EST_LATLNG       (1U<<7)
// #define SAGETECH_VALIDFLAG_EST_VELOCITY     (1U<<8)

static uint8_t txComBuffer[ADSB_COM_TX_BUFFER_LEN];
static uint8_t msgId = 0;

// detect if any port is configured as Sagetech
bool AP_ADSB_Sagetech_MXS::detect()
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
}

// Init, called once after class is constructed
bool AP_ADSB_Sagetech_MXS::init()
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
    return (_port != nullptr);
}

/**
 * @brief The main callback function (Called with freq of 10Hz) that sends 
 * appropriate message types at specific times.
 * 
 * Read Byte from Serial Port Buffer (10Hz)
 * Send installation message (every 5 seconds)
 * Send Flight ID (every 8.2 s)
 * Send Operating Message (every second)
 * Send GPS data (flying: 5Hz, not flying: 1Hz)
 * 
 */
void AP_ADSB_Sagetech_MXS::update()
{
    if (_port == nullptr) {
        return;
    }

    // -----------------------------
    // read any available data on serial port
    // -----------------------------
    uint32_t nbytes = MIN(_port->available(), 10 * PAYLOAD_MXS_MAX_SIZE);
    while (nbytes-- > 0) {
        const int16_t data = (uint8_t)_port->read();
        if (data < 0) {
            break;
        }
        if (parse_byte((uint8_t)data)) {        // CF: The way that it works right now it auto calls handle packet in parse_byte, so is this needed?
            handle_packet(message_in.packet);
        }
    } // while nbytes


    const uint32_t now_ms = AP_HAL::millis();
    
    // -----------------------------
    // handle timers for generating data
    // -----------------------------
    if (!last_packet_initialize_ms || (now_ms - last_packet_initialize_ms >= 5000)) {
        last_packet_initialize_ms = now_ms;
        send_packet(MsgType::Installation);

    } else if (!last_packet_PreFlight_ms || (now_ms - last_packet_PreFlight_ms >= 8200)) {
        last_packet_PreFlight_ms = now_ms;
        // TODO: allow callsign to not require a reboot
        send_packet(MsgType::FlightID);

    } else if (now_ms - last_packet_Operating_ms >= 1000 && (
            last_packet_Operating_ms == 0 || // send once at boot
            // send as data changes
            last_operating_squawk != _frontend.out_state.cfg.squawk_octal ||
            abs(last_operating_alt - _frontend._my_loc.alt) > 1555 ||      // 1493cm == 49ft. The output resolution is 100ft per bit
            last_operating_rf_select != _frontend.out_state.cfg.rfSelect))
    {
        last_packet_Operating_ms = now_ms;
        last_operating_squawk = _frontend.out_state.cfg.squawk_octal;
        last_operating_alt = _frontend._my_loc.alt;
        last_operating_rf_select = _frontend.out_state.cfg.rfSelect;
        send_packet(MsgType::Operating);

    } else if (now_ms - last_packet_GPS_ms >= (_frontend.out_state.is_flying ? 200 : 1000)) {
        // 1Hz when not flying, 5Hz when flying
        last_packet_GPS_ms = now_ms;
        send_packet(MsgType::GPS_Data);
    }
}

/**
 * @brief Takes the message type provided and calls the correct
 * callback function to send the correct message type
 * 
 * @param type : MsgType to send.
 */
void AP_ADSB_Sagetech_MXS::send_packet(const MsgType type)
{
    switch (type) {
    case MsgType::Installation:
        sendInstallationMessage();
        break;
    case MsgType::FlightID:
        sendFlightIdMessage();
        break;
    case MsgType::Operating:
        sendOperatingMessage();
        break;
    case MsgType::GPS_Data:
        sendGpsDataMessage();
        break;
    default:
        break;
    }
}


void AP_ADSB_Sagetech_MXS::sendDataReq(sg_datatype_t dataReqType)
{
    sg_datareq_t dataReq;
    dataReq.reqType = dataReqType;
    sgEncodeDataReq(txComBuffer, &dataReq, msgId++);
    msgWrite(txComBuffer, SG_MSG_LEN_DATAREQ);
}

/**
 * @brief Takes incoming packets, gets their message type, and 
 * appropriately handles them with the correct callbacks.
 * 
 * @param msg Message packet received, cast into Packet type.
 */
void AP_ADSB_Sagetech_MXS::handle_packet(const Packet &msg)
{
    // TODO: Populate with correct callbacks to handle incoming messages
    switch (msg.type) {
    case MsgType::ACK:
        sg_ack_t ack;
        if (sgDecodeAck((uint8_t*) &msg, &ack))
        {
            // TODO: Handle ACK
        }
        // {    // FIXME: have I replaced this appropriately?
        //     Packet ack_msg {};
        //     memcpy(&ack_msg, msg.payload, msg.payload_length); 
        //     if (ack_msg.type != MsgType::ACK) { // sanity check so we don't cause endless recursion
        //         handle_packet(ack_msg);
        //     }
        // }
        break;

    case MsgType::Data_Request:
    case MsgType::Target_Request:
    case MsgType::Mode:
    case MsgType::Installation:
    case MsgType::FlightID:
    case MsgType::Operating:
    case MsgType::GPS_Data:
        // outbound only
        // unhandled by autopilot
        break;

    case MsgType::Installation_Response:
        sg_install_t inst;
        if (sgDecodeInstall((uint8_t*) &msg, &inst))
        {
            // TODO: Pass Install Data to AP
        }
        break;
    case MsgType::FlightID_Response:
        sg_flightid_t flightId;
        if (sgDecodeFlightId((uint8_t*) &msg, &flightId))
        {
            // TODO: Pass Flight ID to AP
        }
        break;
    case MsgType::Status_Response:
        sg_status_t status;
        if (sgDecodeStatus((uint8_t*) &msg, &status))
        {
            // TODO: Pass Status Data to AP
        }
        break;
    case MsgType::Mode_Settings:
        // FIXME: No sgModeSettings function
        break;
    case MsgType::Version_Response:
        sg_version_t version;
        if (sgDecodeVersion((uint8_t*) &msg, &version))
        {
            // TODO: Pass version data to AP
        }
        break;
    case MsgType::Serial_Number_Response:
        sg_serialnumber_t serial;
        if (sgDecodeSerialNumber((uint8_t*) &msg, &serial))
        {
            // TODO: Pass serial number to AP
        }
        break;
    case MsgType::Target_Summary_Report:
        // FIXME: No sgDecode function
        break;

    // ADSB Messages
    case MsgType::ADSB_StateVector_Report:
        sg_svr_t svr;
        if (sgDecodeSVR((uint8_t*) &msg, &svr))
        {
            // TODO: Pass SVR to AP
        }
        break;
    case MsgType::ADSB_ModeStatus_Report:
        sg_msr_t msr;
        if (sgDecodeMSR((uint8_t*) &msg, &msr))
        {
            // TODO: Pass MSR data to AP
        }
        break;
    case MsgType::RESERVED_0x84:
    case MsgType::RESERVED_0x85:
    case MsgType::RESERVED_0x8D:
    case MsgType::TISB_StateVector_Report:
    case MsgType::TISB_ModeStatus_Report:
    case MsgType::TISB_CorasePos_Report:
    case MsgType::TISB_ADSB_Mgr_Report:
    case MsgType::ADSB_Target_State_Report:
    case MsgType::ADSB_Air_Ref_Vel_Report:
        // handle_adsb_in_msg(msg);
        // AFAIK we don't handle these.
        break;
    }
}


/**
 * @brief Received ADSB-in message callback function.
 * 
 * @param msg Received packet.
 */
void AP_ADSB_Sagetech_MXS::handle_adsb_in_msg(const Packet &msg)
{
    // AP_ADSB::adsb_vehicle_t vehicle {};

    // vehicle.last_update_ms = AP_HAL::millis();

    // switch (msg.type) {
    // case MsgType::ADSB_StateVector_Report: { // 0x91
    //     const uint16_t validFlags = le16toh_ptr(&msg.payload[8]);
    //     vehicle.info.ICAO_address = le24toh_ptr(&msg.payload[10]);

    //     if (validFlags & SAGETECH_VALIDFLAG_LATLNG) {
    //         vehicle.info.lat = ((int32_t)le24toh_ptr(&msg.payload[20])) * SAGETECH_SCALER_LATLNG;
    //         vehicle.info.lon = ((int32_t)le24toh_ptr(&msg.payload[23])) * SAGETECH_SCALER_LATLNG;
    //         vehicle.info.flags |= ADSB_FLAGS_VALID_COORDS;
    //     }

    //     if (validFlags & SAGETECH_VALIDFLAG_ALTITUDE) {
    //         vehicle.info.altitude = (int32_t)le24toh_ptr(&msg.payload[26]);
    //         vehicle.info.flags |= ADSB_FLAGS_VALID_ALTITUDE;
    //     }

    //     if (validFlags & SAGETECH_VALIDFLAG_VELOCITY) {
    //         const float velNS = ((int32_t)le16toh_ptr(&msg.payload[29])) * SAGETECH_SCALER_KNOTS_TO_CMS;
    //         const float velEW = ((int32_t)le16toh_ptr(&msg.payload[31])) * SAGETECH_SCALER_KNOTS_TO_CMS;
    //         vehicle.info.hor_velocity = Vector2f(velEW, velNS).length();
    //         vehicle.info.flags |= ADSB_FLAGS_VALID_VELOCITY;
    //     }

    //     if (validFlags & SAGETECH_VALIDFLAG_HEADING) {
    //         vehicle.info.heading = ((float)msg.payload[29]) * SAGETECH_SCALER_HEADING_CM;
    //         vehicle.info.flags |= ADSB_FLAGS_VALID_HEADING;
    //     }

    //     if ((validFlags & SAGETECH_VALIDFLAG_V_RATE_GEO) || (validFlags & SAGETECH_VALIDFLAG_V_RATE_BARO)) {
    //         vehicle.info.ver_velocity = (int16_t)le16toh_ptr(&msg.payload[38]);
    //         vehicle.info.flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
    //     }

    //     _frontend.handle_adsb_vehicle(vehicle);
    //     break;
    // }
    // case MsgType::ADSB_ModeStatus_Report:   // 0x92
    //     vehicle.info.ICAO_address = le24toh_ptr(&msg.payload[9]);

    //     if (msg.payload[16] != 0) {
    //         // if string is non-null, consider it valid
    //         memcpy(&vehicle.info, &msg.payload[16], 8);
    //         vehicle.info.flags |= ADSB_FLAGS_VALID_CALLSIGN;
    //     }

    //     _frontend.handle_adsb_vehicle(vehicle);
    //     break;
    // case MsgType::TISB_StateVector_Report:
    // case MsgType::TISB_ModeStatus_Report:
    // case MsgType::TISB_CorasePos_Report:
    // case MsgType::TISB_ADSB_Mgr_Report:
    //     // TODO
    //     return;

    // default:
    //     return;
    // }

}

/**
 * @brief Handles an incoming byte and processes it through the state
 * machine to determine if end of message is reached.
 * 
 * @param data : incoming byte
 * @return true : if a full packet has been received
 * @return false : if not yet reached packet termination
 */
bool AP_ADSB_Sagetech_MXS::parse_byte(const uint8_t data)
{
    switch (message_in.state) {
        default:
        case ParseState::WaitingFor_Start:
            if (data == START_BYTE) {
                message_in.checksum = data; // initialize checksum here
                message_in.state = ParseState::WaitingFor_MsgType;
            }
            break;

        case ParseState::WaitingFor_MsgType:
            message_in.checksum += data;
            message_in.packet.type = static_cast<MsgType>(data);
            message_in.state = ParseState::WaitingFor_MsgId;
            break;

        case ParseState::WaitingFor_MsgId:
            message_in.checksum += data;
            message_in.packet.id = data;
            message_in.state = ParseState::WaitingFor_PayloadLen;
            break;

        case ParseState::WaitingFor_PayloadLen:
            message_in.checksum += data;
            message_in.packet.payload_length = data;
            message_in.index = 0;
            message_in.state = (data == 0) ? ParseState::WaitingFor_Checksum : ParseState::WaitingFor_PayloadContents;
            break;

        case ParseState::WaitingFor_PayloadContents:
            message_in.checksum += data; // initialize checksum here
            message_in.packet.payload[message_in.index++] = data;
            if (message_in.index >= message_in.packet.payload_length) {
                message_in.state = ParseState::WaitingFor_Checksum;
            }
            break;

        // FIXME: Does this append the checksum to the payload?
        case ParseState::WaitingFor_Checksum:
            message_in.state = ParseState::WaitingFor_Start;
            if (message_in.checksum == data) {
                // append the checksum to the payload and zero out the payload index
                message_in.packet.payload[message_in.index] = data;
                message_in.index = 0;
                // message_in.packet.checksum = data;       // Doing it this way would put it after the max payload length.
                handle_packet(message_in.packet);
            }
            break;
    }
    // FIXME: Where does it return true?
    return false;
}

/**
 * @brief Takes a raw buffer and writes it out to the device port.
 * 
 * @param data : pointer to data buffer
 * @param len : number of bytes to write
 */
void AP_ADSB_Sagetech_MXS::msgWrite(uint8_t *data, uint16_t len)
{
    if (_port != nullptr) {
        _port->write(data, len);
    }
}

/**
 * @brief Callback for sending an installation message.
 * 
 */
void AP_ADSB_Sagetech_MXS::sendInstallationMessage()
{
    sg_install_t inst;
    inst.icao = _frontend.out_state.cfg.ICAO_id;
    memcpy(&inst.reg, &_frontend.out_state.cfg.callsign, 8);    // FIXME: Is callsign same as registration? Callsign is 8 alphanumeric characters left justified and padded with spaces
    inst.com0 = (sg_baud_t) baud57600;
    inst.com1 = (sg_baud_t) baud57600;
    
    // FIXME: Figure out what default values to set this to.
    inst.eth.ipAddress = 0;
    inst.eth.subnetMask = 0;
    inst.eth.portNumber = 0;

    // TODO: Figure out GPS integrity parameters
    inst.sil = (sg_sil_t) silLow;
    inst.sda = (sg_sda_t) sdaMinor;
    inst.emitter = convert_to_sg_emitter_type(_frontend.out_state.cfg.emitterType);
    inst.size = (sg_size_t) ((int8_t) _frontend.out_state.cfg.lengthWidth);
    inst.maxSpeed = convert_to_sg_airspeed_type(_frontend.out_state.cfg.maxAircraftSpeed_knots);
    inst.altOffset = 0;     // Alt encoder offset is legacy field that should always be 0.
    // FIXME: Unsure how to get antenna configurations
    inst.antenna = (sg_antenna_t) antBottom;

    // FIXME: Figure out how to determine booleans
    inst.altRes100 = true;
    inst.hdgTrueNorth = false;
    inst.airspeedTrue = false;
    inst.heater = false;
    inst.wowConnected = true;

    sgEncodeInstall(txComBuffer, &inst, msgId++);
    msgWrite(txComBuffer, SG_MSG_LEN_INSTALL);
}

/**
 * @brief Callback for sending a FlightID message
 * 
 */
void AP_ADSB_Sagetech_MXS::sendFlightIdMessage()
{

    sg_flightid_t flightId;
    memcpy(&flightId.flightId, &_frontend.out_state.cfg.callsign, 9);       // Copy 9 character callsign
    sgEncodeFlightId(txComBuffer, &flightId, msgId++);
    msgWrite(txComBuffer, SG_MSG_LEN_FLIGHT);
}

/**
 * @brief Callback for sending an operating message.
 * 
 */
void AP_ADSB_Sagetech_MXS::sendOperatingMessage()
{
    // Declare Operating Message Type
    sg_operating_t op;

    // Populate operating message structure
    op.squawk = convert_base_to_decimal(8, last_operating_squawk);
    op.opMode = (sg_op_mode_t) modeOff;     // FIXME: Figure out how to update/get the OpMode
                                            // Is this rfSelect/last_operating_rf_select?
    op.savePowerUp = true;      // Save power-up state in non-volatile
    op.enableSqt = false;       // Enable extended squitters
    op.enableXBit = false;      // Enable the x-bit
    op.milEmergency = false;    // Broadcast a military emergency
    op.emergcType = (sg_emergc_t) emergcNone; // Enumerated civilian emergency type

    op.altUseIntrnl = true;     // True = Report altitude from internal pressure sensor
                                // (will ignore other bits in the field)
    // we are using internal, so these don't need to be set
    op.altHostAvlbl = false;    // Host Altitude is being provided
    op.altRes25 = false;        // Host Altitude Resolution from install
    op.altitude = 0;            // Sea-level altitude in feet. Field is ignored when internal altitude is selected

    op.climbValid = false;      // Climb rate is provided (LUA script AHRS does not provide climb rate)
    op.climbRate = 0;           // Climb rate in ft/min. Limits are +/- 16,448 ft/min.
    
    op.headingValid = false;
    op.airspdValid = false;
    const Vector2f speed = AP::ahrs().groundspeed_vector();
    if (speed != Vector2f(0.0f, 0.0f))      // If groundspeed vector is not default values, set valid flags
    {
        op.headingValid = true;
        op.airspdValid = true;
    }
    uint16_t speed_knots = speed.length() * M_PER_SEC_TO_KNOTS;
    double heading = wrap_360(degrees(speed.angle()));
    op.airspd = speed_knots;
    op.heading = heading;

    // Encode the operating message object
    sgEncodeOperating(txComBuffer, &op, msgId++);
    msgWrite(txComBuffer, SG_MSG_LEN_OPMSG);
}

/**
 * @brief Callback for sending a GPS data message
 * 
 */
void AP_ADSB_Sagetech_MXS::sendGpsDataMessage()
{
    sg_gps_t gps;

    // Populate the GPS object
    // Realistic but arbitrary RAIM values
    gps.hpl = 12.0;
    gps.hfom = 23.0;
    gps.vfom = 33.0;
    gps.nacv = (sg_nacv_t) nacv3dot0;

    // Get Vehicle Longitude and Lattidue and Convert to string
    const int32_t longitude = _frontend._my_loc.lng;
    const int32_t latitude =  _frontend._my_loc.lat;
    const double lon_deg = longitude * (double)1.0e-7 * (longitude < 0 ? -1 : 1);
    const double lon_minutes = (lon_deg - int(lon_deg)) * 60;
    snprintf((char*)&gps.longitude, 12, "%03u%02u.%05u", (unsigned)lon_deg, (unsigned)lon_minutes, unsigned((lon_minutes - (int)lon_minutes) * 1.0E5));

    const double lat_deg = latitude * (double)1.0e-7 * (latitude < 0 ? -1 : 1);
    const double lat_minutes = (lat_deg - int(lat_deg)) * 60;
    snprintf((char*)&gps.latitude, 11, "%02u%02u.%05u", (unsigned)lat_deg, (unsigned)lat_minutes, unsigned((lat_minutes - (int)lat_minutes) * 1.0E5));

    // ground speed
    const Vector2f speed = AP::ahrs().groundspeed_vector();
    float speed_knots = speed.length() * M_PER_SEC_TO_KNOTS;
    snprintf((char*)&gps.grdSpeed, 7, "%03u.%02u", (unsigned)speed_knots, unsigned((speed_knots - (int)speed_knots) * 1.0E2));

    // heading
    float heading = wrap_360(degrees(speed.angle()));
    snprintf((char*)&gps.grdTrack, 9, "%03u.%04u", unsigned(heading), unsigned((heading - (int)heading) * 1.0E4));

    gps.latNorth = (latitude >= 0 ? true: false);
    gps.lngEast = (longitude >= 0 ? true: false);

    gps.gpsValid = (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) ? false : true;  // If the status is not OK, gpsValid is false.

    uint64_t time_usec;
    if (AP::rtc().get_utc_usec(time_usec)) {
        // not completely accurate, our time includes leap seconds and time_t should be without
        const time_t time_sec = time_usec / 1000000;
        struct tm* tm = gmtime(&time_sec);

        // format time string
        snprintf((char*)&gps.timeOfFix, 11, "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (time_usec % 1000000) * 1.0e-6);
    } else {
        memset(&gps.timeOfFix,' ', 10);
    }

    // Encode GPS and Send It
    sgEncodeGPS(txComBuffer, &gps, msgId++);
    msgWrite(txComBuffer, sizeof(sg_gps_t));
}


/*
 * Convert base 8 or 16 to decimal. Used to convert an octal/hexadecimal value stored on a GCS as a string field in different format, but then transmitted over mavlink as a float which is always a decimal.
 * baseIn: base of input number
 * inputNumber: value currently in base "baseIn" to be converted to base "baseOut"
 *
 * Example: convert ADSB squawk octal "1200" stored in memory as 0x0280 to 0x04B0
 *          uint16_t squawk_decimal = convertMathBase(8, squawk_octal);
 */
uint32_t AP_ADSB_Sagetech_MXS::convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber)
{
    // Our only sensible input bases are 16 and 8
    if (baseIn != 8 && baseIn != 16) {
        return inputNumber;
    }

    uint32_t outputNumber = 0;
    for (uint8_t i=0; inputNumber != 0; i++) {
        outputNumber += (inputNumber % 10) * powf(10, i);
        inputNumber /= 10;
    }
    return outputNumber;
}

sg_emitter_t AP_ADSB_Sagetech_MXS::convert_to_sg_emitter_type(AP_Int8 emitterType)
{
    if (emitterType < 8) {
        return (sg_emitter_t) ((int8_t) emitterType);
    } else if (emitterType < 13) {
        return (sg_emitter_t) ((int8_t) emitterType + SG_EMIT_OFFSET_B);
    } else if (emitterType < 15) {
        return (sg_emitter_t) ((int8_t) emitterType + SG_EMIT_OFFSET_B + 6);
    } else if (emitterType < 21) {
        return (sg_emitter_t) ((int8_t) emitterType + SG_EMIT_OFFSET_C);
    } else {
        return (sg_emitter_t) SG_EMIT_OFFSET_D;
    }
}

sg_airspeed_t AP_ADSB_Sagetech_MXS::convert_to_sg_airspeed_type(float maxAirSpeed)
{
    int airspeed = (int) maxAirSpeed;
    if (airspeed < 0) {
        return (sg_airspeed_t) speedUnknown;
    } else if (airspeed < 75) {
        return (sg_airspeed_t) speed75kt;
    } else if (airspeed < 150) {
        return (sg_airspeed_t) speed150kt;
    } else if (airspeed < 300) {
        return (sg_airspeed_t) speed300kt;
    } else if (airspeed < 600) {
        return (sg_airspeed_t) speed600kt;
    } else if (airspeed < 1200) {
        return (sg_airspeed_t) speed1200kt;
    } else if (airspeed >= 1200) {
        return (sg_airspeed_t) speedGreater;
    } else {
        return (sg_airspeed_t) speedUnknown;
    }
}

#endif // HAL_ADSB_SAGETECH_MXS_ENABLED

