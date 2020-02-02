/*
	This is a library written for the Ublox ZED-F9P and NEO-M8P-2
	SparkFun sells these at its website: www.sparkfun.com
	Do you like this library? Help support SparkFun. Buy a board!
	https://www.sparkfun.com/products/15136
	https://www.sparkfun.com/products/15005
	https://www.sparkfun.com/products/15733
	https://www.sparkfun.com/products/15193
	https://www.sparkfun.com/products/15210

	Written by Nathan Seidle @ SparkFun Electronics, September 6th, 2018

	This library handles configuring and handling the responses 
	from a Ublox GPS module. Works with most modules from Ublox including
	the Zed-F9P, NEO-M8P-2, NEO-M9N, ZOE-M8Q, SAM-M8Q, and many others.

	https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

	Development environment specifics:
	Arduino IDE 1.8.5

	SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
	The MIT License (MIT)
	Copyright (c) 2016 SparkFun Electronics
	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
	associated documentation files (the "Software"), to deal in the Software without restriction, 
	including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
	and/or sell copies of the Software, and to permit persons to whom the Software is furnished to 
	do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial 
	portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
	NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
	IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SPARKFUN_UBLOX_ARDUINO_LIBRARY_HPP
#define SPARKFUN_UBLOX_ARDUINO_LIBRARY_HPP

/******************************************************************************
 Includes
 *****************************************************************************/

#include <stdint.h>
#include <ti/drivers/I2C.h>

#include "ArduinoUtils.hpp"

//Platform specific configurations

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#ifndef I2C_BUFFER_LENGTH
//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32
#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define UBLOX_DEFAULT_I2C_ADDRESS 0x42 //Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series

//Define a digital pin to aid checksum failure capture and analysis
//Leave set to -1 if not needed
const int checksumFailurePin = -1;

// Global Status Returns
typedef enum
{
	SFE_UBLOX_STATUS_SUCCESS,
	SFE_UBLOX_STATUS_FAIL,
	SFE_UBLOX_STATUS_CRC_FAIL,
	SFE_UBLOX_STATUS_TIMEOUT,
	SFE_UBLOX_STATUS_COMMAND_UNKNOWN,
	SFE_UBLOX_STATUS_OUT_OF_RANGE,
	SFE_UBLOX_STATUS_INVALID_ARG,
	SFE_UBLOX_STATUS_INVALID_OPERATION,
	SFE_UBLOX_STATUS_MEM_ERR,
	SFE_UBLOX_STATUS_HW_ERR,
	SFE_UBLOX_STATUS_DATA_SENT,
	SFE_UBLOX_STATUS_DATA_RECEIVED,
	SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
} sfe_ublox_status_e;

//Registers
const byte UBX_SYNCH_1 = 0xB5;
const byte UBX_SYNCH_2 = 0x62;

//The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
const byte UBX_CLASS_NAV = 0x01;  //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const byte UBX_CLASS_RXM = 0x02;  //Receiver Manager Messages: Satellite Status, RTC Status
const byte UBX_CLASS_INF = 0x04;  //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
const byte UBX_CLASS_ACK = 0x05;  //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
const byte UBX_CLASS_CFG = 0x06;  //Configuration Input Messages: Configure the receiver.
const byte UBX_CLASS_UPD = 0x09;  //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
const byte UBX_CLASS_MON = 0x0A;  //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const byte UBX_CLASS_AID = 0x0B;  //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
const byte UBX_CLASS_TIM = 0x0D;  //Timing Messages: Time Pulse Output, Time Mark Results
const byte UBX_CLASS_ESF = 0x10;  //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
const byte UBX_CLASS_MGA = 0x13;  //Multiple GNSS Assistance Messages: Assistance data for various GNSS
const byte UBX_CLASS_LOG = 0x21;  //Logging Messages: Log creation, deletion, info and retrieval
const byte UBX_CLASS_SEC = 0x27;  //Security Feature Messages
const byte UBX_CLASS_HNR = 0x28;  //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
const byte UBX_CLASS_NMEA = 0xF0; //NMEA Strings: standard NMEA strings

//The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
const byte UBX_CFG_ANT = 0x13;		//Antenna Control Settings. Used to configure the antenna control settings
const byte UBX_CFG_BATCH = 0x93;		//Get/set data batching configuration.
const byte UBX_CFG_CFG = 0x09;		//Clear, Save, and Load Configurations. Used to save current configuration
const byte UBX_CFG_DAT = 0x06;		//Set User-defined Datum or The currently defined Datum
const byte UBX_CFG_DGNSS = 0x70;		//DGNSS configuration
const byte UBX_CFG_GEOFENCE = 0x69;  //Geofencing configuration. Used to configure a geofence
const byte UBX_CFG_GNSS = 0x3E;		//GNSS system configuration
const byte UBX_CFG_INF = 0x02;		//Depending on packet length, either: poll configuration for one protocol, or information message configuration
const byte UBX_CFG_ITFM = 0x39;		//Jamming/Interference Monitor configuration
const byte UBX_CFG_LOGFILTER = 0x47; //Data Logger Configuration
const byte UBX_CFG_MSG = 0x01;		//Poll a message configuration, or Set Message Rate(s), or Set Message Rate
const byte UBX_CFG_NAV5 = 0x24;		//Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
const byte UBX_CFG_NAVX5 = 0x23;		//Navigation Engine Expert Settings
const byte UBX_CFG_NMEA = 0x17;		//Extended NMEA protocol configuration V1
const byte UBX_CFG_ODO = 0x1E;		//Odometer, Low-speed COG Engine Settings
const byte UBX_CFG_PM2 = 0x3B;		//Extended power management configuration
const byte UBX_CFG_PMS = 0x86;		//Power mode setup
const byte UBX_CFG_PRT = 0x00;		//Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
const byte UBX_CFG_PWR = 0x57;		//Put receiver in a defined power state
const byte UBX_CFG_RATE = 0x08;		//Navigation/Measurement Rate Settings. Used to set port baud rates.
const byte UBX_CFG_RINV = 0x34;		//Contents of Remote Inventory
const byte UBX_CFG_RST = 0x04;		//Reset Receiver / Clear Backup Data Structures. Used to reset device.
const byte UBX_CFG_RXM = 0x11;		//RXM configuration
const byte UBX_CFG_SBAS = 0x16;		//SBAS configuration
const byte UBX_CFG_TMODE3 = 0x71;	//Time Mode Settings 3. Used to enable Survey In Mode
const byte UBX_CFG_TP5 = 0x31;		//Time Pulse Parameters
const byte UBX_CFG_USB = 0x1B;		//USB Configuration
const byte UBX_CFG_VALDEL = 0x8C;	//Used for config of higher version Ublox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
const byte UBX_CFG_VALGET = 0x8B;	//Used for config of higher version Ublox modules (ie protocol v27 and above). Configuration Items
const byte UBX_CFG_VALSET = 0x8A;	//Used for config of higher version Ublox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

//The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
const byte UBX_NMEA_MSB = 0xF0;  //All NMEA enable commands have 0xF0 as MSB
const byte UBX_NMEA_DTM = 0x0A;  //GxDTM (datum reference)
const byte UBX_NMEA_GAQ = 0x45;  //GxGAQ (poll a standard message (if the current talker ID is GA))
const byte UBX_NMEA_GBQ = 0x44;  //GxGBQ (poll a standard message (if the current Talker ID is GB))
const byte UBX_NMEA_GBS = 0x09;  //GxGBS (GNSS satellite fault detection)
const byte UBX_NMEA_GGA = 0x00;  //GxGGA (Global positioning system fix data)
const byte UBX_NMEA_GLL = 0x01;  //GxGLL (latitude and long, whith time of position fix and status)
const byte UBX_NMEA_GLQ = 0x43;  //GxGLQ (poll a standard message (if the current Talker ID is GL))
const byte UBX_NMEA_GNQ = 0x42;  //GxGNQ (poll a standard message (if the current Talker ID is GN))
const byte UBX_NMEA_GNS = 0x0D;  //GxGNS (GNSS fix data)
const byte UBX_NMEA_GPQ = 0x040; //GxGPQ (poll a standard message (if the current Talker ID is GP))
const byte UBX_NMEA_GRS = 0x06;  //GxGRS (GNSS range residuals)
const byte UBX_NMEA_GSA = 0x02;  //GxGSA (GNSS DOP and Active satellites)
const byte UBX_NMEA_GST = 0x07;  //GxGST (GNSS Pseudo Range Error Statistics)
const byte UBX_NMEA_GSV = 0x03;  //GxGSV (GNSS satellites in view)
const byte UBX_NMEA_RMC = 0x04;  //GxRMC (Recommended minimum data)
const byte UBX_NMEA_TXT = 0x41;  //GxTXT (text transmission)
const byte UBX_NMEA_VLW = 0x0F;  //GxVLW (dual ground/water distance)
const byte UBX_NMEA_VTG = 0x05;  //GxVTG (course over ground and Ground speed)
const byte UBX_NMEA_ZDA = 0x08;  //GxZDA (Time and Date)

//The following are used to configure the NMEA protocol main talker ID and GSV talker ID
const byte UBX_NMEA_MAINTALKERID_NOTOVERRIDDEN = 0x00; //main talker ID is system dependent
const byte UBX_NMEA_MAINTALKERID_GP = 0x01;			  //main talker ID is GPS
const byte UBX_NMEA_MAINTALKERID_GL = 0x02;			  //main talker ID is GLONASS
const byte UBX_NMEA_MAINTALKERID_GN = 0x03;			  //main talker ID is combined receiver
const byte UBX_NMEA_MAINTALKERID_GA = 0x04;			  //main talker ID is Galileo
const byte UBX_NMEA_MAINTALKERID_GB = 0x05;			  //main talker ID is BeiDou
const byte UBX_NMEA_GSVTALKERID_GNSS = 0x00;			  //GNSS specific Talker ID (as defined by NMEA)
const byte UBX_NMEA_GSVTALKERID_MAIN = 0x01;			  //use the main Talker ID

//The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const byte UBX_INF_CLASS = 0x04;   //All INF messages have 0x04 as the class
const byte UBX_INF_DEBUG = 0x04;   //ASCII output with debug contents
const byte UBX_INF_ERROR = 0x00;   //ASCII output with error contents
const byte UBX_INF_NOTICE = 0x02;  //ASCII output with informational contents
const byte UBX_INF_TEST = 0x03;	//ASCII output with test contents
const byte UBX_INF_WARNING = 0x01; //ASCII output with warning contents

//The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const byte UBX_LOG_CREATE = 0x07;		   //Create Log File
const byte UBX_LOG_ERASE = 0x03;			   //Erase Logged Data
const byte UBX_LOG_FINDTIME = 0x0E;		   //Find index of a log entry based on a given time, or response to FINDTIME requested
const byte UBX_LOG_INFO = 0x08;			   //Poll for log information, or Log information
const byte UBX_LOG_RETRIEVEPOSEXTRA = 0x0F; //Odometer log entry
const byte UBX_LOG_RETRIEVEPOS = 0x0B;	  //Position fix log entry
const byte UBX_LOG_RETRIEVESTRING = 0x0D;   //Byte string log entry
const byte UBX_LOG_RETRIEVE = 0x09;		   //Request log data
const byte UBX_LOG_STRING = 0x04;		   //Store arbitrary string on on-board flash

//The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const byte UBX_MGA_ACK_DATA0 = 0x60;		 //Multiple GNSS Acknowledge message
const byte UBX_MGA_BDS_EPH = 0x03;		 //BDS Ephemeris Assistance
const byte UBX_MGA_BDS_ALM = 0x03;		 //BDS Almanac Assistance
const byte UBX_MGA_BDS_HEALTH = 0x03;	 //BDS Health Assistance
const byte UBX_MGA_BDS_UTC = 0x03;		 //BDS UTC Assistance
const byte UBX_MGA_BDS_IONO = 0x03;		 //BDS Ionospheric Assistance
const byte UBX_MGA_DBD = 0x80;			 //Either: Poll the Navigation Database, or Navigation Database Dump Entry
const byte UBX_MGA_GAL_EPH = 0x02;		 //Galileo Ephemeris Assistance
const byte UBX_MGA_GAL_ALM = 0x02;		 //Galileo Almanac Assitance
const byte UBX_MGA_GAL_TIMOFFSET = 0x02;  //Galileo GPS time offset assistance
const byte UBX_MGA_GAL_UTC = 0x02;		 //Galileo UTC Assistance
const byte UBX_MGA_GLO_EPH = 0x06;		 //GLONASS Ephemeris Assistance
const byte UBX_MGA_GLO_ALM = 0x06;		 //GLONASS Almanac Assistance
const byte UBX_MGA_GLO_TIMEOFFSET = 0x06; //GLONASS Auxiliary Time Offset Assistance
const byte UBX_MGA_GPS_EPH = 0x00;		 //GPS Ephemeris Assistance
const byte UBX_MGA_GPS_ALM = 0x00;		 //GPS Almanac Assistance
const byte UBX_MGA_GPS_HEALTH = 0x00;	 //GPS Health Assistance
const byte UBX_MGA_GPS_UTC = 0x00;		 //GPS UTC Assistance
const byte UBX_MGA_GPS_IONO = 0x00;		 //GPS Ionosphere Assistance
const byte UBX_MGA_INI_POS_XYZ = 0x40;	//Initial Position Assistance
const byte UBX_MGA_INI_POS_LLH = 0x40;	//Initial Position Assitance
const byte UBX_MGA_INI_TIME_UTC = 0x40;   //Initial Time Assistance
const byte UBX_MGA_INI_TIME_GNSS = 0x40;  //Initial Time Assistance
const byte UBX_MGA_INI_CLKD = 0x40;		 //Initial Clock Drift Assitance
const byte UBX_MGA_INI_FREQ = 0x40;		 //Initial Frequency Assistance
const byte UBX_MGA_INI_EOP = 0x40;		 //Earth Orientation Parameters Assistance
const byte UBX_MGA_QZSS_EPH = 0x05;		 //QZSS Ephemeris Assistance
const byte UBX_MGA_QZSS_ALM = 0x05;		 //QZSS Almanac Assistance
const byte UBX_MGA_QZAA_HEALTH = 0x05;	//QZSS Health Assistance

//The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
const byte UBX_MON_COMMS = 0x36; //Comm port information
const byte UBX_MON_GNSS = 0x28;  //Information message major GNSS selection
const byte UBX_MON_HW2 = 0x0B;   //Extended Hardware Status
const byte UBX_MON_HW3 = 0x37;   //HW I/O pin information
const byte UBX_MON_HW = 0x09;	//Hardware Status
const byte UBX_MON_IO = 0x02;	//I/O Subsystem Status
const byte UBX_MON_MSGPP = 0x06; //Message Parse and Process Status
const byte UBX_MON_PATCH = 0x27; //Output information about installed patches
const byte UBX_MON_RF = 0x38;	//RF information
const byte UBX_MON_RXBUF = 0x07; //Receiver Buffer Status
const byte UBX_MON_RXR = 0x21;   //Receiver Status Information
const byte UBX_MON_TXBUF = 0x08; //Transmitter Buffer Status. Used for query tx buffer size/state.
const byte UBX_MON_VER = 0x04;   //Receiver/Software Version. Used for obtaining Protocol Version.

//The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
const byte UBX_NAV_CLOCK = 0x22;		//Clock Solution
const byte UBX_NAV_DOP = 0x04;		//Dilution of precision
const byte UBX_NAV_EOE = 0x61;		//End of Epoch
const byte UBX_NAV_GEOFENCE = 0x39;  //Geofencing status. Used to poll the geofence status
const byte UBX_NAV_HPPOSECEF = 0x13; //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
const byte UBX_NAV_HPPOSLLH = 0x14;  //High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
const byte UBX_NAV_ODO = 0x09;		//Odometer Solution
const byte UBX_NAV_ORB = 0x34;		//GNSS Orbit Database Info
const byte UBX_NAV_POSECEF = 0x01;   //Position Solution in ECEF
const byte UBX_NAV_POSLLH = 0x02;	//Geodetic Position Solution
const byte UBX_NAV_PVT = 0x07;		//All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
const byte UBX_NAV_RELPOSNED = 0x3C; //Relative Positioning Information in NED frame
const byte UBX_NAV_RESETODO = 0x10;  //Reset odometer
const byte UBX_NAV_SAT = 0x35;		//Satellite Information
const byte UBX_NAV_SIG = 0x43;		//Signal Information
const byte UBX_NAV_STATUS = 0x03;	//Receiver Navigation Status
const byte UBX_NAV_SVIN = 0x3B;		//Survey-in data. Used for checking Survey In status
const byte UBX_NAV_TIMEBDS = 0x24;   //BDS Time Solution
const byte UBX_NAV_TIMEGAL = 0x25;   //Galileo Time Solution
const byte UBX_NAV_TIMEGLO = 0x23;   //GLO Time Solution
const byte UBX_NAV_TIMEGPS = 0x20;   //GPS Time Solution
const byte UBX_NAV_TIMELS = 0x26;	//Leap second event information
const byte UBX_NAV_TIMEUTC = 0x21;   //UTC Time Solution
const byte UBX_NAV_VELECEF = 0x11;   //Velocity Solution in ECEF
const byte UBX_NAV_VELNED = 0x12;	//Velocity Solution in NED

//The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const byte UBX_RXM_MEASX = 0x14; //Satellite Measurements for RRLP
const byte UBX_RXM_PMREQ = 0x41; //Requests a Power Management task (two differenent packet sizes)
const byte UBX_RXM_RAWX = 0x15;  //Multi-GNSS Raw Measurement Data
const byte UBX_RXM_RLM = 0x59;   //Galileo SAR Short-RLM report (two different packet sizes)
const byte UBX_RXM_RTCM = 0x32;  //RTCM input status
const byte UBX_RXM_SFRBX = 0x13; //Boradcast Navigation Data Subframe

//The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const byte UBX_SEC_UNIQID = 0x03; //Unique chip ID

//The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const byte UBX_TIM_TM2 = 0x03;  //Time mark data
const byte UBX_TIM_TP = 0x01;   //Time Pulse Timedata
const byte UBX_TIM_VRFY = 0x06; //Sourced Time Verification

//The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
const byte UBX_UPD_SOS = 0x14; //Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup

//The following are used to enable RTCM messages
const byte UBX_RTCM_MSB = 0xF5;  //All RTCM enable commands have 0xF5 as MSB
const byte UBX_RTCM_1005 = 0x05; //Stationary RTK reference ARP
const byte UBX_RTCM_1074 = 0x4A; //GPS MSM4
const byte UBX_RTCM_1077 = 0x4D; //GPS MSM7
const byte UBX_RTCM_1084 = 0x54; //GLONASS MSM4
const byte UBX_RTCM_1087 = 0x57; //GLONASS MSM7
const byte UBX_RTCM_1094 = 0x5E; //Galileo MSM4
const byte UBX_RTCM_1124 = 0x7C; //BeiDou MSM4
const byte UBX_RTCM_1230 = 0xE6; //GLONASS code-phase biases, set to once every 10 seconds

const byte UBX_ACK_NACK = 0x00;
const byte UBX_ACK_ACK = 0x01;
const byte UBX_ACK_NONE = 0x02; //Not a real value

const byte SVIN_MODE_DISABLE = 0x00;
const byte SVIN_MODE_ENABLE = 0x01;

//The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
const byte COM_PORT_I2C = 0;
const byte COM_PORT_UART1 = 1;
const byte COM_PORT_UART2 = 2;
const byte COM_PORT_USB = 3;
const byte COM_PORT_SPI = 4;

const byte COM_TYPE_UBX = (1 << 0);
const byte COM_TYPE_NMEA = (1 << 1);
const byte COM_TYPE_RTCM3 = (1 << 5);

//The following consts are used to generate KEY values for the advanced protocol functions of VELGET/SET/DEL
const byte VAL_SIZE_1 = 0x01;  //One bit
const byte VAL_SIZE_8 = 0x02;  //One byte
const byte VAL_SIZE_16 = 0x03; //Two bytes
const byte VAL_SIZE_32 = 0x04; //Four bytes
const byte VAL_SIZE_64 = 0x05; //Eight bytes

//These are the Bitfield layers definitions for the UBX-CFG-VALSET message (not to be confused with Bitfield deviceMask in UBX-CFG-CFG)
const byte VAL_LAYER_RAM = (1 << 0);
const byte VAL_LAYER_BBR = (1 << 1);
const byte VAL_LAYER_FLASH = (1 << 2);

//Below are various Groups, IDs, and sizes for various settings
//These can be used to call getVal/setVal/delVal
const byte VAL_GROUP_I2COUTPROT = 0x72;
const byte VAL_GROUP_I2COUTPROT_SIZE = VAL_SIZE_1; //All fields in I2C group are currently 1 bit

const byte VAL_ID_I2COUTPROT_UBX = 0x01;
const byte VAL_ID_I2COUTPROT_NMEA = 0x02;
const byte VAL_ID_I2COUTPROT_RTCM3 = 0x03;

const byte VAL_GROUP_I2C = 0x51;
const byte VAL_GROUP_I2C_SIZE = VAL_SIZE_8; //All fields in I2C group are currently 1 byte

const byte VAL_ID_I2C_ADDRESS = 0x01;

enum dynModel // Possible values for the dynamic platform model, which provide more accuract position output for the situation. Description extracted from ZED-F9P Integration Manual
{
	DYN_MODEL_PORTABLE = 0, //Applications with low acceleration, e.g. portable devices. Suitable for most situations.
	// 1 is not defined
	DYN_MODEL_STATIONARY = 2, //Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
	DYN_MODEL_PEDESTRIAN,	 //Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
	DYN_MODEL_AUTOMOTIVE,	 //Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
	DYN_MODEL_SEA,			  //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
	DYN_MODEL_AIRBORNE1g,	 //Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
	DYN_MODEL_AIRBORNE2g,	 //Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
	DYN_MODEL_AIRBORNE4g,	 //Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
	DYN_MODEL_WRIST,		  // Not supported in protocol versions less than 18. Only recommended for wrist worn applications. Receiver will filter out arm motion.
	DYN_MODEL_BIKE,			  // Supported in protocol versions 19.2
};

#ifndef MAX_PAYLOAD_SIZE

#define MAX_PAYLOAD_SIZE 256 //We need ~220 bytes for getProtocolVersion on most ublox modules
//#define MAX_PAYLOAD_SIZE 768 //Worst case: UBX_CFG_VALSET packet with 64 keyIDs each with 64 bit values

#endif

//-=-=-=-=- UBX binary specific variables
typedef struct
{
	byte cls;
	byte id;
	uint16_t len;		   //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter;	  //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	byte *payload;
	byte checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	byte checksumB;
	bool valid; //Goes true when both checksums pass
} ubxPacket;

// Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
typedef struct
{
	byte status;	// Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
	byte numFences; // Number of geofences
	byte combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
	byte states[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
} geofenceState;

// Struct to hold the current geofence parameters
typedef struct
{
	byte numFences; // Number of active geofences
	int32_t lats[4];   // Latitudes of geofences (in degrees * 10^-7)
	int32_t longs[4];  // Longitudes of geofences (in degrees * 10^-7)
	uint32_t rads[4];  // Radii of geofences (in m * 10^-2)
} geofenceParams;

class SFE_UBLOX_GPS
{
public:
	SFE_UBLOX_GPS(void);

	//By default use the default I2C address, and use Wire port
	//bool begin(TwoWire &wirePort = Wire, byte deviceAddress = 0x42); //Returns true if module is detected
    bool begin(I2C_Handle i2cHandle, byte deviceAddress = 0x42); //Returns true if module is detected
	//serialPort needs to be perviously initialized to correct baud rate
	//bool begin(Stream &serialPort); //Returns true if module is detected

	bool isConnected(); //Returns turn if device answers on _gpsI2Caddress address

	bool checkUblox();		//Checks module with user selected commType
	bool checkUbloxI2C();	//Method for I2C polling of data, passing any new bytes to process()
	bool checkUbloxSerial(); //Method for serial polling of data, passing any new bytes to process()

	void process(byte incoming);							   //Processes NMEA and UBX binary sentences one byte at a time
	void processUBX(byte incoming, ubxPacket *incomingUBX); //Given a character, file it away into the uxb packet structure
	void processRTCMframe(byte incoming);				   //Monitor the incoming bytes for start and length bytes
	void processRTCM(byte incoming) __attribute__((weak));  //Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.

	void processUBXpacket(ubxPacket *msg);				   //Once a packet has been received and validated, identify this packet's class/id and update internal flags
	void processNMEA(char incoming) __attribute__((weak)); //Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries

	void calcChecksum(ubxPacket *msg);											   //Sets the checksumA and checksumB of a given messages
	sfe_ublox_status_e sendCommand(ubxPacket outgoingUBX, uint16_t maxWait = 250); //Given a packet and payload, send everything including CRC bytes, return true if we got a response
	sfe_ublox_status_e sendI2cCommand(ubxPacket outgoingUBX, uint16_t maxWait = 250);
	void sendSerialCommand(ubxPacket outgoingUBX);

	void printPacket(ubxPacket *packet); //Useful for debugging

	void factoryReset(); //Send factory reset sequence (i.e. load "default" configuration and perform hardReset)
	void hardReset();	//Perform a reset leading to a cold start (zero info start-up)

	bool setI2CAddress(byte deviceAddress, uint16_t maxTime = 250);							  //Changes the I2C address of the Ublox module
	void setSerialRate(uint32_t baudrate, byte uartPort = COM_PORT_UART1, uint16_t maxTime = 250); //Changes the serial baud rate of the Ublox module, uartPort should be COM_PORT_UART1/2
	//void setNMEAOutputPort(Stream &nmeaOutputPort);													  //Sets the internal variable for the port to direct NMEA characters to
	void enableNMEAOutput();

	bool setNavigationFrequency(byte navFreq, uint16_t maxWait = 250); //Set the number of nav solutions sent per second
	byte getNavigationFrequency(uint16_t maxWait = 250);					 //Get the number of nav solutions sent per second currently being output by module
	bool saveConfiguration(uint16_t maxWait = 250);						 //Save current configuration to flash and BBR (battery backed RAM)
	bool factoryDefault(uint16_t maxWait = 250);							 //Reset module to factory defaults

	sfe_ublox_status_e waitForACKResponse(byte requestedClass, byte requestedID, uint16_t maxTime = 250);   //Poll the module until a config packet and an ACK is received
	sfe_ublox_status_e waitForNoACKResponse(byte requestedClass, byte requestedID, uint16_t maxTime = 250); //Poll the module until a config packet is received

// getPVT will only return data once in each navigation cycle. By default, that is once per second.
// Therefore we should set getPVTmaxWait to slightly longer than that.
// If you change the navigation frequency to (e.g.) 4Hz using setNavigationFrequency(4)
// then you should use a shorter maxWait for getPVT. 300msec would be about right: getPVT(300)
// The same is true for getHPPOSLLH.
#define getPVTmaxWait 1100		// Default maxWait for getPVT and all functions which call it
#define getHPPOSLLHmaxWait 1100 // Default maxWait for getHPPOSLLH and all functions which call it

	bool assumeAutoPVT(bool enabled, bool implicitUpdate = true);				 //In case no config access to the GPS is possible and PVT is send cyclically already
	bool getPVT(uint16_t maxWait = getPVTmaxWait);									 //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Retruns true if new PVT is available.
	//bool setAutoPVT(bool enabled, uint16_t maxWait = 250);						 //Enable/disable automatic PVT reports at the navigation frequency
	bool setAutoPVT(bool enabled, bool implicitUpdate = true, uint16_t maxWait = 250); //Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool getHPPOSLLH(uint16_t maxWait = getHPPOSLLHmaxWait);							 //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Retruns true if new PVT is available.
	void flushPVT();																	 //Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure

	int32_t getLatitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getLongitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getAltitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current altitude in mm above ellipsoid
	int32_t getAltitudeMSL(uint16_t maxWait = getPVTmaxWait);		  //Returns the current altitude in mm above mean sea level
	byte getSIV(uint16_t maxWait = getPVTmaxWait);				  //Returns number of sats used in fix
	byte getFixType(uint16_t maxWait = getPVTmaxWait);			  //Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning
	byte getCarrierSolutionType(uint16_t maxWait = getPVTmaxWait); //Returns RTK solution: 0=no, 1=float solution, 2=fixed solution
	int32_t getGroundSpeed(uint16_t maxWait = getPVTmaxWait);		  //Returns speed in mm/s
	int32_t getHeading(uint16_t maxWait = getPVTmaxWait);			  //Returns heading in degrees * 10^-7
	uint16_t getPDOP(uint16_t maxWait = getPVTmaxWait);				  //Returns positional dillution of precision * 10^-2
	uint16_t getYear(uint16_t maxWait = getPVTmaxWait);
	byte getMonth(uint16_t maxWait = getPVTmaxWait);
	byte getDay(uint16_t maxWait = getPVTmaxWait);
	byte getHour(uint16_t maxWait = getPVTmaxWait);
	byte getMinute(uint16_t maxWait = getPVTmaxWait);
	byte getSecond(uint16_t maxWait = getPVTmaxWait);
	uint16_t getMillisecond(uint16_t maxWait = getPVTmaxWait);
	int32_t getNanosecond(uint16_t maxWait = getPVTmaxWait);
	uint32_t getTimeOfWeek(uint16_t maxWait = getPVTmaxWait);

	int32_t getHighResLatitude(uint16_t maxWait = getHPPOSLLHmaxWait);
	int32_t getHighResLongitude(uint16_t maxWait = getHPPOSLLHmaxWait);
	int32_t getElipsoid(uint16_t maxWait = getHPPOSLLHmaxWait);
	int32_t getMeanSeaLevel(uint16_t maxWait = getHPPOSLLHmaxWait);
	int32_t getGeoidSeparation(uint16_t maxWait = getHPPOSLLHmaxWait);
	uint32_t getHorizontalAccuracy(uint16_t maxWait = getHPPOSLLHmaxWait);
	uint32_t getVerticalAccuracy(uint16_t maxWait = getHPPOSLLHmaxWait);

	//Port configurations
	bool setPortOutput(byte portID, byte comSettings, uint16_t maxWait = 250); //Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setPortInput(byte portID, byte comSettings, uint16_t maxWait = 250);  //Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
	bool getPortSettings(byte portID, uint16_t maxWait = 250);					//Returns the current protocol bits in the UBX-CFG-PRT command for a given port

	bool setI2COutput(byte comSettings, uint16_t maxWait = 250);   //Configure I2C port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setUART1Output(byte comSettings, uint16_t maxWait = 250); //Configure UART1 port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setUART2Output(byte comSettings, uint16_t maxWait = 250); //Configure UART2 port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setUSBOutput(byte comSettings, uint16_t maxWait = 250);   //Configure USB port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setSPIOutput(byte comSettings, uint16_t maxWait = 250);   //Configure SPI port to output UBX, NMEA, RTCM3 or a combination thereof

	//Functions to turn on/off message types for a given port ID (see COM_PORT_I2C, etc above)
	bool configureMessage(byte msgClass, byte msgID, byte portID, byte sendRate, uint16_t maxWait = 250);
	bool enableMessage(byte msgClass, byte msgID, byte portID, byte sendRate = 1, uint16_t maxWait = 250);
	bool disableMessage(byte msgClass, byte msgID, byte portID, uint16_t maxWait = 250);
	bool enableNMEAMessage(byte msgID, byte portID, byte sendRate = 1, uint16_t maxWait = 250);
	bool disableNMEAMessage(byte msgID, byte portID, uint16_t maxWait = 250);
	bool enableRTCMmessage(byte messageNumber, byte portID, byte sendRate, uint16_t maxWait = 250); //Given a message number turns on a message ID for output over given PortID
	bool disableRTCMmessage(byte messageNumber, byte portID, uint16_t maxWait = 250);					//Turn off given RTCM message from a given port

	//General configuration (used only on protocol v27 and higher - ie, ZED-F9P)
	byte getVal8ByGroup(uint16_t group, uint16_t id, byte size, byte layer = VAL_LAYER_BBR, uint16_t maxWait = 250); //Returns the value at a given group/id/size location
	byte getVal8ByKey(uint32_t key, byte layer = VAL_LAYER_BBR, uint16_t maxWait = 250);							   //Returns the value at a given group/id/size location
	byte setVal(uint32_t keyID, uint16_t value, byte layer = VAL_LAYER_BBR, uint16_t maxWait = 250);			   //Sets the 16-bit value at a given group/id/size location
	byte setVal8(uint32_t keyID, byte value, byte layer = VAL_LAYER_BBR, uint16_t maxWait = 250);			   //Sets the 8-bit value at a given group/id/size location
	byte setVal16(uint32_t keyID, uint16_t value, byte layer = VAL_LAYER_BBR, uint16_t maxWait = 250);		   //Sets the 16-bit value at a given group/id/size location
	byte setVal32(uint32_t keyID, uint32_t value, byte layer = VAL_LAYER_BBR, uint16_t maxWait = 250);		   //Sets the 32-bit value at a given group/id/size location
	byte newCfgValset8(uint32_t keyID, byte value, byte layer = VAL_LAYER_BBR);							   //Define a new UBX-CFG-VALSET with the given KeyID and 8-bit value
	byte newCfgValset16(uint32_t keyID, uint16_t value, byte layer = VAL_LAYER_BBR);							   //Define a new UBX-CFG-VALSET with the given KeyID and 16-bit value
	byte newCfgValset32(uint32_t keyID, uint32_t value, byte layer = VAL_LAYER_BBR);							   //Define a new UBX-CFG-VALSET with the given KeyID and 32-bit value
	byte addCfgValset8(uint32_t keyID, byte value);															   //Add a new KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket
	byte addCfgValset16(uint32_t keyID, uint16_t value);															   //Add a new KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket
	byte addCfgValset32(uint32_t keyID, uint32_t value);															   //Add a new KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket
	byte sendCfgValset8(uint32_t keyID, byte value, uint16_t maxWait = 250);									   //Add the final KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
	byte sendCfgValset16(uint32_t keyID, uint16_t value, uint16_t maxWait = 250);								   //Add the final KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
	byte sendCfgValset32(uint32_t keyID, uint32_t value, uint16_t maxWait = 250);								   //Add the final KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket and send it

	//Functions used for RTK and base station setup
	bool getSurveyMode(uint16_t maxWait = 250);																   //Get the current TimeMode3 settings
	bool setSurveyMode(byte mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250); //Control survey in mode
	bool enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250);			   //Begin Survey-In for NEO-M8P
	bool disableSurveyMode(uint16_t maxWait = 250);															   //Stop Survey-In mode

	bool getSurveyStatus(uint16_t maxWait); //Reads survey in status and sets the global variables

	uint32_t getPositionAccuracy(uint16_t maxWait = 1100); //Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,

	byte getProtocolVersionHigh(uint16_t maxWait = 500); //Returns the PROTVER XX.00 from UBX-MON-VER register
	byte getProtocolVersionLow(uint16_t maxWait = 500);  //Returns the PROTVER 00.XX from UBX-MON-VER register
	bool getProtocolVersion(uint16_t maxWait = 500);		//Queries module, loads low/high bytes

	bool getRELPOSNED(uint16_t maxWait = 1100); //Get Relative Positioning Information of the NED frame

	void setDebugStream(DebugStream* debugStream);
	void enableDebugging();
	void disableDebugging();
	//void enableDebugging(Stream &debugPort = Serial);  //Given a port to print to, enable debug messages
	//void disableDebugging(void);					   //Turn off debug statements
	void debugPrint(const char *message);					   //Safely print debug statements
	void debugPrintln(const char *message);				   //Safely print debug statements
	const char *statusString(sfe_ublox_status_e stat); //Pretty print the return value

    bool issueI2cTxn(I2C_Transaction* i2cTransactionP);
    bool issueI2cTxn(I2C_Transaction* i2cTransactionP, uint16_t maxWait);
    bool issueI2cTxnTimeout(I2C_Transaction* i2cTransactionP, uint32_t timeout);

	//Support for geofences
	bool addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, byte confidence = 0, byte pinPolarity = 0, byte pin = 0, uint16_t maxWait = 1100); // Add a new geofence
	bool clearGeofences(uint16_t maxWait = 1100);																											 //Clears all geofences
	bool getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait = 1100);																		 //Returns the combined geofence state
	bool clearAntPIO(uint16_t maxWait = 1100);																												 //Clears the antenna control pin settings to release the PIOs
	geofenceParams currentGeofenceParams;																														 // Global to store the geofence parameters

	bool powerSaveMode(bool power_save = true, uint16_t maxWait = 1100);

	//Change the dynamic platform model using UBX-CFG-NAV5
	bool setDynamicModel(dynModel newDynamicModel = DYN_MODEL_PORTABLE, uint16_t maxWait = 1100);

	//Survey-in specific controls
	struct svinStructure
	{
		bool active;
		bool valid;
		uint16_t observationTime;
		float meanAccuracy;
	} svin;

	//Relative Positioning Info in NED frame specific controls
	struct frelPosInfoStructure
	{
		uint16_t refStationID;

		float relPosN;
		float relPosE;
		float relPosD;

		long relPosLength;
		long relPosHeading;

		int8_t relPosHPN;
		int8_t relPosHPE;
		int8_t relPosHPD;
		int8_t relPosHPLength;

		float accN;
		float accE;
		float accD;

		bool gnssFixOk;
		bool diffSoln;
		bool relPosValid;
		byte carrSoln;
		bool isMoving;
		bool refPosMiss;
		bool refObsMiss;
	} relPosInfo;

	//The major datums we want to globally store
	uint16_t gpsYear;
	byte gpsMonth;
	byte gpsDay;
	byte gpsHour;
	byte gpsMinute;
	byte gpsSecond;
	uint16_t gpsMillisecond;
	int32_t gpsNanosecond;

	int32_t latitude;		 //Degrees * 10^-7 (more accurate than floats)
	int32_t longitude;		 //Degrees * 10^-7 (more accurate than floats)
	int32_t altitude;		 //Number of mm above ellipsoid
	int32_t altitudeMSL;	 //Number of mm above Mean Sea Level
	byte SIV;			 //Number of satellites used in position solution
	byte fixType;		 //Tells us when we have a solution aka lock
	byte carrierSolution; //Tells us when we have an RTK float/fixed solution
	int32_t groundSpeed;	 //mm/s
	int32_t headingOfMotion; //degrees * 10^-5
	uint16_t pDOP;			 //Positional dilution of precision
	byte versionLow;		 //Loaded from getProtocolVersion().
	byte versionHigh;

	uint32_t timeOfWeek;
	int32_t highResLatitude;
	int32_t highResLongitude;
	int32_t elipsoid;
	int32_t meanSeaLevel;
	int32_t geoidSeparation;
	uint32_t horizontalAccuracy;
	uint32_t verticalAccuracy;

	uint16_t rtcmFrameCounter = 0; //Tracks the type of incoming byte inside RTCM frame

private:
	//Depending on the sentence type the processor will load characters into different arrays
	enum SentenceTypes
	{
		NONE = 0,
		NMEA,
		UBX,
		RTCM
	} currentSentence = NONE;

	//Depending on the ubx binary response class, store binary responses into different places
	enum classTypes
	{
		CLASS_NONE = 0,
		CLASS_ACK,
		CLASS_NOT_AN_ACK
	} ubxFrameClass = CLASS_NONE;

	enum commTypes
	{
		COMM_TYPE_I2C = 0,
		COMM_TYPE_SERIAL,
		COMM_TYPE_SPI
	} commType = COMM_TYPE_I2C; //Controls which port we look to for incoming bytes

	//Functions
	uint32_t extractLong(byte spotToStart); //Combine four bytes from payload into long
	uint16_t extractInt(byte spotToStart);  //Combine two bytes from payload into int
	byte extractByte(byte spotToStart);  //Get byte from payload
	void addToChecksum(byte incoming);	  //Given an incoming byte, adjust rollingChecksumA/B

	//Variables
	DebugStream* _debugSerial = NULL;  // preserve the old name for less code change
	bool _nmeaOutputEnabled = false;
	I2C_Handle _i2cHandle;
	//TwoWire *_i2cPort;				//The generic connection to user's chosen I2C hardware
	//Stream *_serialPort;			//The generic connection to user's chosen Serial hardware
	//Stream *_nmeaOutputPort = NULL; //The user can assign an output port to print NMEA sentences if they wish
	//Stream *_debugSerial;			//The stream to send debug messages to if enabled

	byte _gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series
	//This can be changed using the ublox configuration software

	bool _printDebug = false; //Flag to print the serial commands we are sending to the Serial port for debug

	//These are pointed at from within the ubxPacket
	byte payloadAck[2];
	byte payloadCfg[MAX_PAYLOAD_SIZE];

	//Init the packet structures and init them with pointers to the payloadAck and payloadCfg arrays
	ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, false};
	ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, false};

	//Limit checking of new data to every X ms
	//If we are expecting an update every X Hz then we should check every half that amount of time
	//Otherwise we may block ourselves from seeing new data
	byte i2cPollingWait = 100; //Default to 100ms. Adjusted when user calls setNavigationFrequency()

	unsigned long lastCheck = 0;
	bool autoPVT = false;			  //Whether autoPVT is enabled or not
	bool autoPVTImplicitUpdate = true; // Whether autoPVT is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
	byte commandAck = UBX_ACK_NONE;	//This goes to UBX_ACK_ACK after we send a command and it's ack'd
	uint16_t ubxFrameCounter;			  //It counts all UBX frame. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]

	byte rollingChecksumA; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
	byte rollingChecksumB; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

	//Create bit field for staleness of each datum in PVT we want to monitor
	//moduleQueried.latitude goes true each time we call getPVT()
	//This reduces the number of times we have to call getPVT as this can take up to ~1s per read
	//depending on update rate
	struct
	{
		uint32_t gpsiTOW : 1;
		uint32_t gpsYear : 1;
		uint32_t gpsMonth : 1;
		uint32_t gpsDay : 1;
		uint32_t gpsHour : 1;
		uint32_t gpsMinute : 1;
		uint32_t gpsSecond : 1;
		uint32_t gpsNanosecond : 1;

		uint32_t all : 1;
		uint32_t longitude : 1;
		uint32_t latitude : 1;
		uint32_t altitude : 1;
		uint32_t altitudeMSL : 1;
		uint32_t SIV : 1;
		uint32_t fixType : 1;
		uint32_t carrierSolution : 1;
		uint32_t groundSpeed : 1;
		uint32_t headingOfMotion : 1;
		uint32_t pDOP : 1;
		uint32_t versionNumber : 1;
	} moduleQueried;

	struct
	{
		uint16_t all : 1;
		uint16_t timeOfWeek : 1;
		uint16_t highResLatitude : 1;
		uint16_t highResLongitude : 1;
		uint16_t elipsoid : 1;
		uint16_t meanSeaLevel : 1;
		uint16_t geoidSeparation : 1;
		uint16_t horizontalAccuracy : 1;
		uint16_t verticalAccuracy : 1;
	} highResModuleQueried;

	uint16_t rtcmLen = 0;
};

#endif // SPARKFUN_UBLOX_ARDUINO_LIBRARY_HPP
