/*.*******************************************************************
  FILENAME: XCOMdat.h
**********************************************************************
* 	PROJECT:        iXCOM_SDK
*
*
*---------------------------------------------------------------------
* 	Copyright 2021, iMAR GmbH
*---------------------------------------------------------------------
* 	MODULE DESCRIPTION:
*	iXCOM protocol version 2.0
*
*
---------------------------------------------------------------------*/
#ifndef XCOMDAT_H_
#define XCOMDAT_H_
// clang-format off
// NOLINTBEGIN
#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif
#ifdef __cplusplus
#define XCOM_ENUM_EXT class
#else
#define XCOM_ENUM_EXT
#endif
#if !defined XCOM_DISABLE_STRUCT_PACKING
#if defined __clang__ || defined __GNUC__ || defined __MINGW32__
#define XCOM_STRUCT_PACK __attribute__((__packed__))
#elif defined _MSC_VER
#define XCOM_STRUCT_PACK
#pragma pack(push)
#pragma pack(1)
#else
#error This compiler is currently not supported.
#endif
#endif
/**
 * *****************************************************************************************************************************************
 * General
 * *****************************************************************************************************************************************
 */
#define XCOM_MAX_CHANNELS                  32   /**< Number of independent iXCOM channels */
#define XCOM_LOGS_PER_CHANNEL              16   /**< Number of logs per iXCOM channels */
#define XCOM_MAX_MESSAGE_LENGTH            4096 /**< Max. length of iXCOM frames */
#define XCOM_MAX_NUMBER_OF_TEMPERATURES    16   /**< Max. number of supported system temperatures */
#define XCOM_MAX_NUMBER_OF_ARINC825_FRAMES 32   /**< Max. number of supported ARINC825 frames */
#define XCOM_MAX_CLIENT_SUPPORT            8    /**< Max. number of supported clients for MVT feature */
#define XCOM_MAX_CLIENT_LOGS               8    /**< Max. number of supported logs per client (MVT) */
#define XCOM_MAX_SERIAL_PORTS              6    /**< Max. number of serial ports */
#define XCOM_INTERFACE_LAN                 0    /**< Needed for PARXCOM_NETCONFIG */
#define XCOM_INTERFACE_USB                 1    /**< Needed for PARXCOM_NETCONFIG */
#define XCOM_SYNC_BYTE                     0x7E /**< Synchronization character */
#define XCOM_MAX_MSG_PAYLOAD_LENGTH        (XCOM_MAX_MESSAGE_LENGTH - sizeof(XCOMHeader) - sizeof(XCOMFooter))
#define XCOM_MAX_PAR_PAYLOAD_LENGTH        (XCOM_MAX_MESSAGE_LENGTH - sizeof(XCOMHeader) - sizeof(XCOMFooter) - sizeof(XCOMParHeader))
#define XCOMPAR_PLUGIN_MAX_NUM    2
/**
 * *****************************************************************************************************************************************
 * iXCOM Message IDs
 * *****************************************************************************************************************************************
 * XComMessageID shows the available iXCOM data IDs and the description of the message content. It should be noted that not all IDs are
 * available on all iMAR systems. This is dependent on the configuration and on the components being used (e.g. iMADC, iMAG)
 * *****************************************************************************************************************************************
 */
enum XComMessageID {
    XCOM_MSGID_IMURAW               = 0x00,    /**< IMU data - calibrated */
    XCOM_MSGID_IMU2RAW              = 0x7D,    /**< Secondary IMU data - calibrated */
    XCOM_MSGID_IMUCORR              = 0x01,    /**< IMU data - calibrated and corrected for errors estimated by EKF based data fusion */
    XCOM_MSGID_IMUCOMP              = 0x02,    /**< IMU data - calibrated and corrected for estimated errors, compensated for earth rate and gravity */
    XCOM_MSGID_IMUCAL               = 0x31,    /**< IMU special data, used for calibration */
    XCOM_MSGID_RAWDATA              = 0x67,    /**< Raw data, useful for post-processing */
    XCOM_MSGID_OMGINT               = 0x35,    /**< Integrated angular rate */
    XCOM_MSGID_OMGDOT               = 0x50,    /**< Angular acceleration around body axes */
    XCOM_MSGID_LOADFACTOR           = 0x49,    /**< Load factor */
    XCOM_MSGID_TEMPERATURE          = 0x22,    /**< IMU dependent system temperatures (CPU, PCB and sensors) */
    XCOM_MSGID_POWER                = 0x21,    /**< Internal voltage measurements (IMU, FPGA, ISO, VMS, ...) */
    XCOM_MSGID_TIME                 = 0x26,    /**< Time related data, regarding system, PPS, IMU and GNSS */
    XCOM_MSGID_EVENTTIME            = 0x34,    /**< Time measurement of the external event inputs */
    XCOM_MSGID_IMUFILTERED          = 0x56,    /**< IMU data with butterworth filtering */
    XCOM_MSGID_INSSOL               = 0x03,    /**< Integrated navigation solution including IMU data in a selectable frame */
    XCOM_MSGID_INSSOLECEF           = 0x47,    /**< Integrated navigation solution including IMU data in ECEF frame */
    XCOM_MSGID_INSRPY               = 0x04,    /**< Integrated attitude solution (roll, pitch, yaw) */
    XCOM_MSGID_INSDCM               = 0x05,    /**< Integrated attitude solution (direction cosine matrix) */
    XCOM_MSGID_INSQUAT              = 0x06,    /**< Integrated attitude solution (quaternion) */
    XCOM_MSGID_INSVELNED            = 0x07,    /**< Integrated velocity solution in NED frame */
    XCOM_MSGID_INSVELENU            = 0x23,    /**< Integrated velocity solution in ENU frame */
    XCOM_MSGID_INSVELECEF           = 0x08,    /**< Integrated velocity solution in ECEF frame */
    XCOM_MSGID_INSVELBODY           = 0x09,    /**< Integrated velocity solution in body frame */
    XCOM_MSGID_INSPOSLLH            = 0x0A,    /**< INS/GNSS position in LLH (WGS84) */
    XCOM_MSGID_INSPOSECEF           = 0x0B,    /**< INS/GNSS position in ECEF frame */
    XCOM_MSGID_INSPOSUTM            = 0x0C,    /**< INS/GNSS position in UTM frame */
    XCOM_MSGID_INSROTTEST           = 0x0D,    /**< INS rotation test */
    XCOM_MSGID_INSGNDSPEED          = 0x13,    /**< INS speed over ground, course over ground and down velocity */
    XCOM_MSGID_INSTRACKACC          = 0x0E,    /**< INS along-track, cross-track and vertical acceleration */
    XCOM_MSGID_INSMAGHDG            = 0x1A,    /**< INS true heading converted to magnetic heading */
    XCOM_MSGID_INSMGRS              = 0x46,    /**< INS/GNSS position solution in Military Grid Reference System */
    XCOM_MSGID_INSRPYRATE           = 0xA0,    /**< INS/GNSS roll-, pitch- and yaw-rate */
    XCOM_MSGID_ACCNED               = 0xA1,    /**< INS/GNSS acceleration in NED frame */
    XCOM_MSGID_UNIXTIME             = 0xA2,    /**< Unix time of the current inertial epoch */
    XCOM_MSGID_INSWANDERAZIMUTH     = 0xA3,    /**< INS/GNSS wander azimuth */
    XCOM_MSGID_JERKNED              = 0xA4,    /**< INS/GNSS jerk in NED frame */
    XCOM_MSGID_INSRPYACCEL          = 0xA5,    /**< INS/GNSS roll-, pitch- and yaw-acceleration */
    XCOM_MSGID_EKFSTDDEV            = 0x0F,    /**< EKF estimated standard deviations (@deprecated; use EKFSTDDEV3 for new developments) */
    XCOM_MSGID_EKFSTDDEV2           = 0x28,    /**< EKF estimated standard deviations (@deprecated; use EKFSTDDEV3 for new developments) */
    XCOM_MSGID_EKFSTDDEV3           = 0x9A,    /**< EKF estimated standard deviations (including body velocity) */
    XCOM_MSGID_EKFERROR             = 0x10,    /**< EKF estimated sensor errors (@deprecated; use EKFERROR2 for new developments) */
    XCOM_MSGID_EKFERROR2            = 0x27,    /**< EKF estimated sensor uncertainties */
    XCOM_MSGID_EKFPOSCOVAR          = 0x29,    /**< EKF position covariance matrix */
    XCOM_MSGID_EKFTIGHTLY           = 0x11,    /**< EKF used satellite information in tightly coupled mode */
    XCOM_MSGID_EKFSTDDEVECEF        = 0x48,    /**< EKF estimated standard deviations in ECEF frame */
    XCOM_MSGID_GNSSSOL              = 0x12,    /**< GNSS navigation solution. This package contains GNSS position, velocity, number of satellites, PDOP and  receiver status. */
    XCOM_MSGID_GNSSTIME             = 0x14,    /**< GNSS time stamp (UTC) */
    XCOM_MSGID_GNSSSOLCUST          = 0x15,    /**< GNSS custom solution */
    XCOM_MSGID_GNSSHDG              = 0x33,    /**< GNSS attitude information (only available in dual-antenna systems) */
    XCOM_MSGID_GNSSLEVERARM         = 0x1B,    /**< GNSS lever arm estimates */
    XCOM_MSGID_GNSSVOTER            = 0x1C,    /**< GNSS voter information */
    XCOM_MSGID_GNSSALIGNBSL         = 0x38,    /**< GNSS RTK quality ENU baselines */
    XCOM_MSGID_GNSSSATINF           = 0x51,    /**< GNSS satellite status information */
    XCOM_MSGID_GNSSSTREAM           = 0x58,    /**< GNSS raw data stream in proprietary binary protocol (depends on internal receiver type) */
    XCOM_MSGID_GNSSOBSSTREAM        = 0x59,    /**< GNSS correction data in proprietary binary protocol (depends on the selected data format) */
    XCOM_MSGID_GNSSDOP              = 0x60,    /**< DOP values for the satellites used in the GNSS solution */
    XCOM_MSGID_GNSSHWMON            = 0x1E,    /**< GNSS hardware monitor (voltages and temperatures of the integrated GNSS receiver) */
    XCOM_MSGID_PORTSTATS            = 0x43,    /**< COM port statistics of the integrated GNSS receiver */
    XCOM_MSGID_GNSSPOSECEF          = 0x61,    /**< GNSS position solution in ECEF coordinates */
    XCOM_MSGID_GNSSVELECEF          = 0x62,    /**< GNSS velocity solution in ECEF coordinates */
    XCOM_MSGID_PASSTHROUGH          = 0x63,    /**< Passthrough data message */
    XCOM_MSGID_PLUGINDATA           = 0x64,    /**< XCOM message ID that is used by the plugins to communicate with the GUI */
    XCOM_MSGID_MAGDATA2             = 0x65,    /**< Magnetometer raw data (magnetic fields) */
    XCOM_MSGID_IPST                 = 0x66,    /**< Inertial measurement systems for pipeline  surveying and drilling applications */
    XCOM_MSGID_GREENLIGHTLOCAL      = 0x70,    /**< Greenlight specific (OEBB) message */
    XCOM_MSGID_GREENLIGHTSMARTPHONE = 0x71,    /**< Greenlight specific (OEBB) message */
    XCOM_MSGID_ENCODERDATA          = 0x80,    /**< Absolute encoder position + velocity + status information */
    XCOM_MSGID_CPULOAD              = 0x81,    /**< CPU load and status information */
    XCOM_MSGID_HEAVE                = 0x1F,    /**< Heave information (@deprecated) */
    XCOM_MSGID_WHEELDATA            = 0x16,    /**< (Single-) Odometer measurements */
    XCOM_MSGID_AIRDATA              = 0x17,    /**< Air data computer measurements */
    XCOM_MSGID_MAGDATA              = 0x18,    /**< Magnetometer measurements */
    XCOM_MSGID_ADC24DATA            = 0x37,    /**< ADC measurements of the optional ADC24 board */
    XCOM_MSGID_CSACDATA             = 0x42,    /**< Status information of the optional chip scale atomic clock */
    XCOM_MSGID_SYSSTAT              = 0x19,    /**< Extended system status */
    XCOM_MSGID_CANSTATUS            = 0x24,    /**< @deprecated; CAN controller status information */
    XCOM_MSGID_STATFPGA             = 0x20,    /**< FPGA status information */
    XCOM_MSGID_ADC24STATUS          = 0x36,    /**< Status information of the optional ADC24 board */
    XCOM_MSGID_ARINC429STAT         = 0x1D,    /**< ARINC429 status information */
    XCOM_MSGID_MONITOR              = 0x57,    /**< Monitor messages (for debugging purpose only) */
    XCOM_MSGID_MVCSLAVE             = 0x39,    /**< Slave measurements (multi-vehicle communication module) */
    XCOM_MSGID_MVCDATA              = 0x41,    /**< Master measurements (multi-vehicle communication module) */
    XCOM_MSGID_POSTPROC             = 0x40,    /**< Log containing online navigation solution as well as data needed for postprocessing */
    XCOM_MSGID_NTRIPSTATUS          = 0x45,    /**< Log containing status information of the internal Ntrip client */
    XCOM_MSGID_NMEA0183_GGA         = 0x72,    /**< NMEA-0183: Time, position, and fix related data */
    XCOM_MSGID_NMEA0183_GLL         = 0x73,    /**< NMEA-0183: Geographic Position - Latitude/Longitude */
    XCOM_MSGID_NMEA0183_GSA         = 0x74,    /**< NMEA-0183: GNSS DOP and active satellites */
    XCOM_MSGID_NMEA0183_HDT         = 0x75,    /**< NMEA-0183: True heading */
    XCOM_MSGID_NMEA0183_RMC         = 0x76,    /**< NMEA-0183: Recommended Minimum Navigation Message */
    XCOM_MSGID_NMEA0183_VTG         = 0x77,    /**< NMEA-0183: Track Made Good and Ground Speed Message */
    XCOM_MSGID_NMEA0183_ZDA         = 0x78,    /**< NMEA-0183: Time and Date - UTC, Day, Month, Year and Local Time Zone Message */
    XCOM_MSGID_NMEA0183_GST         = 0x79,    /**< NMEA-0183: GNSS Pseudorange Error Statistics Message */
    XCOM_MSGID_NMEA0183_PIAHS       = 0x7A,    /**< NMEA-0183: Attitude, Heading, Heave and Standard Deviation Message (iMAR Proprietary) */
    XCOM_MSGID_NMEA0183_PISTATUS1   = 0x7B,    /**< NMEA-0183: Status Information Message (iMAR Proprietary) */
    XCOM_MSGID_NMEA0183_PISTATUS2   = 0x7F,    /**< NMEA-0183: Extended Status Information Message (iMAR Proprietary) */
    XCOM_MSGID_NMEA0183_PAPBN       = 0x7C,    /**< NMEA-0183: ECEF positions and velocities (AIRBUS Proprietary) */
    XCOM_MSGID_NMEA0183_PIARTS1     = 0x83,    /**< NMEA-0183: Acceleration, rate, time status (iMAR Proprietary) */
    XCOM_MSGID_NMEA0183_PASHR       = 0x84,    /**< NMEA-0183: Inertial Attitude Data (Proprietary) */
    XCOM_MSGID_ABDPROTOCOL          = 0x90,    /**< ABD protocol */
    XCOM_MSGID_CANGATEWAY           = 0x91,    /**< CAN/XCOM gateway message */
    XCOM_MSGID_CANSTATUS2           = 0x92,    /**< CAN status message */
    XCOM_MSGID_EXTPOSLLH            = 0x93,    /**< External position (llh). */
    XCOM_MSGID_EXTVELNED            = 0x94,    /**< External velocity (ned). */
    XCOM_MSGID_EXTVELBODY           = 0x95,    /**< External velocity (body frame). */
    XCOM_MSGID_EXTSTREAM            = 0x97,    /**< External sensor raw data stream */
    XCOM_MSGID_EXTHEIGHT            = 0x98,    /**< External height */
    XCOM_MSGID_EXTHEADING           = 0x9B,    /**< External heading */
    XCOM_MSGID_EXTMAGNETOMETER      = 0x9C,    /**< External megnetometer */
    XCOM_MSGID_SYNCCOUNTER          = 0x99,    /**< External synchronization counter value and timestamp */
    XCOM_MSGID_ODOLEVERARM          = 0x96,    /**< Odometer lever arm estimates */
    XCOM_MSGID_FPGARAW              = 0x32,    /**< FPGA raw data stream */
    XCOM_MSGID_BATSTAT              = 0x44,    /**< Battery status message */
    XCOM_MSGID_BATSTAT2             = 0x82,    /**< Battery status message */
    XCOM_MSGID_SCUECEF              = 0xF6,    /**< Integrated navigation solution and status information for iSCU */
    XCOM_MSGID_ADSE                 = 0xFC,    /**< Integrated navigation solution and status information for ADSE framework */
    XCOM_MSGID_LEONARDO             = 0x52,    /**< Integrated navigation solution and status information for Leonardo */
    XCOM_MSGID_CHARM                = 0x53,    /**< Integrated navigation solution and status information for Charm */
    XCOM_MSGID_SCU                  = 0x54,    /**< Integrated navigation solution and status information for SCU */
    XCOM_MSGID_ERGRADDON            = 0x55,    /**< GNSS data from ERGR */
    XCOM_MSGID_EKFSTATEXT           = 0x85,    /**< Extended Kalman Filter status */
    XCOM_MSGID_STANDSTILL           = 0xD1,    /**< Values that are checked in standstill detector, useful for tuning */
    XCOM_MSGID_AHRS                 = 0xD3,    /**< AHRS module output */
    XCOM_MSGID_IPVAS                = 0xD2,    /**< Inertial data, position, velocity and status information */
    XCOM_MSGID_POSAVE               = 0xD4,    /**< Position averaging result */
    XCOM_MSGID_TRANSFERALIGN        = 0x9D,    /**< Transfer alignment message */
    XCOM_MSGID_COMMAND              = 0xFD,    /**< iXCOM command ID */
    XCOM_MSGID_RESPONSE             = 0xFE,    /**< iXCOM response ID */
    XCOM_MSGID_PARAMETER            = 0xFF     /**< iXCOM parameter ID */
};
/**
 * *****************************************************************************************************************************************
 * iXCOM COMMAND IDs:
 * *****************************************************************************************************************************************
 * The iXCOM protocol allows sending commands via the command message ID 0xFD (XCOM_MSGID_COMMAND), The command message payload consists of
 * a header selecting a specific command and a variable payload.
 * *****************************************************************************************************************************************
 */
enum XCOMcmdCategory {
    XCOM_CMDID_LOG          = 0x0000,   /**< The LOG command enables/disables the output of several packages. */
    XCOM_CMDID_EXT          = 0x0001,   /**< @deprecated; will be removed in the near future */
    XCOM_CMDID_CAL          = 0x0002,   /**< @deprecated; will be removed in the near future */
    XCOM_CMDID_CONF         = 0x0003,   /**< This set of commands saves and loads a configuration to/from internal storage */
    XCOM_CMDID_EKF          = 0x0004,   /**< Set of commands to save current position/heading and to start and alignment */
    XCOM_CMDID_XCOM         = 0x0005,   /**< Set of commands to control the behavior of the XCOM protocol module */
    XCOM_CMDID_FPGA         = 0x0006,   /**< Set of commands to control the FPGA module */
    XCOM_CMDID_EXTAID       = 0x0007,   /**< Set of commands to control the external aiding sources */
    XCOM_CMDID_PATH_CTRL    = 0x0008,   /**< Set of commands to control the path following module */
    XCOM_CMDID_USR_IF       = 0x0009,   /**< Set of commands to control application specific modules */
    XCOM_CMDID_DUMP         = 0x000A,   /**< Set of commands to control the behavior of the  dumper module */
    XCOM_CMDID_SCU          = 0x000B,   /**< Set of commands to control the behavior of the  SCU */
    XCOM_CMDID_ALARM        = 0x1000,   /**< Project specific command ID */
    XCOM_CMDID_ACK          = 0x1001,   /**< Project specific command ID */
    XCOM_CMDID_SIGSTATE     = 0x1002,   /**< Project specific command ID */
    XCOM_CMDID_LOGIN        = 0x1003,   /**< Project specific command ID */
    XCOM_CMDID_MAX          = XCOM_CMDID_SIGSTATE
};
/**
 * iXCOM XCOM command:
 * *****************************************************************************************************************************************
 * The iXCOM commands must be used to initialize/open or close the iXCOM data interface. The iXCOM command needs as parameter the
 * corresponding channel number.
 * *****************************************************************************************************************************************
 */
enum XCOMcmd_XCom {
    XCOM_CMDXCOM_CLOSE              = 0x0000,   /**< Close an iXCOM channel */
    XCOM_CMDXCOM_OPEN               = 0x0001,   /**< Open an iXCOM channel */
    XCOM_CMDXCOM_COLDRESET          = 0x0002,   /**< Performing a cold reset (complete removal of power and restart) */
    XCOM_CMDXCOM_WARMRESET          = 0x0003,   /**< Performing a warm reset (forcing a restart via a reboot with out powering down) */
    XCOM_CMDXCOM_RESETOMGINT        = 0x0004,   /**< Reset OMG integration */
    XCOM_CMDXCOM_UPDATESVN          = 0x0005,   /**< Load baseline via SVN */
    XCOM_CMDXCOM_RESETTIMEBIAS      = 0x0006,   /**< Reset estimated time bias */
    XCOM_CMDXCOM_SKIPLOGFILE        = 0x0007,   /**< Skip log-file (close the current log-file and open a new one) */
    XCOM_CMDXCOM_TRACELOG           = 0x0008,   /**< Start trace logger (for debugging purpose only) */
    XCOM_CMDXCOM_BACKUPAPP          = 0x0009,   /**< Creating a backup image of the active application partition */
    XCOM_CMDXCOM_BACKUPCALIB        = 0x000A,   /**< Creating a backup image of the calibration partition */
    XCOM_CMDXCOM_BACKUPROOT         = 0x000B,   /**< Creating a backup image of the active root partition */
    XCOM_CMDXCOM_BACKUPIFS          = 0x000C,   /**< Creating a backup image of the active ifs partition */
    XCOM_CMDXCOM_BACKUPSYSTEM       = 0x000D,   /**< Creating a eMMC backup image */
    XCOM_CMDXCOM_UPDATEAPP          = 0x000E,   /**< Updating the application partition (the image must be stored as </data/upload/appfs_qnx6fs.imar>) */
    XCOM_CMDXCOM_UPDATECALIB        = 0x000F,   /**< Updating the calibration partition (the image must be stored as </data/upload/calibfs_qnx6fs.imar>) */
    XCOM_CMDXCOM_UPDATEROOT         = 0x0010,   /**< Updating the root partition (the image must be stored as </data/upload/rootfs_qnx6fs.imar>) */
    XCOM_CMDXCOM_UPDATEIFS          = 0x0011,   /**< Updating the ifs partition (the image must be stored as </data/upload/ifs_fat.imar>) */
    XCOM_CMDXCOM_FWFACTORYBACKUP    = 0x0012,   /**< Creating a factory backup of iNAT firmware executable */
    XCOM_CMDXCOM_FORMAT             = 0x0013,   /**< Format recorder partition and create directory structure */
    XCOM_CMDXCOM_EJECTDEVICE        = 0x0014,   /**< Eject recording device (USB) */
    XCOM_CMDXCOM_SETTIME            = 0x0015,   /**< Set system time */
    XCOM_CMDXCOM_GYROCALIB          = 0x0016,   /**< Start gyro bias calibration */
    XCOM_CMDXCOM_LOGLISTPRESET      = 0x0017,   /**< This command configures one XCOM channel with a predefined loglist */
    XCOM_CMDXCOM_DOWNLOADROOTFS     = 0x0018,   /**< This command loads the latest rootfs from version control system */
    XCOM_CMDXCOM_DOWNLOADFIRMWARE   = 0x0019,   /**< This command loads the latest firmware executable from version control system */
    XCOM_CMDXCOM_UPDATEFIRMWARE     = 0x001A,   /**< This command updates the current firmware executable */
    XCOM_CMDXCOM_SAMPLEMAG          = 0x001B,   /**< Feed the next magnetometer sample into the magnetometer calibration module  */
    XCOM_CMDXCOM_MAX                = XCOM_CMDXCOM_SAMPLEMAG
};
/**
 * iXCOM LOG command:
 * *****************************************************************************************************************************************
 * The LOG command manipulates the log list of the currently active channel. Besides a specific message ID, a Trigger, a Command Parameter
 * and a Divider value are expected.
 * *****************************************************************************************************************************************
 */
enum XComLogCmdParam {
    XCOM_CMDLOG_ADD         = 0x0000,   /**< This parameter adds a log to the log-list of the IMS on the currently used Channel Number. */
    XCOM_CMDLOG_STOP        = 0x0001,   /**< Stop data output of a specific message. The selected log is still contained in the log list of
                                             the IMS,  but output will be interrupted. */
    XCOM_CMDLOG_START       = 0x0002,   /**< Start data output of a previously stopped log. */
    XCOM_CMDLOG_CLEAR       = 0x0003,   /**< Delete a specific log from the log-list. */
    XCOM_CMDLOG_CLEARALL    = 0x0004,   /**< This command clears the log-list of the current channel. */
    XCOM_CMDLOG_STOPALL     = 0x0005,   /**< Stop data output on this channel. The configured data logs are still contained in the log-list,
                                             but  output will be interrupted. */
    XCOM_CMDLOG_STARTALL    = 0x0006,   /**< Start data output of all configured data logs of the current log list. */
    XCOM_CMDLOG_MAX         = XCOM_CMDLOG_STARTALL
};
/**
 * iXCOM LOG command trigger type:
 * *****************************************************************************************************************************************
 * To every log, a trigger to generate a message has to be chosen out of the following possibilities.
 * *****************************************************************************************************************************************
 */
enum XComLogTrigger {
    XCOM_CMDLOG_TRIG_SYNC       = 0x00, /**< The log is generated synchronously with the IMU sample clock. */
    XCOM_CMDLOG_TRIG_EVENT      = 0x01, /**< The message will be generated and sent immediately (asynchronously to
                                             the  IMU sample clock) as soon as new data are available (related to the requested log). */
    XCOM_CMDLOG_TRIG_POLLED     = 0x02, /**< This message log will be generated once immediately after a request is received. */
    XCOM_CMDLOG_TRIG_EXTEVENT   = 0x03, /**< Log generation is triggered via input by an external event. */
    XCOM_CMDLOG_TRIG_PPTEVENT   = 0x04, /**< Log generation is triggered via PPT event. */
    XCOM_CMDLOG_TRIG_AVG        = 0x05, /**< The log is averaged over a specific time period */
    XCOM_CMDLOG_TRIG_PPS        = 0x06, /**< Log generation is triggered via PPS event. */
    XCOM_CMDLOG_TRIG_MAX        = XCOM_CMDLOG_TRIG_PPS
};
/**
 * iXCOM CONF command:
 * *****************************************************************************************************************************************
 * The CONF command can be used to store or load the system configuration to or from internal storage.
 * *****************************************************************************************************************************************
 */
enum XCOMcmd_Conf {
    XCOM_CMDCONF_SAVE           = 0x0000, /**< Save the current configuration to the internal storage of the system. This configuration will be
                                               loaded during the next system boot. */
    XCOM_CMDCONF_LOAD           = 0x0001, /**< Load the stored configuration into RAM. */
    XCOM_CMDCONF_DELIVERYLOAD   = 0x0002, /**< The current configuration will be deleted and the delivery setting will be restored after an
                                               automatic reboot */
    XCOM_CMDCONF_DELIVERYSAVE   = 0x0003, /**< Saves current configuration files and stores them delivery configuration */
    XCOM_CMDCONF_FACTORYLOAD    = 0x0004, /**< The current configuration will be deleted and the factory setting will be restored after an
                                               automatic  reboot */
    XCOM_CMDCONF_MAX            = XCOM_CMDCONF_FACTORYLOAD
};
/**
 * iXCOM EKF command:
 * *****************************************************************************************************************************************
 * The EKF command can be used to control the behavior of the Extended Kalman Filter
 * *****************************************************************************************************************************************
 */
enum XCOMcmd_Ekf {
    XCOM_CMDEKF_STARTALIGN              = 0x0000,   /**< Starting an alignment with the existing alignment parameters */
    XCOM_CMDEKF_STOREPOS                = 0x0001,   /**< The INS will store the current position to the internal storage. The stored position will be
                                                         loaded when the STOREDPOS mode is selected in the PAREKF_STARTUPV2 parameter. */
    XCOM_CMDEKF_STOREHDG                = 0x0002,   /**< The INS will store the current heading to the internal storage. The stored heading will be
                                                         loaded when the STOREDHDG mode is selected in the PAREKF_STARTUPV2 parameter. */
    XCOM_CMDEKF_STOREANTOFFSET          = 0x0003,   /**< Store the estimated antenna offset to the configuration loaded in RAM. To make the estimated
                                                         antenna offset persist after a reboot, the CONF Command has to be issued with the SAVE parameter. */
    XCOM_CMDEKF_FORCEDZUPT              = 0x0004,   /**< @deprecated */
    XCOM_CMDEKF_ALIGNCOMPLETE           = 0x0005,   /**< If this command is received the INS will complete the static alignment. Motion is allowed
                                                         after issuing this command */
    XCOM_CMDEKF_COPYODOSCF              = 0x0006,   /**< Save estimated odometer scale factor to the configuration loaded in RAM. To make this setting
                                                         persist after a reboot, the CONF Command has to be issued with the SAVE parameter. */
    XCOM_CMDEKF_LOADPARFROMFILE         = 0x0007,   /**< Load EKF specific default parameter from read-only configuration file */
    XCOM_CMDEKF_STARTZUPTCALIBRATION    = 0x0008,   /**< If the EKF is initialized successfully, start Zero Velocity Update calibration */
    XCOM_CMDEKF_STOREMAGSTATE           = 0x0009,   /**< If the stationary alignment has completed successfully, start the iMAG calibration. */
    XCOM_CMDEKF_DELETEMAGSTATE          = 0x000A,   /**< Remove iMAG calibration files from system. */
    XCOM_CMDEKF_FORCEDSINGLEZUPT        = 0x000B,   /**< Performs a single zero velocity update */
    XCOM_CMDEKF_ABORTVERIFIEDALIGN      = 0x000C,   /**< Abort verified stored heading alignment */
    XCOM_CMDEKF_STORESTATE              = 0x000D,   /**< The INS will store the current state to the internal storage. */
    XCOM_CMDEKF_MAX                     = XCOM_CMDEKF_STORESTATE
};
/**
 * iXCOM FPGA command:
 * *****************************************************************************************************************************************
 * This command is not intended to be used by the common operator! Wrong usage may damage or destroy the system
 *
 * NOTE:
 * Before usage contact iMAR design engineer team for approval (support@imar-navigation.de).
 * *****************************************************************************************************************************************
 */
enum XCOMcmd_Fpga {
    XCOM_CMDFPGA_IMUPOWER           = 0x0000,   /**< @deprecated: Use XCOMPAR_PARFPGA_POWER instead */
    XCOM_CMDFPGA_GNSSPOWER          = 0x0001,   /**< @deprecated: Use XCOMPAR_PARFPGA_POWER instead */
    XCOM_CMDFPGA_ISOPOWER           = 0x0002,   /**< @deprecated: Use XCOMPAR_PARFPGA_POWER instead */
    XCOM_CMDFPGA_VMSPOWER           = 0x0003,   /**< @deprecated: Use XCOMPAR_PARFPGA_POWER instead */
    XCOM_CMDFPGA_SHUTDOWN           = 0x0004,   /**< Leaving operational mode and entering standby mode. If this command is received, all internal
                                                     components will powered down. */
    XCOM_CMDFPGA_POWERUP            = 0x0005,   /**< Leaving standby mode and entering operational mode. If this command is received, the system will
                                                     perform an initial alignment and starts navigation. All internal components will be powered up. */
    XCOM_CMDFPGA_FLUSHFRAM          = 0x0006,   /**< If the command is received, the system will flush data to the FRAM if possible. */
    XCOM_CMDFPGA_RESETSYNCCOUNTER   = 0x0007,   /**< Reset external synchronization counter value. */
    XCOM_CMDFPGA_MAX                = XCOM_CMDFPGA_RESETSYNCCOUNTER
};
/**
 * iXCOM EXTAID command:
 * *****************************************************************************************************************************************
 * The EXTAID command controls the external aiding sources inside the INS. This command can be used to send measurement updates directly to
 * the EKF module. In addition to that forced aiding commands are also supported. The forced aiding will be applied inside the filter until
 * the aiding switch will be set to 0
 * *****************************************************************************************************************************************
 */
enum XCOMcmd_Extaid {
    XCOM_CMDEXTAID_FREEZE_ALT   = 0x0000,   /**< Freeze altitude output */
    XCOM_CMDEXTAID_FREEZE_HDG   = 0x0001,   /**< Freeze heading output */
    XCOM_CMDEXTAID_FREEZE_VEL   = 0x0002,   /**< Freeze velocity ourout */
    XCOM_CMDEXTAID_POS_LLH      = 0x0003,   /**< External position aiding in LLH frame */
    XCOM_CMDEXTAID_VEL_NED      = 0x0004,   /**< External velocity aiding in NED frame. */
    XCOM_CMDEXTAID_HDG          = 0x0005,   /**< External heading aiding. */
    XCOM_CMDEXTAID_HGT          = 0x0006,   /**< External height aiding.*/
    XCOM_CMDEXTAID_VEL_BODY     = 0x0007,   /**< External velocity aiding in INS body frame. */
    XCOM_CMDEXTAID_BAROALT      = 0x0008,   /**< External barometric altitude aiding. */
    XCOM_CMDEXTAID_VEL_ECEF     = 0x0009,   /**< External velocity aiding in ECEF frame */
    XCOM_CMDEXTAID_POS_ECEF     = 0x000A,   /**< External position aiding in ECEF frame */
    XCOM_CMDEXTAID_POS_UTM      = 0x000B,   /**< External position aiding in UTM frame */
    XCOM_CMDEXTAID_POS_MGRS     = 0x000C,   /**< External position aiding in MGRS frame */
    XCOM_CMDEXTAID_VEL_ECEF2    = 0x000D,   /**< External velocity aiding in ECEF frame */
    XCOM_CMDEXTAID_VEL_NED2     = 0x000E,   /**< External velocity aiding in NED frame. */
    XCOM_CMDEXTAID_HGT2         = 0x000F,   /**< External height aiding.*/
    XCOM_CMDEXTAID_MAGFIELD     = 0x0010,   /**< External magnetic field aiding.*/
    XCOM_CMDEXTAID_MAX          = XCOM_CMDEXTAID_MAGFIELD
};
/**
 * XCOM Parameter
 * *****************************************************************************************************************************************
 * The iXCOM protocol provides the ability to set or request all system parameters separately. The parameters can be set via the binary
 * interface. The binary interface communicates via the parameter channel ID 0xFF ('XCOM_MSGID_PARAMETER').
 * *****************************************************************************************************************************************
 */
enum XCOMpar_action {
    XCOM_PAR_SET = 0x00, /**< Changing a XCOM parameter */
    XCOM_PAR_GET = 0x01  /**< Reading a XCOM parameter */
};
enum XComParameterID {
    /**
     * *************************************************************************************************************************************
     * SYS Parameter
     * *************************************************************************************************************************************
     */
    XCOMPAR_PARSYS_PRJNUM           = 0,    /**< This parameter is read-only and will be factory set during INS production. It contains the
                                                 iMAR project number under which the system has been manufactured. */
    XCOMPAR_PARSYS_PARTNUM          = 1,    /**< This parameter is read-only and will be factory set during production. It contains the part
                                                 number (article number) including revision letter of the INS. */
    XCOMPAR_PARSYS_SERIALNUM        = 2,    /**< This parameter is read-only and will be factory set during production. It contains the serial
                                                 number of the INS. */
    XCOMPAR_PARSYS_MFG              = 3,    /**< This parameter is read-only and will be factory set during production. It contains the
                                                 manufacturing date of the INS. */
    XCOMPAR_PARSYS_CALDATE          = 4,    /**< This parameter is read-only and will be factory set during calibration. It contains the last
                                                 calibration date of the INS. */
    XCOMPAR_PARSYS_FWVERSION        = 5,    /**< This parameter is read-only and will be factory set during production. It contains the firmware
                                                 version currently operating on the INS. */
    XCOMPAR_PARSYS_NAVLIB           = 6,    /**< This parameter is read-only and will be factory set during production. It contains the
                                                 NAVLIB version currently installed on the INS. */
    XCOMPAR_PARSYS_EKFLIB           = 7,    /**< This parameter is read-only and will be factory set during production. It contains the version
                                                 number of the EKFLIB currently installed on the device. */
    XCOMPAR_PARSYS_EKFPARSET        = 8,    /**< This parameter is read-only and will be factory set during production. It contains the name
                                                 of the EKF parameter set being used currently. */
    XCOMPAR_PARSYS_NAVNUM           = 9,    /**< This parameter is read-only and will be factory set during production. It contains the version
                                                 number of the strapdown navigation module installed on the system. */
    XCOMPAR_PARSYS_NAVPARSET        = 10,   /**< This parameter is read-only and will be factory set during production. */
    XCOMPAR_PARSYS_MAINTIMING       = 11,   /**< This parameter is read-only and will be factory set during production. This parameter contains
                                                 the nominal sampling rate in Hz of the integrated inertial sensors. Writing of this parameter
                                                 is password-protected. */
    XCOMPAR_PARSYS_PRESCALER        = 12,   /**< This parameter is read-only and will be factory set during production. It contains the
                                                 downsampling factor which is used in the internal compensation methods, such as the coning and
                                                 sculling correction module. The prescaler also affects the maximum rate which is applicable
                                                 through the message log mechanism. Writing this parameter is password-protected. */
    XCOMPAR_PARSYS_UPTIME           = 13,   /**< This parameter is read-only. The parameter contains the system’s current uptime (time since
                                                 last reboot) in [sec]. */
    XCOMPAR_PARSYS_OPERATIONHOUR    = 14,   /**< This parameter is read-only. It contains the accumulated operating time in [sec] over the
                                                 lifetime of the system. */
    XCOMPAR_PARSYS_BOOTMODE         = 15,   /**< This parameter defines the boot behavior of the system. In the default setting (= 0), the
                                                 system will start in normal operational mode. When setting this parameter to standby (=1),
                                                 the system will boot into power saving mode and to enter normal operational mode, the wakeup
                                                 command will have to be sent. */
    XCOMPAR_PARSYS_FPGAVERSION      = 16,   /**< This read-only parameter contains the FPGA firmware version. */
    XCOMPAR_PARSYS_CONFIGCRC        = 17,   /**< This read-only parameter holds the checksums calculated over the internal configuration binary files. */
    XCOMPAR_PARSYS_OSVERSION        = 18,   /**< This read-only parameter contains the version number of the host operating system. */
    XCOMPAR_PARSYS_SYSNAME          = 19,   /**< This read-only parameter contains the product name, e.g. iNAT-FSSG-01. */
    XCOMPAR_PARSYS_MINFPGAVER       = 20,   /**< This read-only parameter defines the minimum required FPGA firmware version for the firmware
                                                 installed on the device. If the actual FPGA version is lower than the version given in this
                                                 parameter, some firmware features may not be available. */
    XCOMPAR_PARSYS_SYNCMODE         = 21,   /**< This parameter defines the time synchronization source. */
    XCOMPAR_PARSYS_CALIBID          = 22,   /**< This parameter defines the calibration ID which is need fpr an unique identifier during the factory calibration */
    XCOMPAR_PARSYS_EEPROM           = 23,   /**< This parameter holds the content of the EEPROM device. The EEPROM device stores hardware dependent information */
    XCOMPAR_PARSYS_GNSSTYPE         = 24,   /**< This parameter holds the GNSS receiver type and hardware information */
    XCOMPAR_PARSYS_GITSHA           = 25,   /**< This parameter holds the git hash of the current commit of the firmware source code. */
    XCOMPAR_PARSYS_GITBRANCH        = 26,   /**< This parameter holds the git branch name of the firmware source code. */
    XCOMPAR_PARSYS_CIPIPELINEID     = 27,   /**< This parameter holds the ci/cd pipeline id */
    XCOMPAR_PARSYS_USBCONFIG        = 28,   /**< This parameter configures the USB interface */
    XCOMPAR_PARSYS_IMUIDENT         = 29,   /**< This parameter holds the IMU identification (manufacturer and model) */
    XCOMPAR_PARSYS_FEATURES         = 30,   /**< This parameter is used to get the firmware features */
    XCOMPAR_PARSYS_ROOTFSVERSION    = 31,   /**< This parameter holds rootfs version string */
    XCOMPAR_PARSYS_CPUINFO          = 32,   /**< This parameter contains the cpu information */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * IMU Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARIMU_MISALIGN         = 105,  /**< This parameter sets the installation misalignment between the INS enclosure and the
                                                 vehicle frame. */
    XCOMPAR_PARIMU_TYPE             = 107,  /**< This parameter defines the inertial sensor type and will be factory set during
                                                 production. */
    XCOMPAR_PARIMU_LATENCY          = 108,  /**< This parameter adjusts the timestamp of the inertial data which is used to synchronize
                                                 IMU measurements with external sensors */
    XCOMPAR_PARIMU_CALIBCOARSE      = 109,  /**< This parameter defines calibration parameters which will be used by the system to
                                                 enable additional correction of the calibrated IMU values. */
    XCOMPAR_PARIMU_CROSSCOUPLING    = 110,  /**< This parameter stores crosscoupling matrices which are used by the system after
                                                 application of PARIMU_CALIB and before strapdown and EKF calculation */
    XCOMPAR_PARIMU_REFPOINTOFFSET   = 111,  /**< This parameter stores the offset of the enclosure reference point with respect to
                                                 the center of navigation. */
    XCOMPAR_PARIMU_BANDSTOP         = 112,  /**< This parameter configures the bandstop filter which can be enabled for the accelerations
                                                 and angular rates */
    XCOMPAR_PARIMU_COMPMETHOD       = 113,  /**< This parameter configures the downsampling behavior prior to strapdown calculations if
                                                 the prescaler value in PARSYS_PRESCALER is greater than 1. */
    XCOMPAR_PARIMU_ACCLEVERARM      = 114,  /**< This parameter configures coordinates of the accelerometers in the body frame.
                                                 These values are used to compensate for the size effect. */
    XCOMPAR_PARIMU_STRAPDOWNCONF    = 115,  /**< This parameter configures the strapdown algorithm for different types of IMUs */
    XCOMPAR_PARIMU_C_ENC_IMU        = 116,  /**< This parameter defines the rotation matrix from the iNAT standard enclosure frame
                                                 (z down, x axis pointing from the plugs into the device) to a custom enclosure frame */
    XCOMPAR_PARIMU_RANGE            = 117,  /**< This parameter defines the measurement range of the IMU */
    XCOMPAR_PARIMU_CALIBDATA        = 118,  /**< This parameter reads/writes the calibration data of the IMU */
    XCOMPAR_PARIMU_GYROBIASMODELING = 119,  /**< This parameter enables/disables the internal gyro bias model */
    XCOMPAR_PARIMU_CALIBTEMPERATURE = 120,  /**< This parameter enables/disables the calibration over temperature */
    XCOMPAR_PARIMU_SECONDARY        = 121,  /**< This parameter enables/disables the secondary IMU (if available) */
    XCOMPAR_PARIMU_CALIBMODEL       = 122,  /**< This parameter configures the IMU calibration model */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * GNSS Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARGNSS_PORT            = 200,  /**< This parameter is read-only and will be factory set during production */
    XCOMPAR_PARGNSS_BAUD            = 201,  /**< This parameter is read-only and will be set during production */
    XCOMPAR_PARGNSS_ANTOFFSET       = 204,  /**< This parameter configures the GNSS lever arm of the primary or secondary antenna */
    XCOMPAR_PARGNSS_RTKMODE         = 207,  /**< This parameter enables/disables the RTK mode */
    XCOMPAR_PARGNSS_AIDFRAME        = 209,  /**< This parameter sets the GNSS aiding frame. The configured frame is valid for GNSS
                                                  position and velocity aiding. */
    XCOMPAR_PARGNSS_RTCMV3AIDING    = 210,  /**< This parameter can be used to input DGNSS correction data into the internal GNSS receiver
                                                 over an iXCOM connection */
    XCOMPAR_PARGNSS_DUALANTMODE     = 211,  /**< This parameter enables or disables GNSS dual antenna mode inside the GNSS module. */
    XCOMPAR_PARGNSS_LOCKOUTSYSTEM   = 212,  /**< This parameter can be used to exclude certain satellite constellations from the GNSS
                                                 solution computation. */
    XCOMPAR_PARGNSS_RTCMV3CONFIG    = 213,  /**< This parameter enables or disables the RTCMv3 forwarding module. */
    XCOMPAR_PARGNSS_NAVCONFIG       = 214,  /**< This parameter contains the configuration for satellite elevation and CN0cutoff.*/
    XCOMPAR_PARGNSS_STDDEV          = 215,  /**< This parameter scales/limits the standard deviation of the GNSS engine. For every payload
                                                 entry of this parameter, only values ≥ 0 are accepted. */
    XCOMPAR_PARGNSS_VOTER           = 216,  /**< This parameter configures the GNSS voter module. */
    XCOMPAR_PARGNSS_MODEL           = 217,  /**< This parameter gives a list of valid authorized models available and expiry date information */
    XCOMPAR_PARGNSS_VERSION         = 218,  /**< This parameter contains GNSS receiver version information, both for hard- and software components */
    XCOMPAR_PARGNSS_RTKSOLTHR       = 219,  /**< This parameter sets the threshold for the RTK solution status */
    XCOMPAR_PARGNSS_TERRASTAR       = 220,  /**< This parameter enables/disables the usage of PPP solution */
    XCOMPAR_PARGNSS_REFSTATION      = 221,  /**< This parameter configures the GNSS reference station feature. */
    XCOMPAR_PARGNSS_FIXPOS          = 222,  /**< This parameter configures the fixed position of the GNSS receiver */
    XCOMPAR_PARGNSS_POSAVE          = 223,  /**< This parameter configures the position averaging on the integrated GNSS receiver. */
    XCOMPAR_PARGNSS_CORPORTCFG      = 224,  /**< This parameter configures the serial correction port of the internal GNSS receiver */
    XCOMPAR_PARGNSS_GATEWAYCFG      = 225,  /**< This parameter configures the GNSS gateway */
    XCOMPAR_PARGNSS_SWITCHER        = 226,  /**< This parameter configures the GNSS switcher (internal/external GNSS receiver) */
    XCOMPAR_PARGNSS_ANTENNAPOWER    = 227,  /**< This parameter  enables or disables the supply of electrical power from the internal power
                                                 source of the receiver */
    XCOMPAR_PARGNSS_PPSCONTROL      = 228,  /**< This parameter provides a method for controlling the polarity, period and pulse width of
                                                 the PPS output  */
    XCOMPAR_PARGNSS_HDGOFFSET       = 229,  /**< This parameter adds on offset to the dual antenna GNSS heading */
    XCOMPAR_PARGNSS_STATUSCONFIG    = 230,  /**< This parameter is used to configure the various status mask fields in the RXSTATUSEVENT log */
    XCOMPAR_PARGNSS_FWUPDATE        = 231,  /**< This parameter is used to start a firmware update of the internal GNSS receiver */
    XCOMPAR_PARGNSS_PERIOD          = 232,  /**< This parameter is used to set the update period of the internal GNSS receiver */
    XCOMPAR_PARGNSS_FEATURES        = 233,  /**< This parameter is used to get the features of the internal GNSS receiver */
    XCOMPAR_PARGNSS_REFSTATIONV2    = 234,  /**< This parameter is used to configure the next generation of iMAR's reference stations */
    XCOMPAR_PARGNSS_GEOIDSELECTION  = 235,  /**< This parameter is used to switch between the receivers geoid model and the inat internal model */
    XCOMPAR_PARGNSS_ANTCONFIG       = 236,  /**< This parameter configures the GNSS antenna interface */
    XCOMPAR_PARGNSS_GGAQUALITY      = 237,  /**< This parameter is used to customize the receiver's NMEA0183-GGA quality indicator */
    XCOMPAR_PARGNSS_FIXPOSV2        = 238,  /**< This parameter configures the fixed position of the GNSS receiver */
    XCOMPAR_PARGNSS_PVTMODE         = 239,  /**< This parameter configures the PVT mode of the GNSS engine */
    XCOMPAR_PARGNSS_FRONTENDMODE    = 240,  /**< This parameter configures the frontend mode of the GNSS engine */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * MAG Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARMAG_PORT     = 300,  /**< This parameter configures the serial port of the magnetometer module. */
    XCOMPAR_PARMAG_RATE     = 302,  /**< This parameter configures the sampling rate of the magnetometer module. */
    XCOMPAR_PARMAG_MISALIGN = 304,  /**< This parameter changes the misalignment of the magnetometer reference frame with
                                         respect to the IMU reference frame */
    XCOMPAR_PARMAG_CAL      = 307,  /**< This parameter contains the calibration parameters of the magnetometer. */
    XCOMPAR_PARMAG_CALSTATE = 308,  /**< This parameter holds the on-board magnetometer calibration procedure status of the 2D
                                         magnetometer calibration routine. */
    XCOMPAR_PARMAG_FOM      = 309,  /**< This parameter contains the Figure Of Merit (FOM) of the 2D magnetometer calibration method. */
    XCOMPAR_PARMAG_CFG      = 310,  /**< This parameter contains the magnetometer configuration items. */
    XCOMPAR_PARMAG_ENABLE   = 311,  /**< This parameter enables/disables the magnetometer modules */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * MADC Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_MADC_ENABLE     = 400,  /**< This parameter enables/disables the MADC module */
    XCOMPAR_MADC_LEVERARM   = 401,  /**< This parameter configures the lever arm between the static port and the INS. It should be
                                         measured in INS enclosure frame coordinates with respect to the enclosure reference point. All
                                         leverarm standard deviations only accept values > 0. */
    XCOMPAR_MADC_LOWPASS    = 402,  /**< This parameter configures the low pass filter inside the iMADC module. Regarding the cut-off
                                         frequency, only values > 0 Hz are accepted. */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * REC Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_REC_CONFIG      = 600,    /**< This parameter specifies the behavior of the recording module. */
    XCOMPAR_REC_START       = 603,    /**< This parameter starts the recording module if the “Autostart” field of PARREC_CONFIG is set to 0 */
    XCOMPAR_REC_STOP        = 604,    /**< This parameter stops the recording module and is write-only. */
    XCOMPAR_REC_POWERLOGGER = 605,    /**< @deprecated */
    XCOMPAR_REC_SUFFIX      = 606,    /**< This parameter contains a user defined suffix string, which will be attached to the default log directory name. */
    XCOMPAR_REC_DISKSPACE   = 607,    /**< This parameter contains the disk space, which is left over on the system e.g. for saving logs. */
    XCOMPAR_REC_AUXILIARY   = 608,    /**< @deprecated */
    XCOMPAR_REC_CURRENTFILE = 609,    /**< This parameters is read-only and returns the path of the current log file */
    XCOMPAR_REC_LOGROTATE   = 610,    /**< This parameter enables/disables log rotation */
    XCOMPAR_REC_DEVICE      = 611,    /**< This parameter selects the recording device */
    XCOMPAR_REC_FILESIZE    = 612,    /**< This parameter defines the max. file size in MB */
    XCOMPAR_REC_MOTION      = 613,    /**< This parameter configures the motion-dependent data recording  */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * EKF Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PAREKF_ALIGNMODE        = 700,    /**< @deprecated */
    XCOMPAR_PAREKF_ALIGNTIME        = 701,    /**< @deprecated */
    XCOMPAR_PAREKF_COARSETIME       = 702,    /**< @deprecated */
    XCOMPAR_PAREKF_VMP              = 703,    /**< This parameter specifies a virtual measurement point. */
    XCOMPAR_PAREKF_AIDING           = 704,    /**< This parameter defines the used aiding sources of the extended Kalman filter. */
    XCOMPAR_PAREKF_DELAY            = 706,    /**< This parameter defines the delay in [ms] of the delayed navigation module against the
                                                   real-time navigator */
    XCOMPAR_PAREKF_STARTUP          = 707,    /**< @deprecated */
    XCOMPAR_PAREKF_HDGPOSTHR        = 708,    /**< This parameter defines the thresholds of the Alignment Status and Position Accuracy fields
                                                   of the global status  */
    XCOMPAR_PAREKF_SMOOTH           = 709,    /**< This parameter defines the smoothing factor in [samples]. */
    XCOMPAR_PAREKF_ZUPT             = 712,    /**< This parameter contains the configuration of the zero velocity detector */
    XCOMPAR_PAREKF_ZUPT2            = 713,    /**< This parameter contains the extended configuration of the zero velocity detector */
    XCOMPAR_PAREKF_DEFPOS           = 714,    /**< This parameter defines the default position */
    XCOMPAR_PAREKF_DEFHDG           = 715,    /**< This parameter defines the default heading */
    XCOMPAR_PAREKF_OUTLIER          = 716,    /**< This parameter defines the outlier rejection mask of the integrated Kalman filter. */
    XCOMPAR_PAREKF_POWERDOWN        = 717,    /**< This parameter defines the power-down behavior of the system */
    XCOMPAR_PAREKF_EARTHRAD         = 718,    /**< This parameter is read-only and contains the earth radii. */
    XCOMPAR_PAREKF_STOREDPOS        = 719,    /**< This read-only parameter provides the stored position and the associated standard deviation. */
    XCOMPAR_PAREKF_ALIGNZUPTSTDDEV  = 720,    /**< @deprecated */
    XCOMPAR_PAREKF_POSAIDSTDDEVTHR  = 721,    /**< This parameter configures the standard deviation threshold of the position aiding module */
    XCOMPAR_PAREKF_SCHULERMODE      = 722,    /**< This parameter configures the system for Schuler Mode test. */
    XCOMPAR_PAREKF_STOREDATT        = 723,    /**< This parameter is read-only and contains the stored attitude. */
    XCOMPAR_PAREKF_ODOMETER         = 724,    /**< This parameter provides advanced odometer configuration
                                                   options which can be used to accommodate for odometer signal abnormalities. */
    XCOMPAR_PAREKF_ODOBOGIE         = 725,    /**< This parameter configures the odometer data processing module for railway applications */
    XCOMPAR_PAREKF_GNSSLEVERARMEST  = 726,    /**< This parameter configures GNSS lever arm estimation. */
    XCOMPAR_PAREKF_GNSSAIDRATE      = 727,    /**< This parameter configures the GNSS aiding rates inside the Kalman filter. */
    XCOMPAR_PAREKF_KINALIGNTHR      = 728,    /**< @deprecated */
    XCOMPAR_PAREKF_GNSSPDOP         = 729,    /**< This parameter configures the PDOP threshold inside the Kalman filter */
    XCOMPAR_PAREKF_DUALANTAID       = 730,    /**< This parameter configures the dual antenna aiding inside the Kalman filter. */
    XCOMPAR_PAREKF_STARTUPV2        = 731,    /**< This parameter can be used to define the initialization behaviour of the Kalman filter. */
    XCOMPAR_PAREKF_MAGATTAID        = 732,    /**< This parameter configures the iMAG parameters inside the Kalman filter. */
    XCOMPAR_PAREKF_MADCAID          = 733,    /**< This parameter configures the iMADC parameters inside the Kalman filter. */
    XCOMPAR_PAREKF_ALIGNMENT        = 734,    /**< This parameter configures the alignment method of the INS */
    XCOMPAR_PAREKF_GRAVITYAIDING    = 735,    /**< This parameter configures the gravity aiding inside the Kalman filter. */
    XCOMPAR_PAREKF_FEEDBACK         = 736,    /**< This parameter enables or disables the error compensation in the real-time navigation module */
    XCOMPAR_PAREKF_ZARU             = 737,    /**< This parameter configures Zero Angular Rate Updates. */
    XCOMPAR_PAREKF_IMUCONFIG        = 738,    /**< This parameter defines the initial standard deviations and the process noise model for
                                                   the used inertial sensors. */
    XCOMPAR_PAREKF_ZUPTCALIB        = 739,    /**< This parameter defines the calibration time for the Zero Velocity Update mechanism. */
    XCOMPAR_PAREKF_STATEFREEZE      = 740,    /**< This parameter contains the state freeze mask for realtime navigation. */
    XCOMPAR_PAREKF_RECOVERY         = 741,    /**< This parameter contains the realignment condition bit mask. */
    XCOMPAR_PAREKF_STOREDVALSTDDEV  = 742,    /**< This parameter overwrites the stored position standard deviation */
    XCOMPAR_PAREKF_ODOCHECK         = 743,    /**< This parameter configures the odometer outlier detection module */
    XCOMPAR_PAREKF_ODOREVDETECT     = 744,    /**< This parameter configures the odometer reverse detection module */
    XCOMPAR_PAREKF_PSEUDOPOS        = 745,    /**< This parameter configures the pseudo position output */
    XCOMPAR_PAREKF_NISTHR           = 746,    /**< This parameter configures behaviour of the outlier detection */
    XCOMPAR_PAREKF_FHUPD            = 747,    /**< This parameter configures the 'frozen heading update' module */
    XCOMPAR_PAREKF_FORCEDZUPT       = 748,    /**< This parameter enables/disables forced zero velocity updates */
    XCOMPAR_PAREKF_ZARUSTDDEV       = 749,    /**< This parameter configures the standard deviation of the Zero Angular Rate Updates. */
    XCOMPAR_PAREKF_FPUPD            = 750,    /**< This parameter configures the 'frozen position update' module */
    XCOMPAR_PAREKF_SLEWMODE         = 751,    /**< This parameter configures the operation mode of the slew module */
    XCOMPAR_PAREKF_ODOLEVERARMEST   = 752,    /**< This parameter configures the odometer lever-arm estimation module */
    XCOMPAR_PAREKF_REFFRAME         = 753,    /**< This parameter configures the reference frame for GNSS RTK data */
    XCOMPAR_PAREKF_AUTOHEIGHT       = 754,    /**< This parameter configures the automatic heigt updates */
    XCOMPAR_PAREKF_AUTOREALIGN      = 755,    /**< This parameter configures the automatic realignment mechanism */
    XCOMPAR_PAREKF_ZUPTREQUEST      = 756,    /**< This parameter configures the automatic ZUPT request feature */
    XCOMPAR_PAREKF_POSUPDATEREQUEST = 757,    /**< This parameter configures the automatic position update request feature */
    XCOMPAR_PAREKF_STOREDALIGNMODE  = 758,    /**< This parameter configures the verification process during stored heading alignment */
    XCOMPAR_PAREKF_EXTAIDREFFRAME   = 759,    /**< This parameter configures the reference frame for external position data */
    XCOMPAR_PAREKF_IMUCONFIG2       = 760,    /**< This parameter defines the initial standard deviations and the process noise model for
                                                   the used inertial sensors (read only). */
    XCOMPAR_PAREKF_MUTEGYROBIASRW   = 761,    /**< This parameter is used to configure the gyro bias muting module */
    XCOMPAR_PAREKF_FM0MOTIONCTRL    = 762,    /**< This parameter is used to configure the motion control in filter-mode 0 */
    XCOMPAR_PAREKF_ZDRUPT           = 763,    /**< This parameter is used to enable/disable ZDRUPT within the EKF module */
    XCOMPAR_PAREKF_STATERECOVERY    = 764,    /**< This parameter is used to enable/disable EKF state recovery during initialization */
    XCOMPAR_PAREKF_WAHBA            = 765,    /**< This parameter is used to configure the wahba alignment filter */
    XCOMPAR_PAREKF_MUTEACCELBIASRW  = 766,    /**< This parameter is used to configure the accel. bias muting module */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * DAT Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARDAT_POS      = 800,    /**< This parameter defines the position output frame of the INSSOL message. */
    XCOMPAR_PARDAT_VEL      = 801,    /**< This parameter defines the velocity output frame of the INSSOL message. */
    XCOMPAR_PARDAT_IMU      = 802,    /**< This parameter defines the kind of inertial data of the INSSOL message. */
    XCOMPAR_PARDAT_SYSSTAT  = 803,    /**< This parameter defines the content of the XComMsgSystemstatus message. */
    XCOMPAR_PARDAT_REFFRAME = 804,    /**< This parameter configures the output reference frame */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * XCOM Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARXCOM_SERIALPORT      = 902,    /**< @deprecated */
    XCOMPAR_PARXCOM_NETCONFIG       = 903,    /**< This parameter configures the network  interface of the system. */
    XCOMPAR_PARXCOM_LOGLIST         = 905,    /**< @deprecated */
    XCOMPAR_PARXCOM_AUTOSTART       = 906,    /**< @deprecated */
    XCOMPAR_PARXCOM_NTRIP           = 907,    /**< This parameter configures the integrated NTRIP client. */
    XCOMPAR_PARXCOM_POSTPROC        = 908,    /**< This parameter allows an easy setup of the internal data logger if post-processing or
                                                   detailed offline analysis is required. If enabled, logging will be automatically started
                                                   after bootup. */
    XCOMPAR_PARXCOM_BROADCAST       = 909,    /**< This parameter configures the iXCOM UDP broadcasting. */
    XCOMPAR_PARXCOM_UDPCONFIG       = 910,    /**< This parameter configures the iXCOM UDP module. */
    XCOMPAR_PARXCOM_DUMPENABLE      = 911,    /**< @deprecated */
    XCOMPAR_PARXCOM_MIGRATOR        = 912,    /**< @deprecated */
    XCOMPAR_PARXCOM_TCPKEEPAL       = 913,    /**< @deprecated */
    XCOMPAR_PARXCOM_CANGATEWAY      = 914,    /**< @deprecated */
    XCOMPAR_PARXCOM_DEFAULTIP       = 915,    /**< This parameter holds the system default IP address, used as fallback option. */
    XCOMPAR_PARXCOM_ABDCONFIG       = 916,    /**< This parameter configures the XComMsgAbdProtocol module */
    XCOMPAR_PARXCOM_LOGLIST2        = 917,    /**< This parameter contains 16 structs, each holding a pair of message ID with a divider for
                                                   configuring certain data logs. This parameter is read only and should only be used to read
                                                   a log-list of a specific iXCOM channel. If the log-list should be changed, please use the
                                                   parameter PARXCOM_LOGLIST. */
    XCOMPAR_PARXCOM_CALPROC         = 918,    /**< This parameter contains entries related to the calibration processing interface. */
    XCOMPAR_PARXCOM_CLIENT          = 919,    /**< @deprecated This parameter is for configuring an iXCOM client and its client logs. The
                                                   maximum supported number of clients is currently eight. */
    XCOMPAR_PARXCOM_FRAMEOUT        = 920,    /**< This parameter allows to set up pre-configured output frames (PARIMU_C_ENC_IMU) for different
                                                   custom coordinate systems. Depending on the value of this parameter, the output roll and
                                                   pitch angles are also adjusted. */
    XCOMPAR_PARXCOM_INTERFACE       = 921,    /**< This parameter contains the interface configuration of the available serial ports */
    XCOMPAR_PARXCOM_MONITOR         = 922,    /**< This parameter configures the monitor module */
    XCOMPAR_PARXCOM_QCONN           = 923,    /**< This parameter enables/disables the qconn debugging client */
    XCOMPAR_PARXCOM_NETCHECK        = 924,    /**< This parameter enables/disables iMAR's netcheck */
    XCOMPAR_PARXCOM_FASTBOOT        = 926,    /**< This parameter enables/disables fast boot operation */
    XCOMPAR_PARXCOM_CONSOLE         = 927,    /**< This parameter enables/disables the debugging agent */
    XCOMPAR_PARXCOM_DNSCONFIG       = 928,    /**< This parameter configures the internal nameserver */
    XCOMPAR_PARXCOM_PLUGINVERSION   = 929,    /**< This parameter requests version information of the installed plugins */
    XCOMPAR_PARXCOM_NTRIPV2         = 930,    /**< This parameter configures the integrated NTRIP client. */
    XCOMPAR_PARXCOM_ABDVERSION      = 931,    /**< This parameter configures the ABD protocol version */
    XCOMPAR_PARXCOM_MQTTCONFIG      = 932,    /**< This parameter configures MQTT monitor module */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * FPGA Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARFPGA_IMUSTATUSREG    = 1000,    /**< The IMU status register contains information about the FPGA IMU module. The content is IMU
                                                    dependent and the used type of IMU can be determined via the PARIMU_TYPE  */
    XCOMPAR_PARFPGA_HDLC            = 1001,    /**< The HDLC configuration register holds the HDLC configuration of the system. */
    XCOMPAR_PARFPGA_TIMING          = 1002,    /**< he FPGA timing configuration register holds the trigger configuration of the system. */
    XCOMPAR_PARFPGA_TIMER           = 1003,    /**< This parameter contains an array, holding the FPGA timer matrix rows. */
    XCOMPAR_PARFPGA_INTERFACE       = 1004,    /**< This parameter contains an array, holding the FPGA interface matrix rows. */
    XCOMPAR_PARFPGA_CONTROLREG      = 1005,    /**< This parameter contains the FPGA control register mask. */
    XCOMPAR_PARFPGA_POWERUPTHR      = 1006,    /**< his parameter contains the power up threshold for the input voltage. If this input
                                                    value is below the defined threshold, all electrical loads will be shut down. */
    XCOMPAR_PARFPGA_INTMAT245       = 1007,    /**< @deprecated */
    XCOMPAR_PARFPGA_TYPE            = 1008,    /**< This read-only parameter gives information about the FPGA used in the device. */
    XCOMPAR_PARFPGA_GLOBALCONF      = 1009,    /**< @deprecated */
    XCOMPAR_PARFPGA_HDLCPINMODE     = 1010,    /**< This parameter configures the modes of the HDLC pins */
    XCOMPAR_PARFPGA_POWER           = 1011,    /**< This parameter contains the FPGA power switch mask */
    XCOMPAR_PARFPGA_ALARMTHR        = 1012,    /**< @deprecated */
    XCOMPAR_PARFPGA_PPTCONFIG       = 1013,    /**< This parameter configures the PPT output behavior. The PPT pulse is generated with the
                                                    divider defined in field 1 of the payload from the IMU trigger. */
    XCOMPAR_PARFPGA_AUTOWAKEUP      = 1014,    /**< This parameter configures the FPGA auto wakeup option. */
    XCOMPAR_PARFPGA_MCP23S08        = 1015,    /**< This parameter configures the inputs/outputs of the MCP23S08 chip. The behavior of
                                                    the chip depends on the hardware platform */
    XCOMPAR_PARFPGA_CSAC            = 1016,    /**< This parameter configures the chip scale atomic clock (CSAC) */
    XCOMPAR_PARFPGA_GOBITMASK       = 1017,    /**< This parameter defines which sources are used for the FPGA NOGO bit. If a bit in the
                                                    mask is set ’0’, the corresponding error will not trigger an FPGA NOGO error. */
    XCOMPAR_PARFPGA_IMUPOWERMASK    = 1018,    /**< This parameter defines which sources are used for the IMU power error bit. */
    XCOMPAR_PARFPGA_SYSPOWERMASK    = 1019,    /**< This parameter defines which sources are used for the system power error bit. */
    XCOMPAR_PARFPGA_CSACAUTO        = 1020,    /**< This parameter configures the automatic selection of internal/external clock of the
                                                    chip scale atomic clock (CSAC), when automatic mode is enabled */ 
    XCOMPAR_PARFPGA_TRIGTIMEOUT     = 1021,    /**< This parameter defines the time after which a IMU trigger timeout should be reported. */
    XCOMPAR_PARFPGA_HOLDOVERTIME    = 1022,    /**< This parameter defines the holdover time of the system */
    XCOMPAR_PARFPGA_VMSPBIT         = 1023,    /**< This parameter configures the VMS PBIT module */
    XCOMPAR_PARFPGA_USBID           = 1024,    /**< This parameter configures the 5V output via USB connector */
    XCOMPAR_PARFPGA_IOVP            = 1025,    /**< This parameter configures the thresholds for the input over-voltage protection module */
    XCOMPAR_PARFPGA_DEVICE          = 1026,    /**< This read-only parameter gives information about the FPGA device */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * ODO Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARODO_SCF            = 1100,    /**< This parameter defines the odometer scale factor in [meter/tick]. */
    XCOMPAR_PARODO_TIMEOUT        = 1101,    /**< This parameter defines the odometer timeout in [sec].  */
    XCOMPAR_PARODO_MODE           = 1102,    /**< This parameter defines the odometer mode and its related items. */
    XCOMPAR_PARODO_LEVERARM       = 1103,    /**< This parameter configures the lever arm between the INS and the odometer sensor.
                                                  The measurement should be done as accurately as possible. The x, y and z values of the
                                                  parameter represent the vector from the INS enclosure reference point to the odometer
                                                  sensor. */
    XCOMPAR_PARODO_VELSTDDEV      = 1104,    /**< This parameter configures the odometer velocity standard deviation used for EKF aiding.
                                                  The standard deviation entry only accepts values > 0. */
    XCOMPAR_PARODO_DIRECTION      = 1105,    /**< This parameter configures the odometer mounting direction. The direction vector must
                                                  be normalized to 1 */
    XCOMPAR_PARODO_CONSTRAINTS    = 1106,    /**< This parameter configures the odometer constraints. If the constraints are enabled the EKF
                                                  will perform ZUPTS aiding for these axis that are not selected via the PARODO_DIRECTION
                                                  parameter. The parameter entry for the standard deviation of the odometer constraint
                                                  only accepts value > 0. */
    XCOMPAR_PARODO_RATE           = 1107,    /**< This parameter configures the maximal odometer measurement update rate. The rate entry
                                                  only accepts values > 0 Hz. */
    XCOMPAR_PARODO_THR            = 1108,    /**< This parameter contains the threshold values. Processing velocity via odometer measurement
                                                  will be disabled if values exceed these limitations. As threshold parameter entries,
                                                  only positive values ≥ 0 are allowed. */
    XCOMPAR_PARODO_EQEP           = 1109,    /**< @deprecated */
    XCOMPAR_PARODO_INVERTER       = 1110,    /**< This parameter configures the odometer inverter. The default value of this parameter makes
                                                  the odometer interface compliant to previous generations of iNAT hardware. */
    XCOMPAR_PARODO_SWAP           = 1111,    /**< This parameter swaps the odometer inputs (A/B) */
    XCOMPAR_PARODO_PPDDIVIDER     = 1112,    /**< This parameter configures the pulse per distance output */
    XCOMPAR_PARODO_MODE2          = 1113,    /**< This parameter defines the odometer mode and its related items. */
    XCOMPAR_PARODO_LATENCY        = 1114,    /**< This parameter adjusts the timestamp of the odometer data */
    XCOMPAR_PARODO_LATCONSTRAINTS = 1115,    /**< This parameter adjusts the timestamp of the odometer data */
    XCOMPAR_PARODO_VELSIGN        = 1116,    /**< This parameter configures the odometer velocity sign */
    XCOMPAR_PARODO_PRIMIDX        = 1117,    /**< This parameter selects the primary odometer */
    XCOMPAR_PARODO_CBIT           = 1118,    /**< This parameter configures the odometer cbit module */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * NMEA0183 Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARNMEA0183_COM         = 1300,    /**< This parameter configures the NMEA com port */
    XCOMPAR_PARNMEA0183_ENABLE      = 1301,    /**< This parameter configures the GPS quality mode and source selection */
    XCOMPAR_PARNMEA0183_TXMASK      = 1302,    /**< This parameter selects the messages to transmit (UDP or UART) within the NMEA module */
    XCOMPAR_PARNMEA0183_RATE        = 1304,    /**< This parameter sets the data rate (UDP or UART) within the NMEA module. */
    XCOMPAR_PARNMEA0183_UDP         = 1305,    /**< This parameter configures the UDP transmission within the NMEA module. */
    XCOMPAR_PARNMEA0183_VTGSELECT   = 1306,    /**< This parameter configures the source of the heading of the VTG message. */
    XCOMPAR_PARNMEA0183_PRECISION   = 1307,    /**< This parameter defines the precision of the NMEA0183 protocol */
    XCOMPAR_PARNMEA0183_LOCALZONE   = 1308,    /**< This parameter defines the local zone description of the ZDA sentence */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * ARINC825 Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_ARINC825_PORT           = 1200,    /**< @deprecated */
    XCOMPAR_ARINC825_BAUD           = 1201,    /**< @deprecated */
    XCOMPAR_ARINC825_ENABLE         = 1202,    /**< @deprecated */
    XCOMPAR_ARINC825_FRAMELIST      = 1204,    /**< @deprecated */
    XCOMPAR_ARINC825_BUSRECOVERY    = 1205,    /**< @deprecated */
    XCOMPAR_ARINC825_RESETSTATUS    = 1206,    /**< @deprecated */
    XCOMPAR_ARINC825_SCALEFACTOR    = 1207,    /**< @deprecated */
    XCOMPAR_ARINC825_EVENTMASK      = 1208,    /**< @deprecated */
    XCOMPAR_ARINC825_GENERALCAN     = 1209,    /**< @deprecated */
    XCOMPAR_ARINC825_PROTOCOL       = 1210,    /**< @deprecated */
    XCOMPAR_ARINC825_OBDII          = 1211,    /**< @deprecated */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * ARINC429 Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARARINC429_CONFIGPRIVATE   = 1400, /**< @deprecated (private) */
    XCOMPAR_PARARINC429_LIST            = 1401, /**< @deprecated */
    XCOMPAR_PARARINC429_CONFIG          = 1402, /**< @deprecated (private) */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * I/O Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARIO_HW245     = 1500,    /**< @deprecated */
    XCOMPAR_PARIO_HW288     = 1501,    /**< @deprecated */
    XCOMPAR_PARIO_SYNCOUT   = 1502,    /**< @deprecated */
    XCOMPAR_PARIO_SYNCOUT2  = 1503,    /**< This parameter configures the synchronization output module */
    XCOMPAR_PARIO_SYNCIN    = 1504,    /**< This parameter configures the synchronization input module */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * SCU Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARSCU_NADIR    = 1600,    /**< @deprecated */
    XCOMPAR_PARSCU_STAB     = 1601,    /**< @deprecated */
    XCOMPAR_PARSCU_PARAM    = 1602,    /**< @deprecated */
    XCOMPAR_PARSCU_ERRBUF   = 1603,    /**< @deprecated */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * TIMESYNC Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARTIMESYNC_NTP     = 1800,    /**< This parameter configures the NTP module */
    XCOMPAR_PARTIMESYNC_PEGASUS = 1801,    /**< This parameter configures the pegasus timing module */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * CAN Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARCAN_DEVICECONFIG     = 1900,    /**< This parameter configures the internal CAN controller */
    XCOMPAR_PARCAN_SIMPLECAN        = 1901,    /**< This parameter configures the internal SimpleCAN protocol stack  */
    XCOMPAR_PARCAN_BUSTERMINATION   = 1902,    /**< This parameter configures the bus termination resistors */
    XCOMPAR_PARCAN_VRUCAN           = 1903,    /**< This parameter configures the internal VRU-CAN protocol stack  */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * Plugin Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_PARPLUGIN_GENERIC   = 2000,    /**< This parameter can be used for communicating with the plugins */
    XCOMPAR_PARPLUGIN_LIST      = 2001,    /**< This parameters configures the plugin list used by the iNAT system */
    /**
     * -------------------------------------------------------------------------------------------------------------------------------------
     * External Sensor Parameter
     * -------------------------------------------------------------------------------------------------------------------------------------
     */
    XCOMPAR_EXTSENSOR_NMEA0183      = 2100,    /**< This parameter configures the external NMEA0183 input interface */
    XCOMPAR_EXTSENSOR_VIPS          = 2101,    /**< This parameter configures the external VIPS interface */
    XCOMPAR_EXTSENSOR_VBODYMISALIGN = 2102,    /**< This parameter defines the rotation matrix from the external sensor to the iNAT standard enclosure frame. */
    XCOMPAR_EXTSENSOR_INAT          = 2103,    /**< This parameters configures the external iNAT interface */
    XCOMPAR_INVALID = 0xFFFF
};
/**
 * XCOM Extended System Status
 *
 * Note: Once a sensor failure related to bit numbers 4-11 (Accelerometer and Gyro overrange, invalid calibration) or bit 18 (EKF error)
 * is set, it will not be reset during mission. The bit numbers 4-11 and 18 can be reset via re-alignment.
 */
typedef struct {
    uint32_t IMUINVALID : 1;           /**< IMU data are invalid. This critical error can occur if the calibration is invalid or
                                            the IMU temperature is out of range */
    uint32_t IMUCRCERROR : 1;          /**< CRC error in the internal communication */
    uint32_t IMUTIMEOUTERROR : 1;      /**< IMU timeout bit is set if the main CPU does not receive any data from the IMU PCB */
    uint32_t IMUSAMPLELOST : 1;        /**< IMU sample lost. This can be done if a CRC error occurred */
    uint32_t ACCXOR : 1;               /**< IMU’s x-accelerometer has detected an over range. */
    uint32_t ACCYOR : 1;               /**< IMU’s y-accelerometer has detected an over range. */
    uint32_t ACCZOR : 1;               /**< IMU’s z-accelerometer has detected an over range. */
    uint32_t ZUPT_REQUEST : 1;         /**< This bit is set if the system automatically requests a ZUPT */
    uint32_t OMGXOR : 1;               /**< IMU’s x-gyroscope has detected an over range. */
    uint32_t OMGYOR : 1;               /**< IMU’s y-gyroscope has detected an over range. */
    uint32_t OMGZOR : 1;               /**< IMU’s z-gyroscope has detected an over range. */
    uint32_t INVALIDCALIBRATION : 1;   /**< IMU’s calibration is invalid */
    uint32_t BITFAIL : 1;              /**< Built-In-Test failure. If this critical error occurred the data should not be used */
    uint32_t DEFCONFIG : 1;            /**< This bit is set if the system cannot find a configuration file. In this case the default
                                            configuration will be loaded */
    uint32_t GNSSINVALID : 1;          /**< GNSS solution is invalid */
    uint32_t ZUPTCALIBRUNNING : 1;     /**< Zero Velocity Update is currently calibrating */
    uint32_t GNSSCRCERR : 1;           /**< CRC error in the internal communication. */
    uint32_t GNSSTIMEOUT : 1;          /**< GNSS timeout bit is set if the main CPU does not receive any data from the GNSS receiver. */
    uint32_t EKFERROR : 1;             /**< Error inside the extended Kalman filter. If this bit is set, the solution might be invalid. */
    uint32_t EKFSAVEDPOSERR : 1;       /**< The INS could not load the last saved position for the initial alignment or
                                            the stored position is invalid. */
    uint32_t EKFSAVEDHDGERR : 1;       /**< The INS could not load the last saved heading for the initial alignment or the
                                            stored heading is invalid. */
    uint32_t MAGTIMEOUT : 1;           /**< MAG timeout bit is set if the main CPU does not receive any data from the magnetometer. */
    uint32_t MADCTIMEOUT : 1;          /**< MADC timeout bit is set if the main CPU does not receive any data from the air data computer. */
    uint32_t POS_UPDATE_REQUEST : 1;   /**< This bit is set if the system automatically requests a position update */
    uint32_t INMOTION : 1;             /**< System is in-motion */
    uint32_t ODOPLAUSIBILITYERROR : 1; /**< Odometer plausibility check failed */
    uint32_t ODOHARDWAREERROR : 1;     /**< Error inside the odometer module */
    uint32_t WAITFORFORCEDVAL : 1;     /**< The system is configured for forced position alignment. The bit is set
                                            until initial values are received */
    uint32_t PPSLOST : 1;              /**< If this bit is set the system recognized a timeout of the trigger signal. This may result
                                            to in accuracy in timestamp. */
    uint32_t LIMACCURACY : 1;          /**< Limited accuracy, due to: IMU CRC Error, IMU Timeout, Invalid calibration, EKF error */
    uint32_t RECENABLE : 1;            /**< Internal data recorder enabled/disabled */
    uint32_t FPGANOGO : 1;             /**< FPGA Error */
} XCOMSystemStatus;
typedef union {
    XCOMSystemStatus bits;
    uint32_t value;
} XCOMSystemStatusType;
/**
 * XCOM Extended System Status2
 */
typedef struct {
    uint32_t MAINTIMING_ERROR : 1;   /**< IMU maintiming error */
    uint32_t RTC_ERROR : 1;          /**< RTC time + data not invalid */
    uint32_t reserced : 30;          /**< Reserved for further use */
} XCOMSystemStatus2;
typedef union {
    XCOMSystemStatus2 bits;
    uint32_t value;
} XCOMSystemStatus2Type;
/**
 * The extended Kalman filter aiding status contains information about alignment, measurement updates and outlier detection.
 */
typedef struct {
    uint32_t POSLLH_UPDATE : 1;       /**< Position aiding with GNSS data in the current time step */
    uint32_t POSLLH_LAT_OUTLIER : 1;  /**< GNSS latitude outlier detected */
    uint32_t POSLLH_LON_OUTLIER : 1;  /**< GNSS longitude outlier detected */
    uint32_t POSLLH_ALT_OUTLIER : 1;  /**< GNSS altitude outlier detected */
    uint32_t VNED_UPDATE : 1;         /**< Velocity aiding with GNSS data in the current time step. */
    uint32_t VNED_VN_OUTLIER : 1;     /**< GNSS VNorth outlier detected */
    uint32_t VNED_VE_OUTLIER : 1;     /**< GNSS VEast outlier detected */
    uint32_t VNED_VD_OUTLIER : 1;     /**< GNSS VDown outlier detected */
    uint32_t HEIGHT_UPDATE : 1;       /**< External height aiding in the current time step. */
    uint32_t HEIGHT_OUTLIER : 1;      /**< Height outlier detected */
    uint32_t BAROHEIGHT_UPDATE : 1;   /**< Height aiding from barometer in the current time step. */
    uint32_t BAROHEIGHT_OUTLIER : 1;  /**< Baro-Altitude outlier detected */
    uint32_t VBDY_UPDATE : 1;         /**< External velocity aiding (body velocity) in the current time step. */
    uint32_t VBDY_VX_OUTLIER : 1;     /**< Vbody_x outlier detected */
    uint32_t VBDY_VY_OUTLIER : 1;     /**< Vbody_y outlier detected */
    uint32_t VBDY_VZ_OUTLIER : 1;     /**< Vbody_z outlier detected */
    uint32_t VODO_UPDATE : 1;         /**< Velocity aiding from odometer in the current time step. */
    uint32_t VODO_OUTLIER : 1;        /**< Odometer velocity outlier detected */
    uint32_t VAIR_UPDATE : 1;         /**< Velocity aiding from air data computer in the current time step. */
    uint32_t VAIR_OUTLIER : 1;        /**< Air speed outlier detected */
    uint32_t HDGT_UPDATE : 1;         /**< Heading aiding in the current time step. */
    uint32_t HDGT_OUTLIER : 1;        /**< Heading outlier detected */
    uint32_t HDGM_UPDATE : 1;         /**< Heading aiding from magnetometer in the current time step. */
    uint32_t HDGM_OUTLIER : 1;        /**< Magnetic heading outlier detected */
    uint32_t MAGFIELD_UPDATE : 1;     /**< Magnetic field aiding from magnetometer in the current time step. */
    uint32_t MAGFIELD_HX_OUTLIER : 1; /**< Magx outlier detected */
    uint32_t MAGFIELD_HY_OUTLIER : 1; /**< Magy outlier detected */
    uint32_t MAGFIELD_HZ_OUTLIER : 1; /**< Magz outlier detected */
    uint32_t PSEUDORANGE_UPDATE : 1;  /**< Pseudorange aiding from GNSS in the  current time step. */
    uint32_t RANGERATE_UPDATE : 1;    /**< Range rate aiding from GNSS in the current time step */
    uint32_t TIME_UPDATE : 1;         /**< Time update in the current time step */
    uint32_t ZUPT_UPDATE : 1;         /**< Zero velocity update in the current time step */
} XCOMEkfAidingStatLo;
typedef union {
    XCOMEkfAidingStatLo bits;
    uint32_t value;
} XCOMEkfAidingStatLoType;
typedef struct {
    uint32_t RTKLLH_UPD : 1;           /**< RTK position aiding from GNSS in the current time step */
    uint32_t RTKLLH_LON_OUTLIER : 1;   /**< RTK latitude outlier detected */
    uint32_t RTKLLH_LAT_OUTLIER : 1;   /**< RTK longitude outlier detected  */
    uint32_t RTKLLH_ALT_OUTLIER : 1;   /**< RTK altitude outlier detected  */
    uint32_t BSLXYZ_UPDATE : 1;        /**< Dual antenna aiding in the current time step */
    uint32_t BSLXYZ_OUTLIER : 1;       /**< Dual antenna outlier detected */
    uint32_t COG_UPDATE : 1;           /**< Course over ground update in the current timestep */
    uint32_t COG_OUTLIER : 1;          /**< Course over ground outlier detected */
    uint32_t TDCP_UPDATE : 1;          /**< TDCP aiding in the current time step */
    uint32_t TDCP_DD_UPDATE : 1;       /**< TDCP-DD aiding in the current time step */
    uint32_t ODO_ALONGTRK_UPDATE : 1;  /**< Odometer track aiding in the current time step */
    uint32_t ODO_ALONGTRK_OUTLIER : 1; /**< Odometer track outlier detected */
    uint32_t ODO_CONST_UPDATE : 1;     /**< Odometer constraint aiding in the current time step */
    uint32_t ODO_CONST_OUTLIER : 1;    /**< Odometer constraint outlier detected */
    uint32_t GRAV_UPDATE : 1;          /**< Gravity update */
    uint32_t GRAV_OUTLIER : 1;         /**< Gravity outlier detected */
    uint32_t EXTPOS_UPD : 1;           /**< External position update */
    uint32_t EXTPOS_OUTLIER : 1;       /**< External position outlier detected */
    uint32_t EXTVEL_UPD : 1;           /**< External velocity update */
    uint32_t EXTVEL_OUTLIER : 1;       /**< External velocity outlier detected */
    uint32_t ZARU : 1;                 /**< Zero Angular Rate Update */
    uint32_t WAHBA_POSTOOOLD : 1;      /**< Waiting for more recent position update before transition to Filter Mode 2 */
    uint32_t WAHBA_VELTOOOLD : 1;      /**< Waiting for more recent velocity update before transition to Filter Mode 2 */
    uint32_t WAHBA_INSUFFOBS : 1;      /**< Not received enough measurements yet */
    uint32_t WAHBA_UPDATE : 1;         /**< Vector direction update in coarse alignment   filter */
    uint32_t WAHBA_FILTER : 1;         /**< Coarse alignment filter active */
    uint32_t FILTERMODE_1 : 1;         /**< EKF mode 1 */
    uint32_t FILTERMODE_2 : 1;         /**< EKF mode 2 */
    uint32_t WAHBA_INTERNAL : 1;       /**< Coarse alignment filter active */
    uint32_t LEVELLING_COMPLETE : 1;   /**< Levelling has completed */
    uint32_t ALIGN_COMPLETE : 1;       /**< Stationary alignment has completed */
    uint32_t INITPOS_SET : 1;          /**< The initial position is set */
} XCOMEkfAidingStatHi;
typedef union {
    XCOMEkfAidingStatHi bits;
    uint32_t value;
} XCOMEkfAidingStatHiType;
typedef struct {
    uint32_t waiting_for_intial_values : 1;     /**< Set if the navigation module waits for user input */
    uint32_t floating_point_error : 1;          /**< Set if a floating point error occured */
    uint32_t stored_position_error : 1;         /**< Set if stored position is invalid */
    uint32_t stored_heading_error : 1;          /**< Set if stored heading is invalid */
    uint32_t verified_stored_heading_error : 1; /**< Set if verified stored heading alignment failed */
    uint32_t zupt_request : 1;                  /**< Set if zero velocity update is needed */
    uint32_t pos_update_request : 1;            /**< Set if position update is needed */
    uint32_t vms_pbit_error : 1;                /**< Set if VMS PBIT fails */
    uint32_t gyro_bias_rw_muted : 1;            /**< Set if gyro bias random walk is muted */
    uint32_t gps_clock_source : 1;              /**< Set if GPS is used as clock source for input/output data */
    uint32_t fsm_dynamic_alignment : 1;         /**< Set if alignment fsm sets dynamic alignment mode */
    uint32_t gyro_calibration_running : 1;      /**< Set if gyro calibration is running */
    uint32_t ekf_state_recovery_failed : 1;     /**< Set if EKF state recovery failed */
    uint32_t wahba_vel_timeout : 1;             /**< Set if global WAHBA velocity timeout occurred */
    uint32_t frozen_height_updates : 1;         /**< Set if frozen height updates are processed */
    uint32_t gnss_sample_rejected : 1;          /**< Set if GNSS samples are rejected caused by a transmission delay larger than the filter delay */
    uint32_t odometer_cbit_error : 1;           /**< Set if odometer CBIT fails */
    uint32_t accel_bias_rw_muted : 1;           /**< Set if accel. bias random walk is muted */
    uint32_t reserved : 14;                     /**< Reserved for further use */
} XCOMEkfExtendedStatus;
typedef union {
    XCOMEkfExtendedStatus bits;
    uint32_t value;
} XCOMEkfExtendedStatusType;
/**
 * iXCOM Response Package
 * If a message/command is received, iXCOM will send a response containing an acknowledgment or error messages. This feature should be used
 * as a handshake between user and INS.
 */
enum XCOM_ENUM_EXT XCOMResp {
    OK                          = 0,     /**< Command was received correctly */
    INVALIDPAR                  = 1,     /**< Invalid parameter of the received command */
    INVALIDCRC                  = 2,     /**< The checksum of the received package was incorrectly */
    INVALIDLOG                  = 3,     /**< The requested log message does not exist */
    INVALIDRATE                 = 4,     /**< The requested rate is invalid */
    INVALIDPORT                 = 5,     /**< The requested port is invalid */
    INVALIDCMD                  = 6,     /**< The requested command does not exist */
    INVALIDID                   = 7,     /**< The received command parameter is not valid for this INS */
    INVALIDCHANNEL              = 8,     /**< The requested iXCOM Channel ID is used by another client */
    PAROUTOFRANGE               = 9,     /**< The received value is out of range */
    LOGEXISTS                   = 10,    /**< The received message ID already exists for the used channel */
    INVALIDTRIGGER              = 11,    /**< The requested trigger is invalid for this message */
    INTERNALERROR               = 12,    /**< An internal error occurred (RAM/ROM) */
    PATHEXISTS                  = 13,    /**< The received recording path already exists */
    UARTBUFFEROVERFLOW          = 14,    /**< UART buffer overflow. iXCOM channel will be closed */
    STOREDVALUEHASNOTBEENSET    = 15,    /**< Stored value has not been set */
    MAGERRORALIGN               = 16,    /**< 3D calibration not available during alignment */
    CONFIGERROR                 = 17,    /**< Error inside configuration module */
    PARAMCANNOTBECHANGED        = 18,    /**< Parameter is read-only and cannot be changed */
    GNSSPORTALREADYINUSE        = 19,    /**< GNSS serial port is already in use */
    USERDEFINEDMESSAGE          = 20,    /**< User defined response message */
    RECORDERNOTRUNNING          = 21,    /**< Unable to start recorder modeule */
    INVALIDLENGTH               = 22,    /**< Invalid frame length */
    NOTIMPLEMENTED              = 23,    /**< Parameter/Command is not implemented */
    DEPRECATED                  = 24,    /**< Parameter is deprecated */
    INVALIDPLUGINIDX            = 25,    /**< Invalid plugin index */
    CMDREJECTED                 = 26,    /**< Command was rejected due to a power error */
    MAXLOGSLIMIT                = 27,    /**< Max number of logs per channel exceeded */
};
typedef struct XCOM_STRUCT_PACK {
    uint8_t sync;               /**< Synchronization character (is always set to 0x7E;XCOM_SYNC_BYTE) */
    uint8_t msg_id;             /**< iXCOM message ID. iXCOM distinguishes between four different types of message IDs (ordinary message IDs;
                                     command ID; parameter ID, response ID) */
    uint8_t frame_counter;      /**< The frame counter counts from 0 to 255 and starts at 0 again after it has reached 255. For messages,
                                     this can be used to detect lost messages. For parameters and commands, the system response to an
                                     input parameter or command will contain the same frame counter as the input. */
    uint8_t trigger_source;     /**< see XComLogTrigger */
    uint16_t msg_len;           /**< Length of the overall message (from sync byte to crc16 field), depends on the message log (number of bytes) */
    uint16_t gps_week;          /**< GPS week number */
    uint32_t gps_time_sec;      /**< GPS time of week - integer part in [sec] */
    uint32_t gps_time_usec;     /**< GPS time of week - fractional part in [μs] */
} XCOMHeader;
/**
 * The global status is a summarized status word that contains a combination of several status bits. This status word should be used to
 * evaluate the data integrity and is attached as part of the footer at the end of every message.
 * If an error occurs, the detailed status should be read.
 */
typedef struct {
    uint16_t HWERROR : 1;      /**< Unexpected hardware error. */
    uint16_t COMERROR : 1;     /**< Internal communication error between the individual components. */
    uint16_t NAVERROR : 1;     /**< Internal error in the navigation solution. If this bit is Critical set the INS solution should not be
                                    used. */
    uint16_t CALERROR : 1;     /**< Internal error in the calibration table. */
    uint16_t GYROVERRANGE : 1; /**< Gyro over range error. To see which axis is affected, the extended system status should be triggered. */
    uint16_t ACCOVERRAGE : 1;  /**< Acceleration over range error. To see which axis is affected, the extended system status  should be
                                    triggered. */
    uint16_t GNSSINVALID : 1;  /**< GNSS solution is invalid */
    uint16_t STANDBY : 1;      /**< System stays in standby mode. */
    uint16_t DYNAMICALIGN : 1; /**< The INS is in dynamic alignment mode */
    uint16_t TIMEINVALID : 1;  /**< The message time tag in the header represents  internal system time and has not been set from  GNSS. */
    uint16_t NAVMODE : 1;      /**< The INS has reached the fine-heading mode */
    uint16_t AHRSMODE : 1;     /**< The INS has reached the AHRS mode*/
    uint16_t ALIGNMODE : 2;    /**< Alignment mode:
                                   0: LEVELLING: This solution status is set during the levelling phase.
                                   1: STATIONARY_ALIGN: This solution status is set during stationary alignment of the INS.
                                   2: ALIGNMENT_COMPLETE: This solution status is set if the alignment has finished and the standard
                                      deviation of the estimated heading is larger than THR_HEADING (set through parameter
                                      PAREKF_HDGPOSTHR).
                                   3: HEADING_GOOD: This solution status is set if the alignment has finished and the standard deviation
                                      of the estimated heading is lower than THR_HEADING (set through parameter PAREKF_HDGPOSTHR). */
    uint16_t POSMODE : 2;      /**< Position mode:
                                   0: POS_BAD_ACCURACY: This solution status is set during alignment, if no valid position solution is
                                      available.
                                   1: POS_MEDIUM_ACCURACY: This solution status is set, if the INS has calculated a valid position
                                      solution and the standard deviation of the estimated position is lower than THR_POS_MED AND
                                      larger than THR_POS_HIGH (set through parameter PAREKF_HDGPOSTHR).
                                   2: POS_HIGH_ACCURACY: This solution status is set if the INS has calculated a valid position solution
                                      and the standard deviation of the estimated position is lower than THR_POS_HIGH (set through
                                      parameter PAREKF_HDGPOSTHR).
                                   3: POS_UNDEFINED: The position accuracy is undefined. */
} XCOMGlobalStatus;
typedef union {
    uint16_t value;
    XCOMGlobalStatus bits;
} XCOMGlobalStatusType;
/**
 * Global status alignment mode
 */
enum XCOMGlobalStatusAlignmode {
    Levelling           = 0,    /**< This solution status is set during the levelling phase */
    Aligning            = 1,    /**< This solution status is set during stationary alignment of the INS */
    AlignmentComplete   = 2,    /**< This solution status is set if the alignment has finished and the standard deviation of the estimated
                                     heading is larger than THR_HEADING (set through parameter PAREKF_HDGPOSTHR) */
    AlignmentGood       = 3     /**< This solution status is set if the alignment has finished and the standard deviation of the estimated
                                     heading is lower than THR_HEADING (set through parameter PAREKF_HDGPOSTHR) */
};
/**
 * Global status position mode
 */
enum XCOMGlobalStatusPosMode {
    PosBad          = 0,    /**< This solution status is set during alignment, if no valid position solution is available */
    PosMedium       = 1,    /**< This solution status is set, if the INS has calculated a valid position solution and the standard deviation of the
                                 estimated position is lower than THR_POS_MED AND larger than THR_POS_HIGH (set through parameter PAREKF_HDGPOSTHR) */
    PosHigh         = 2,    /**< This solution status is set if the INS has calculated a valid position solution and the standard deviation of
                                 the estimated position is lower than THR_POS_HIGH (set through parameter PAREKF_HDGPOSTHR) */
    PosUndefined    = 3    /**< The position accuracy is undefined */
};
typedef struct XCOM_STRUCT_PACK {
    union {
        /**< The global status is a summarized status word that contains a combination of several status bits. This status word
           should be used to evaluate the data integrity and is attached as part of the footer at the end of every message. If an
           error occurs, the detailed status should be read. */
        XCOMGlobalStatus bits;
        uint16_t value;
    } global_status;
    uint16_t crc16; /**< Always sent, calculated over all bytes starting with synchronization byte. */
} XCOMFooter;
/**
 * ******************************************************************************************
 * XCOM message definition
 * ******************************************************************************************
 */
typedef struct XCOM_STRUCT_PACK {
    uint8_t idx;              /**< Plugin index */
    uint8_t reserved;         /**< Reserved for further use */
    uint16_t plugin_param_id; /**< Plugin specific id */
} XCOMPluginHeader;
typedef struct XCOM_STRUCT_PACK {
    uint16_t plugin_data_id; /**< Plugin specific message id */
    uint16_t reserved;       /**< Reserved for further use */
} XCOMPluginDataHeader;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                   /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                                   /**< XCOM footer */
} XCOMGeneric;
typedef struct XCOM_STRUCT_PACK {
    uint16_t param_id;  /**< Parameter ID */
    uint8_t reserved;   /** Reserved for further use */
    uint8_t is_request; /** Requesting/Changing a parameter */
} XCOMParHeader;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                      /**< XCOM header */
    XCOMParHeader param_header;                             /**< XCOM parameter header */
    uint8_t payload_buffer[XCOM_MAX_PAR_PAYLOAD_LENGTH];    /**< XCOM payload buffer */
    XCOMFooter footer;                                      /**< XCOM footer */
} XCOMParamGeneric;
typedef struct XCOM_STRUCT_PACK {
    uint8_t port;        /**< External serial port number */
    uint8_t reserved[3]; /**< Reserved for further use */
} XCOMPassthroughHeader;
typedef struct XCOM_STRUCT_PACK {
    uint8_t device;      /**< Can device*/
    uint8_t reserved[3]; /**< Reserved for further use */
} XCOMCanGatewayHeader;
#define XCOM_MAX_CAN_PAYLOAD_LENGTH 8
typedef struct XCOM_STRUCT_PACK {
    uint8_t data[XCOM_MAX_CAN_PAYLOAD_LENGTH];
    uint8_t length;          /**< payload length */
    uint8_t is_extended_mid; /**< 1=29-bit MID, 0=11-bit MID */
    uint8_t is_remote_frame; /**< 1=remote frame request, 0=data frame */
    uint8_t reserved;        /**< reserved for further use */
    uint32_t mid;            /**< Message id */
    double timestamp;        /**< Absolute timestamps (GPS time) in [sec] */
} XCOMmsg_CanFrameType;
/**
 * This message contains different time related data, regarding system, IMU, PPS and GNSS.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;         /**< XCOM header */
    double sys_time;           /**< System time in [sec] */
    double imu_interval;       /**< IMU interval time in [sec] */
    double time_since_pps;     /**< Time since last PPS in [sec] */
    double pps_imu_time;       /**< PPS IMU time in [sec] */
    double pps_gnss_time;      /**< PPS GNSS time in [sec] */
    double gnss_bias;          /**< GNSS time bias in [sec] */
    double gnss_bias_smoothed; /**< GNSS time bias smoothed in [sec] */
    XCOMFooter footer;
} XCOMmsg_TIME;
/**
 * The IMURAW message contains the calibrated measurements from the IMU. This message is not corrected for IMU errors estimated by the EKF.
 * The IMU misalignment defined in PARIMU_MISALIGN is applied to the IMURAW log.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float acc[3];       /**< Calibrated acceleration along IMU x-, y- and z-axis in [m/s2] */
    float omg[3];       /**< Calibrated angular rate along IMU x-, y- and z-axis in [rad/s] */
    XCOMFooter footer;
} XCOMmsg_IMURAW;
/**
 * The IMUCORR message contains the calibrated measurements from the IMU. This log is corrected for IMU errors estimated by the EKF.
 * The IMU misalignment defined in PARIMU_MISALIGN is applied to the IMUCORR log.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float acc[3];       /**< Calibrated and corrected acceleration along IMU x-, y- and z-axis in [m/s2] */
    float omg[3];       /**< Calibrated and corrected angular rate along IMU x-, y- and z-axis in [rad/s] */
    XCOMFooter footer;
} XCOMmsg_IMUCORR;
/**
 * The IMUCOMP message contains the calibrated measurements from the IMU (IMUCORR), additionally compensated for earth rate and gravity.
 * The IMU misalignment defined in PARIMU_MISALIGN is applied to the IMUCOMP log.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float acc[3];       /**< Calibrated and compensated acceleration along IMU x-, y- and z-axis in [m/s2] */
    float omg[3];       /**< Calibrated and compensated angular rate along IMU x-, y- and z-axis in [rad/s] */
    XCOMFooter footer;
} XCOMmsg_IMUCOMP;
/**
 * The IMUFILTERED message contains the low-pass filtered calibrated measurements from the IMU. The IMU misalignment defined in
 * PARIMU_MISALIGN is applied to the IMUFILTERED log.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;    /**< XCOM header */
    float acc[3];         /**< Low-pass filtered acceleration along IMU x-, y- and z-axis in [m/s2] */
    float omg[3];         /**< Low-pass filtered angular rate along IMU x-, y- and z-axis in [rad/s] */
    XCOMFooter footer;
} XCOMmsg_IMUFILTERED;
/**
 * The OMGINT message contains the integrated angular rates for each axis separately.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;      /**< XCOM header */
    float omg_int[3];       /**< Integrated angular rate for x-, y- and z-axis in [rad] */
    float integration_time; /**< Integration time in [sec] */
    XCOMFooter footer;
} XCOMmsg_OMGINT;
/**
 * This message contains system raw data used for post-processing.
 * #domain: public
 * #rate: full
 * #name: XCOMmsg_RAWDATA
 */
#define XCOMMSG_RAWDATA_MAXODOMETERS    3   /**< Number of supported odometers */
#define XCOMMSG_RAWDATA_IMUAXIS_ACCX    0   /**< IMU temperature field contains ACCX temperature measurement */
#define XCOMMSG_RAWDATA_IMUAXIS_ACCY    1   /**< IMU temperature field contains ACCY temperature measurement */
#define XCOMMSG_RAWDATA_IMUAXIS_ACCZ    2   /**< IMU temperature field contains ACCZ temperature measurement */
#define XCOMMSG_RAWDATA_IMUAXIS_OMGX    3   /**< IMU temperature field contains OMGX temperature measurement */
#define XCOMMSG_RAWDATA_IMUAXIS_OMGY    4   /**< IMU temperature field contains OMGY temperature measurement */
#define XCOMMSG_RAWDATA_IMUAXIS_OMGZ    5   /**< IMU temperature field contains OMGZ temperature measurement */
typedef struct XCOM_STRUCT_PACK {
    int32_t odo_ticks;              /**< Odometer / Timing data, up/down counter of odometer pulses which will never be reset */
    uint32_t odo_trig_event;        /**< Odometer / Timing data, ticks elapsed until first odometer event [1 tick = 25 ns] */
    uint32_t odo_trig_next_event;   /**< Odometer / Timing data, ticks from last odometer event until the end of interval [1 tick = 25 ns] */
} XCOMRawOdometer;
typedef struct {
    uint8_t pps_event:1;
    uint8_t reserved:7;
} XCOMRawEventType;
typedef union {
    XCOMRawEventType bits;
    uint8_t value;
} RAWDATAEVENTS;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    float acc[3];                   /**< Calibrated Accelerometer output for x-, y- and z-axis in m/s2 */
    float omg[3];                   /**< Calibrated Gyroscope output for x-, y- and z-axis in rad/s */
    uint32_t odometer_interval;     /**< Measured odometer interval [1 tick = 25 ns] */
    XCOMRawOdometer odometer[XCOMMSG_RAWDATA_MAXODOMETERS]; /**< Odometer / Timing data */
    uint32_t sys_status;            /**< System Status  */
    uint32_t fpga_status;           /**< FPGA Status  */
    float imu_temperature;          /**< IMU temperature in °C  */
    uint8_t imu_temperature_axis;   /**< IMU temperature axis identifier (see XCOMMSG_RAWDATA_IMUAXIS_* in appendix)  */
    RAWDATAEVENTS events;           /**< see XCOMRawEventType */
    uint16_t rr_addr;               /**< Round robin address field */
    uint16_t rr_data[2];            /**< Content of round robin register (PCB temperatures + ADC measurements) */
    uint32_t user_event[2];         /**< Event timer [1 tick = 25 ns] */
    XCOMFooter footer;
} XCOMmsg_RAWDATA;
/**
 * This message contains the integrated solution of filter. This log includes the inertial data, integrated position, velocity and
 * attitude and status information in one package, instead of using four logs (IMUCOMP, INSRPY, INSPOSXXX and INSVELXXX).
 * The advantage of using this log is that the protocol overhead will be reduced (only one header has to be transmitted together with
 * all contained data).
 * #name: XCOMmsg_INSSOL
 * #domain: public
 * #rate: full
 */
enum XCOMINSSOL_DatSelBits {
    XCOM_INSSOL_IMURAW          = 0,    /**< IMU data from IMURAW */
    XCOM_INSSOL_IMUCORR         = 1,    /**< IMU data from IMUCORR */
    XCOM_INSSOL_IMUCOMP         = 2,    /**< IMU data from IMUCOMP */
    XCOM_INSSOL_VELNED          = 3,    /**< Field 5 contains VELNED data */
    XCOM_INSSOL_VELENU          = 4,    /**< Field 5 contains VELENU data */
    XCOM_INSSOL_VELECEF         = 5,    /**< Field 5 contains VELECEF data */
    XCOM_INSSOL_VELBODY         = 6,    /**< Field 5 contains VELBODY data */
    XCOM_INSSOL_ALTITUDE        = 7,    /**< Field 7 contains a WGS84 altitude */
    XCOM_INSSOL_MEAN_SEA_LEVEL  = 8,    /**< Field 7 contains MSL */
    XCOM_INSSOL_BARO_ALTITUDE   = 9,    /**< Field 7 contains a barometric altitude */
    XCOM_INSSOL_WGS84_POSITION  = 10,   /**< Field 6 contains a WGS84 position */
    XCOM_INSSOL_ECEF_POSITION   = 11,   /**< Field 6 contains a ECEF position */
    XCOM_INSSOL_RPY_OUTPUT      = 15    /**< RPY output mode (used for DIN70.000 frame) */
};
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;       /**< XCOM header */
    float accel[3];          /**< Calibrated acceleration along IMU x-, y-, and z-axis in [m/s2]. */
    float gyro[3];           /**< Calibrated angular rate along IMU x-, y-, and z-axis in [rad/s]. */
    float rpy[3];            /**< Roll, pitch and yaw in [rad]. */
    float vel[3];            /**< Velocity along x-, y- and z-axis in [m/s]. The reference  frame can be selected via the PARDAT_VEL
                                  parameter (ENU, NED,  ECEF or body frame). */
    double pos[2];           /**< Longitude and latitude in [rad]. The selected values are masked in the data_selection field. */
    float altitude;          /**< Altitude/Height in [m]. The content (WGS or MSL or Baro based) can be selected via the PARDAT_POS
                                  parameter. The selected value is masked in the data_selection field. */
    int16_t undulation;      /**< Relationship between the geoid and the ellipsoid in [cm]. */
    uint16_t data_selection; /**< Data selection mask. This field contains the selected data sources for this
                                  message (see XCOMINSSOL_DatSel in appendix)  */
    XCOMFooter footer;
} XCOMmsg_INSSOL;
/**
 * This message contains the integrated solution of filter in ECEF frame. This log includes the inertial data, integrated position, velocity
 * and attitude and status information in one package, instead of using four logs (IMUCOMP, INSRPY, INSPOSXXX and INSVELXXX). The advantage
 * of using this log is that the protocol overhead will be reduced (only one header has to be transmitted together with all contained data).
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float acc[3];       /**< Calibrated acceleration along IMU x-, y-, and z-axis in [m/s2]. */
    float omg[3];       /**< Calibrated angular rate along IMU x-, y-, and z-axis in [rad/s]. */
    double pos_ecef[3]; /**< Estimated position along ECEF x-, y- and z-coordinate in [m] */
    float vel_ecef[3];  /**< Estimated velocity along ECEF x-, y- and z-coordinate  in [m/s] */
    float q_nb[4];      /**< Body frame to NED quaternion */
    XCOMFooter footer;
} XCOMmsg_INSSOLECEF;
/**
 * This message contains the load-factor.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;      /**< XCOM header */
    float load_factor[3];   /**< Load factor x, y and z (unitless) */
    XCOMFooter footer;
} XCOMmsg_LOADFACTOR;
/**
 * This message contains information about the ground speed
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float gnd_speed;   /**< INS speed over ground in [m/s] */
    float track_angle; /**< NS course over ground in [rad] */
    float v_down;      /**< INS down velocity in [m/s] */
    XCOMFooter footer;
} XCOMmsg_INSGNDSPEED;
/**
 * This message contains the angular acceleration
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float omg_dot[3];   /**< Angular acceleration around x, y and z in [rad/s2] */
    XCOMFooter footer;
} XCOMmsg_OMGDOT;
/**
 * This message contains the measured acceleration in ACV
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float acc[3];       /**< Measured acceleration in ACV (along-track, cross-track, vertical) frame in [m/s2] */
    XCOMFooter footer;
} XCOMmsg_INSTRACKACC;
/**
 * This message contains the INS true heading converted to magnetic heading
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;        /**< XCOM header */
    float magnetic_heading;   /**< INS true heading converted to magnetic heading [rad] */
    float magnetic_cog;       /**< Course over ground [rad] */
    float magnetic_deviation; /**< Magnetic deviation [rad] */
    XCOMFooter footer;
} XCOMmsg_INSMAGHDG;
/**
 * The SYS_STAT message contains the extended system status. The content of this log can be configured via the PARDAT_SYSSTAT parameter.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    uint32_t mode;      /**< This field contains the configuration of the payload */
    union {             /**< System status information */
        XCOMSystemStatus bits;
        uint32_t value;
    } sysstat;
    XCOMFooter footer;
} XCOMmsg_SYSSTAT;
/**
 * This message contains the extended status of the internal Kalman Filter
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                          /**< XCOM header */
    XCOMEkfExtendedStatusType ext_status;      /**< This field contains extended filter status */
    uint32_t reserved[4];
    XCOMFooter footer;
} XCOMmsg_EKFSTATEXT;
/**
 * This output message includes the inertial data, integrated position, velocity and attitude and status information.
 * The IMU misalignment defined in PARIMU_MISALIGN is applied to the IPVAS log.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    float heading;                          /**< Heading reference to North, clockwise: 0 to 2π in [rad]*/
    uint32_t utc_year;                      /**< UTC year */
    uint8_t utc_month;                      /**< UTC month (1-12) */
    uint8_t utc_day;                        /**< UTC day (1-31) */
    uint8_t utc_hour;                       /**< UTC hour (0-23) */
    uint8_t utc_min;                        /**< UTC minute (0-59) */
    uint32_t utc_ms;                        /**< UTC millisecond (0-60999): Maximum of 60999 when leap second is applied. */
    double runtime;                         /**< Run time of INS starting from zero in [sec] */
    double latitude;                        /**< INS latitude (±π) in [rad] */
    double longitude;                       /**< INS longitude (±π/2) in [rad] */
    float height_msl;                       /**< INS height above sea level (WGS84) in [m] */
    float altitude;                         /**< INS altitude above WGS84 ellipsoid in [m] */
    float rpy[3];                           /**< Roll, pitch and yaw in [rad] */
    float climb_rate;                       /**< INS climb rate in [m/s] */
    float speed;                            /**< INS total speed in [m/s] */
    float v_ned[3];                         /**< INS north, east and down velocity in [m/s] */
    float v_vehicle[3];                     /**< INS x, y and z velocity in vehicle frame in [m/s] */
    float rpy_rate[3];                      /**< INS Roll-, pitch- and yaw-rate in [rad/s] */
    float omg_vehicle[3];                   /**< Calibrated angular rate along vehicle x-, y- and z-axis in [rad/s].  */
    float acc_ned[3];                       /**< Calibrated acceleration in NED frame in [m/s2] */
    float acc_vehicle[3];                   /**< Calibrated acceleration along vehicle x-, y- and z-axis in [m/s2] */
    XCOMFooter footer;                      /**< XCOM footer */
} XCOMmsg_IPVAS;
/**
 * This message contains the insights of the position averaging module
 * #domain: public
 * #rate: full
 * #name: XCOMmsg_POSAVE
 */
#define POSAVE_AVGSTATUS_OFF      0 /**< Position averaging is turned off */
#define POSAVE_AVGSTATUS_RUNNING  1 /**< Position averaging is in progress and not yet finished */
#define POSAVE_AVGSTATUS_COMPLETE 2 /**< Position averaging process completed */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    uint32_t status;                        /**< Status of the position averaging process */
    uint32_t number_of_samples;             /**< Number of samples in the average */
    double pos[3];                          /**< Averaged position (lat, lon and alt) in [rad] or [m] */
    double stddev[3];                       /**< Standard deviation of averaged position (lat, lon and alt) in [m] */
    double min[3];                          /**< Minimum of averaged position (lat, lon and alt) in [rad] or [m] */
    double max[3];                          /**< Maximum of averaged position (lat, lon and alt) in [rad] or [m] */
    double runtime;                         /**< Elapsed time of averaging in [s] */
    XCOMFooter footer;
} XCOMmsg_POSAVE;
/**
 * The INSRPY message contains the integration filter attitude solution in Euler representation (roll, pitch and yaw).
 * The given Euler angles describe the orientation of the body frame with respect to the navigation frame.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;   /**< XCOM header */
    float rpy[3];        /**< Roll, pitch and yaw in [rad] */
    XCOMFooter footer;
} XCOMmsg_INSRPY;
/**
 * This message contains the integration filter’s orientation solution as direction cosine matrix(DCM)
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float c_nb[9];      /**< Direction Cosine Matrix coefficients */
    XCOMFooter footer;
} XCOMmsg_INSDCM;
/**
 * This message contains the integration filter’s orientation solution. The quaternion provides a redundant, non singular attitude
 * representation that is well suited for describing arbitrary and large rotations.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float q_nb[4];      /**< Body frame to NED quaternion */
    XCOMFooter footer;
} XCOMmsg_INSQUAT;
/**
 * This message contains the position in WGS84 geodetic coordinates.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    double longitude;   /**< Longitude (WGS84) in [rad] */
    double latitude;    /**< Latitude (WGS84) in [rad] */
    float altitude;     /**< Height in [m] */
    XCOMFooter footer;
} XCOMmsg_INSPOSLLH;
/**
 * This message contains the position in UTM coordinates.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    int32_t zone;      /**< UTM zone number */
    double easting;    /**< UTM east position in [m] */
    double northing;   /**< UTM north position in [m] */
    float height;      /**< Height in [m] */
    double grid_north; /**< UTM grid north in [rad] */
    XCOMFooter footer;
} XCOMmsg_INSPOSUTM;
/**
 * INS/GNSS position solution in Military Grid Reference System (MGRS).
 * #domain: public
 * #rate: full
 * #name: XCOMmsg_INSMGRS
 */
#define INSMGRS_STRING_LEN 64
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;             /**< XCOM header */
    uint32_t error_code;           /**< 0: No error */
    uint8_t mgrs[INSMGRS_STRING_LEN]; /**< MGRS coordinate string */
    double grid_north;             /**< MGRS grid north in [rad] */
    XCOMFooter footer;
} XCOMmsg_INSMGRS;
/**
 * INS/GNSS roll-, pitch- and yaw-rate
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;           /**< XCOM header */
    double rpy_rate[3];          /**< Roll-, pitch- and yaw-rate in [rad/s] */
    XCOMFooter footer;
} XCOMmsg_INSRPYRATE;
/**
 * INS/GNSS roll-, pitch- and yaw-acceleration
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
  XCOMHeader header;           /**< XCOM header */
  double rpy_accel[3];          /**< Roll-, pitch- and yaw-acceleration in [rad/s2] */
  XCOMFooter footer;
} XCOMmsg_INSRPYACCEL;
/**
 * INS/GNSS acceleration in NED frame
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    double acc_ned[3];          /**< Acceleration in NED frame [m/s/s] */
    XCOMFooter footer;
} XCOMmsg_ACCNED;
/**
 * INS/GNSS jerk in NED frame
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    double jerk_ned[3];         /**< Jerk in NED frame [m/s/s/s] */
    XCOMFooter footer;
} XCOMmsg_JERKNED;
/**
 * INS/GNSS wander azimuth
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    double wander_azimuth;      /**< Wander azimuth in [rad] */
    double reserved[2];         /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_INSWANDERAZIMUTH;
/**
 * Unix time of the current inertial epoch
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;             /**< XCOM header */
    uint32_t unix_time_sec;        /**< Unix time - integer part in [sec] */
    uint32_t unix_time_usec;       /**< Unix time - fractional part in [μs] */
    uint32_t reserved[2];          /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_UNIXTIME;
/**
 * The INSPOSECEF message contains the position in ECEF coordinates.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    double pos_ecef[3]; /**< ECEF x-, y- and z-coordinate in [m] */
    XCOMFooter footer;
} XCOMmsg_INSPOSECEF;
/**
 * The INSVELBODY message contains the estimated velocity vector of the INS with respect to earth coordinated in the body frame.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float velocity[3];  /**< Estimated velocity along body’s x-/y-/z-axis in [m/s] */
    XCOMFooter footer;  /**< XCOM footer */
} XCOMmsg_INSVELBODY;
/**
 * The INSVELENU message contains the estimated velocity vector of the INS with respect to earth coordinated in the ENU frame.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float velocity[3];  /**< Estimated ENU velocity in [m/s] */
    XCOMFooter footer;  /**< XCOM footer */
} XCOMmsg_INSVELENU;
/**
 * The INSVELNED message contains the estimated velocity vector of the INS with respect to earth coordinated in the NED frame.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float velocity[3];  /**< Estimated NED velocity in [m/s] */
    XCOMFooter footer;  /**< XCOM footer */
} XCOMmsg_INSVELNED;
/**
 * The INSVELECEF message contains the estimated velocity vector of the INS with respect to earth coordinated in the ECEF frame.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float velocity[3];  /**< Estimated velocity along ECEF x-/y-/z-axis [m/s] */
    XCOMFooter footer;  /**< XCOM footer */
} XCOMmsg_INSVELECEF;
/**
 * The MAGDATA message contains the magnetometer measurements. The Extended Kalman Filter is able to process magnetic heading as well as the
 * magnetic field vector.
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float field[3];     /**< Measured magnetic field along sensors x-, y- and z-axis in [nT] */
    float heading;      /**< Obtained magnetic heading in [rad] */
    float bank;         /**< Magnetometer bank in [rad] */
    float elevation;    /**< Magnetometer elevation in [rad] */
    float deviation;    /**< Magnetic deviation in [rad] */
    uint32_t status;    /**< Magnetometer status */
    XCOMFooter footer;
} XCOMmsg_MAGDATA;
/**
 * The MAGDATA2 message contains the non-calibrated magnetic raw measurements
 * #domain: public
 * #rate: event
 * #name: XCOMmsg_MAGDATA2
 */
#define XCOMMSG_MAGDATA2_TEMPIDX_ADT7320_INT    0               /**< Internal temperature sensor */
#define XCOMMSG_MAGDATA2_TEMPIDX_ADT7320_EXT    1               /**< External temperature sensor */
#define XCOMMSG_MAGDATA2_TEMPSCALE              (1.0/128.0)     /**< Temperature scale factor */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    float raw_values[3];        /**< Field values in [nT] */
    uint8_t bit_error;          /**< Built-In Self Test result (0: ok - 1: error) */
    uint8_t temperature_idx;    /**< Temperature index */
    int16_t temperature;        /**< Temperature in [°C] */
    XCOMFooter footer;
} XCOMmsg_MAGDATA2;
/**
 * The AIRDATA message contains the Micro Air Data Computer (iMADC) measurements. This log is only available with iMAR’s iMADC.
 * The iMADC must be connected via CAN (ARINC825) to the IMS. The maximum data rate of the AIRDATA log is limited to 50 Hz.
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    float tas;                          /**< True air speed in [m/s] */
    float ias;                          /**< Indicated air speed in [m/s] */
    float baro_altitude;                /**< Barometric pressure altitude in [m] */
    float baro_altitude_rate;           /**< Barometric altitude rate (rate of climb) in [m/s] */
    float pd;                           /**< Dynamic pressure in [hPa] */
    float ps;                           /**< Static pressure in [hPa] */
    float oat;                          /**< Outside air temperature in [◦C] */
    float estimated_bias;               /**< Estimated bias in [m] */
    float estimated_scalefactor;        /**< Estimated scale factor */
    float estimated_bias_stddev;        /**< Estimated bias standard deviation in [m] */
    float estimated_scalefactor_stddev; /**< Estimated scale factor standard deviation */
    uint32_t status;                    /**< see airDataComputerStatus */
    XCOMFooter footer;
} XCOMmsg_AIRDATA;
/**
 * The EKFSTDDEV message contains the standard deviation of the estimated position, velocity, attitude, heading and sensor errors.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float pos[3];      /**< Standard deviation of estimated easting, northing and height in [m] */
    float vel[3];      /**< Standard deviation of estimated north, east and down velocity in [m/s] */
    float tilt[3];     /**< Standard deviation of estimated tilt around north, east and down axis in [rad]. Down axis tilt stddev is equivalent
                            to yaw angle stddev. */
    float bias_acc[3]; /**< Standard deviation of estimated acceleration bias along IMU x-, y- and z-axis in [m/s2 ] */
    float bias_omg[3]; /**< Standard deviation of estimated angular rate bias along IMU x-, y- and z-axis in [rad/s] */
    float scf_acc[3];  /**< Standard deviation of estimated acceleration scale  factor along IMU x-, y- and z-axis */
    float scf_omg[3];  /**< Standard deviation of estimated angular rate scale  factor along IMU x-, y- and z-axis */
    float scf_odo;     /**< Standard deviation of the estimated odometer scale factor */
    XCOMFooter footer;
} XCOMmsg_EKFSTDDEV;
/**
 * The EKFSTDDEV2 message contains the estimated sensor errors as well as the current standard deviations of position parameters and of
 * orientation angles (RPY) by the extended Kalman Filter. Furthermore, it displays the accuracy of other estimated values.
 * In addition to the standard deviations reported in the previous version of this log, estimated sensor nonorthogonality and
 * misalignment standard deviations are included in this log.
 *
 * For the standard deviation of the position:
 * The accuracy is given in meters. No special resolution and no special value range are used, this is only limited by the IEEE float
 * format. The float value corresponds to the 1 sigma interval (in meters) supplied by the Kalman Filter.
 *
 * For the standard deviation of the orientation angles (RPY):
 * The float value corresponds to the 1 sigma interval (in rad) supplied by the Kalman Filter. The value range is −π to +π.
 * The resolution is only limited by the float data type. Additionally, there is a pre-defined sequence to follow when using RPY: first, the
 * rotation is around the z-axis, then around the (already rotated) y-axis, finally around the x-axis. If you need further help on this
 * topic, please contact iMAR support (support@imar-navigation.de).
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float pos[3];      /**< Standard deviation of estimated longitude, latitude and  height in [m] */
    float vel[3];      /**< Standard deviation of estimated north, east and down velocity in [m/s] */
    float tilt[3];     /**< Standard deviation of estimated tilt around north, east and down axis in [rad] */
    float bias_acc[3]; /**< Standard deviation of estimated acceleration bias along IMU x-, y- and z-axis in [m/s2 ] */
    float bias_omg[3]; /**< Standard deviation of estimated angular rate bias along IMU x-, y- and z-axis in [rad/s] */
    float ma_acc[9];   /**< Standard deviation of estimated acceleration misalignment along IMU x-, y- and z-axis in [rad] */
    float ma_omg[9];   /**< Standard deviation of estimated angular rate misalignment along IMU x-, y- and z-axis in [rad] */
    float scf_odo;     /**< Standard deviation of the estimated odometer scale factor */
    float ma_odo[2];   /**< Standard deviation of estimated odometer misalignment around first and second axis perpendicular to odometer
                            direction [rad] */
    XCOMFooter footer;
} XCOMmsg_EKFSTDDEV2;
/**
 * The EKFSTDDEV3 message contains the uncertainties of the estimated sensor errors as well as the current standard deviations of position
 * parameters and of orientation angles (RPY) by the extended Kalman Filter. Furthermore, it displays the accuracy of other estimated values.
 * In addition to the standard deviations reported in the previous version of this log, estimated sensor nonorthogonality and
 * misalignment standard deviations are included in this log.
 *
 * For the standard deviation of the position:
 * The accuracy is given in meters. No special resolution and no special value range are used, this is only limited by the IEEE float
 * format. The float value corresponds to the 1 sigma interval (in meters) supplied by the Kalman Filter.
 *
 * For the standard deviation of the orientation angles (RPY):
 * The float value corresponds to the 1 sigma interval (in rad) supplied by the Kalman Filter. The value range is −π to +π.
 * The resolution is only limited by the float data type. Additionally, there is a pre-defined sequence to follow when using RPY: first, the
 * rotation is around the z-axis, then around the (already rotated) y-axis, finally around the x-axis. If you need further help on this
 * topic, please contact iMAR support (support@imar-navigation.de).
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float pos[3];      /**< Standard deviation of estimated longitude, latitude and  height in [m] */
    float vel_ned[3];  /**< Standard deviation of estimated north, east and down velocity in [m/s] */
    float tilt[3];     /**< Standard deviation of estimated tilt around north, east and down axis in [rad] */
    float bias_acc[3]; /**< Standard deviation of estimated acceleration bias along IMU x-, y- and z-axis in [m/s2 ] */
    float bias_omg[3]; /**< Standard deviation of estimated angular rate bias along IMU x-, y- and z-axis in [rad/s] */
    float ma_acc[9];   /**< Standard deviation of estimated acceleration misalignment along IMU x-, y- and z-axis in [rad] */
    float ma_omg[9];   /**< Standard deviation of estimated angular rate misalignment along IMU x-, y- and z-axis in [rad] */
    float scf_odo;     /**< Standard deviation of the estimated odometer scale factor */
    float ma_odo[2];   /**< Standard deviation of estimated odometer misalignment around first and second axis perpendicular to odometer
                            direction in [rad] */
    float vel_body[3]; /**< Standard deviation of estimated x, y, z body velocity in [m/s] */
    XCOMFooter footer;
} XCOMmsg_EKFSTDDEV3;
/**
 * This message contains the sensor uncertainties estimated by the extended Kalman filter in the ECEF frame. In addition to the standard
 * deviations reported in the EKFSTDDEV3, estimated sensor nonorthogonality and misalignment standard deviations are included in this log.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float pos[3];      /**< Standard deviation of estimated ECEF x, y and z in [m] */
    float vel[3];      /**< Standard deviation of estimated ECEF x, y and z velocity in  [m/s] */
    float tilt[3];     /**< Standard deviation of estimated tilt around north, east and down axis in [rad] */
    float bias_acc[3]; /**< Standard deviation of estimated acceleration bias along IMU x-, y- and z-axis in [m/s2] */
    float bias_omg[3]; /**< Standard deviation of estimated angular rate bias along IMU x-, y- and z-axis in [rad/s] */
    float ma_acc[9];   /**< Standard deviation of estimated acceleration misalignment along IMU x-, y- and z-axis in [rad] */
    float ma_omg[9];   /**< Standard deviation of estimated angular rate misalignment along IMU x-, y- and z-axis in [rad] */
    float scf_odo;     /**< Standard deviation of the estimated odometer scale factor */
    float ma_odo[2];   /**< Standard deviation of estimated odometer misalignment around first and second axis perpendicular to odometer
                            direction in [rad] */
    XCOMFooter footer;
} XCOMmsg_EKFSTDDEVECEF;
/**
 * This parameter contains the 3x3 position covariance matrix.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float covar[9];    /**< Position covariance matrix */
    XCOMFooter footer;
} XCOMmsg_EKFPOSCOVAR;
/**
 * The EKFERROR message contains the estimated sensor errors by the extended Kalman filter.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;     /**< XCOM header */
    float bias_acc[3];     /**< Estimated bias of acceleration along IMU x-, y- and z-axis in [m/s2] */
    float bias_omg[3];     /**< Estimated bias of angular rate along IMU x-, y- and z-axis in [rad/s] */
    float scf_acc[3];      /**< Estimated scale factor of acceleration along IMU x-, y- and z-axis */
    float scf_omg[3];      /**< Estimated scale factor of angular rate along IMU x-, y- and z-axis */
    float scf_odo;         /**< Estimated odometer scale factor */
    float odo_misalign[2]; /**< Estimated odometer misalignment around first and second axis perpendicular to odometer direction in [rad] */
    XCOMFooter footer;
} XCOMmsg_EKFERROR;
/**
 * The EKFERROR2 message contains the sensor uncertainties estimated by the extended Kalman filter. In addition to the sensor errors reported
 * in the EKFERROR2, estimated sensor non orthogonalities and misalignments are included in this log.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float bias_acc[3]; /**< Estimated bias of acceleration along IMU x-, y- and z-axis in [m/s2] */
    float bias_omg[3]; /**< Estimated bias of angular rate along IMU x-, y- and z-axis in [rad/s] */
    float ma_acc[9];   /**< Accelerometer misalignment matrix */
    float ma_omg[9];   /**< Gyro misalignment matrix */
    float scf_odo;     /**< Estimated odometer scale factor */
    float ma_odo[2];   /**< Estimated odometer misalignment around first and second axis perpendicular to odometer direction in [rad] */
    XCOMFooter footer;
} XCOMmsg_EKFERROR2;
/**
 * This message contains the result of iMAR's tightly coupled solution.
 * #domain: hidden
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    uint8_t sats_available_psr;
    uint8_t sats_used_psr;
    uint8_t satsAvailableRR;
    uint8_t sats_used_rr;
    uint8_t sats_available_tdcp;
    uint8_t sats_used_tdcp;
    uint8_t ref_sat_tdcp;
    uint8_t reserved;
    uint32_t used_sats_psr_gps;
    uint32_t outlier_sats_psr_gps;
    uint32_t used_sats_psr_glonass;
    uint32_t outlier_sats_psr_glonass;
    uint32_t used_sats_rr_gps;
    uint32_t outlier_sats_rr_gps;
    uint32_t used_sats_rr_glonass;
    uint32_t outlier_sats_rr_glonass;
    uint32_t used_sats_tdcp_gps;
    uint32_t outlier_sats_tdcp_gps;
    uint32_t used_sats_tdcp_glonass;
    uint32_t outlier_sats_tdcp_glonass;
    XCOMFooter footer;
} XCOMmsg_EKFTIGHTLY;
/**
 * This message contains the voltage and current measurements of the INS.
 * #domain: public
 * #rate: event
 * #name: XCOMmsg_POWER
 */
#define XCOMMSG_POWER_CHANNELS 32
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    float power[XCOMMSG_POWER_CHANNELS];    /**< Measured voltages in [V] */
    XCOMFooter footer;
} XCOMmsg_POWER;
/**
 * This message contains a system dependent float array, which holds the temperatures of the internal components such as sensors, PCB and
 * CPU. The data rate of this log is limited to 1 Hz.
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                  /**< XCOM header */
    float temperature[XCOM_MAX_NUMBER_OF_TEMPERATURES]; /**< IMU dependent internal temperature array */
    XCOMFooter footer;
} XCOMmsg_TEMPERATURE;
/**
 * This message contains the status of an external connect ADC24 card
 * #domain: hidden
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;    /**< XCOM header */
    uint32_t rr_idx[4];   /**< Round robin index field */
    uint32_t rr_value[4]; /**< Round robin data field */
    XCOMFooter footer;
} XCOMmsg_ADC24STATUS;
/**
 * This message contains the measurements of an external connect ADC24 card
 * #domain: hidden
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;           /**< XCOM header */
    int32_t acc[3];              /**< ADC24 measurements */
    uint16_t frame_counter;      /**< ADC24 frame counter */
    int16_t temperature;         /**< ADC24 board temperature */
    uint8_t error_status;        /**< ADC24 error status  */
    uint8_t interval_counter[3]; /**< ADC24 inerval counter */
    XCOMFooter footer;
} XCOMmsg_ADC24DATA;
/**
 * This message contains the status of ARINC429 information
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    uint32_t status;    /**< ARINC429 status information */
    XCOMFooter footer;
} XCOMmsg_ARINC429STAT;
/**
 * This message contains status information of the internal Ntrip client.
 * #name: XCOMmsg_NTRIPSTATUS
 */
 #define XCOMMSG_NTRIPSTATUS_BUFFERSIZE 256
#define XCOMMSG_NTRIPSTATUS_ERRCODE_CANTRESOLVEPORT             (1 << 0)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_SERVERNAMELOOKUPFAIL        (1 << 1)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_REQDATATOOLONG              (1 << 2)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_LOGINDATATOOLONG            (1 << 3)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_BINDERROR                   (1 << 4)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_CONNECTERROR                (1 << 5)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_SENDERROR                   (1 << 6)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_SESSIONNUMEXTRACTERR        (1 << 7)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_SRCTABLECHECKFAILED         (1 << 8)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_CONTENTLENEXTRACTERR        (1 << 9)    /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_REQUESTDATAERROR            (1 << 10)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_SELECTERROR                 (1 << 11)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_ILLEGALUDPDATA              (1 << 12)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_ILLEGALUDPHEADER            (1 << 13)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_CONNECTIONCLOSED            (1 << 14)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_RECEIVEERROR                (1 << 15)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_PORTERROR                   (1 << 16)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_INVALIDSESSIONNUMBER        (1 << 17)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_DATASTREAMERROR             (1 << 18)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_CONNECTIONFAILED            (1 << 19)   /**< */
#define XCOMMSG_NTRIPSTATUS_ERRCODE_SERIALERROR                 (1 << 20)   /**< */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                  /**< XCOM header */
    uint32_t status;                                    /**< Status information code (see XCOMMSG_NTRIPSTATUS_ERRCODE_* in appendix) */
    int8_t last_msg[XCOMMSG_NTRIPSTATUS_BUFFERSIZE];    /**< Recent status message */
    uint32_t error_counter;                             /**< Error counter */
    XCOMFooter footer;
} XCOMmsg_NTRIPSTATUS;
/**
 * This message contains the measurements of the MVC slaves
 * #domain: hidden
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;   /**< XCOM header */
    double pos_ecef[3];  /**< Estimated position along ECEF x-, y- and z-axis in [m] */
    double vel_ecef[3];  /**< Estimated velocity along ECEF x-, y- and z-axis in [m/s] */
    double q_nb[4];      /**< Body frame to NED quaternion */
    float acc[3];        /**< Calibrated acceleration along IMU x-, y- and z-axis in [m/s2] */
    float omg[3];        /**< Calibrated angular rate along IMU x-, y- and z-axis in [rad/s] */
    float pos_stddev[3]; /**< Standard deviation of estimated easting, northing and height in [m] */
    float vel_stddev[3]; /**< Standard deviation of estimated north, east and down velocity in [m/s] */
    float att_stddev[3]; /**< Standard deviation of tilt around north, east, and down axis. Down axis tilt stddev.
                              is equivalent to yaw angle stddev. [rad] */
    XCOMFooter footer;
} XCOMmsg_MVCSLAVE;
/**
 * This message contains the MVC data
 * #domain: hidden
 * #rate: event
 * #name: XCOMmsg_MVCDATA
 */
typedef struct XCOM_STRUCT_PACK {
    double gpsTime;        /**< GPS time of week in [s] */
    double pos_ecef[3];    /**< Estimated position along ECEF x-, y- and z-axis in [m] */
    double vel_ecef[3];    /**< Estimated velocity along ECEF x-, y- and z-axis in [m/s] */
    double q_nb[4];        /**< Body frame to NED quaternion */
    float acc[3];          /**< Calibrated acceleration along IMU x-, y- and z-axis in [m/s2] */
    float omg[3];          /**< Calibrated angular rate along IMU x-, y- and z-axis in [rad/s] */
    float pos_std_dev[3];  /**< Standard deviation of estimated easting, northing  and height in [m] */
    float vel_std_dev[3];  /**< Standard deviation of estimated north, east and  down velocity in [m/s] */
    float att_std_dev[3];  /**< Standard deviation of tilt around  north, east, and down axis. Down axis
                                tilt stddev. is equivalent to yaw angle stddev. [rad] */
    XCOMGlobalStatusType globalStatus; /**< Global system status */
    uint16_t ttc;          /**< Time to closest encounter */
} XCOMMvcDataType;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;      /**< XCOM header */
    int32_t slaveIdx;       /**< Slave index */
    XCOMMvcDataType master; /**< Master data */
    XCOMMvcDataType slave;  /**< Slave data */
    XCOMFooter footer;
} XCOMmsg_MVCDATA;
/**
 * This message contains the transmission latency between master and slave devices.
 * #domain: hidden
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;         /**< XCOM header */
    double tx_time;            /**< GPS time */
    double pos_ecef[3];        /**< Reference position in ECEF frame [m] */
    XCOMFooter footer;
} XCOMmsg_USR_LATENCYTX;
/**
 * This message contains the CSAC status and monitoring data.  For a detailed description of the several fields, please refer to
 * the CSAC manual.
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;            /**< XCOM header */
    uint32_t status;              /**< Unit status */
    uint32_t alarm;               /**< Pending unit alarms */
    uint8_t serial_number[32];       /**< Unit serial number */
    uint32_t mode;                /**< Mode of operation */
    uint32_t contrast;            /**< Indication of signal level */
    float laser_current;          /**< Laser current [mA] */
    float tcx0;                   /**< Tuning voltage [V] */
    float heat_p;                 /**< Physics package heater power [mW] */
    float sig;                    /**< DC signal level [V] */
    float temperature;            /**< Unit temperature [°C] */
    int32_t steer;                /**< Frequency adjust */
    float atune;                  /**< Analog tuning voltage input */
    int32_t phase;                /**< Difference between CSAC and external 1PPS [ns] */
    uint32_t disc_ok;             /**< Disciplining status */
    uint32_t time_since_power_on; /**< Time since power-on [s] */
    uint32_t time_since_lock;     /**< Time since lock [s] */
    uint8_t data_valid;           /**< Data validity flag */
    uint8_t reserved;             /**< Reserved for further use */
    uint16_t fw_version;          /**< Firmware version */
    XCOMFooter footer;
} XCOMmsg_CSACDATA;
/**
 * The GNSSSOL message contains the GNSS solution (origin is the primary antenna). If the INS is aided with RTK corrections, this log contains the RTK position.
 * #domain: public
 * #rate: gnss
 * #name: XCOMmsg_GNSSSOL
 */
enum GnssSolStatus {
    SOL_COMPUTED        = 0,    /**< Solution computed */
    INSUFFICIENT_OBS    = 1,    /**< Insufficient observations */
    NO_CONVERGENCE      = 2,    /**< No convergence */
    SINGULARITY         = 3,    /**< Singularity at parameters matrix */
    COV_TRACE           = 4,    /**< Covariance trace exceeds maximum (trace >1000 m) */
    TEST_DIST           = 5,    /**< Test distance exceeded (maximum of 3 rejections if distance >10 km) */
    COLD_START          = 6,    /**< Not yet converged from cold start */
    V_H_LIMIT           = 7,    /**< Height or velocity limits exceeded (in accordance with  export licensing restrictions) */
    VARIANCE            = 8,    /**< Variance exceeds limits */
    RESIDUALS           = 9,    /**< Residuals are too large */
    DELTA_POS           = 10,   /**< Delta position is too large */
    NEGATIVE_VAR        = 11,   /**< Negative variance */
    RESERVED            = 12,   /**< Reserved for further use */
    INTEGRITY_WARNING   = 13,   /**< Large residuals make position unreliable */
    PENDING             = 18,   /**< When a FIX position command is entered, the receiver computes its own position and determines if the fixed
                                     position is valid */
    INVALID_FIX         = 19,   /**< The fixed position, entered using the FIX position    command, is not valid */
    UNAUTHORIZED        = 20,   /**< Position type is unauthorized - HP or XP on a receiver not authorized for it */
    ANTENNA_WARNING     = 21    /**< Antenna warning */
};
enum GnssPosVelType {
    NONE                = 0,     /**< No solution */
    FIXEDPOS            = 1,     /**< Position has been fixed by the FIX POSITION command */
    FIXEDHEIGHT         = 2,     /**< Position has been fixed by the FIX HEIGHT/AUTO command */
    DOPPLER_VELOCITY    = 8,     /**< Velocity computed using instantaneous Doppler */
    SINGLE              = 16,    /**< Single point position */
    PSRDIFF             = 17,    /**< Pseudorange differential solution */
    SBAS                = 18,    /**< Solution calculated using corrections from an SBAS */
    PROPAGATED          = 19,    /**< Propagated by a Kalman filter without new observations */
    OMNISTAR            = 20,    /**< OmniSTAR VBS position (L1 sub-metre) */
    L1_FLOAT            = 32,    /**< Floating L1 ambiguity solution */
    IONOFREE_FLOAT      = 33,    /**< Floating ionospheric-free ambiguity solution */
    NARROW_FLOAT        = 34,    /**< Floating narrow-lane ambiguity solution */
    L1_INT              = 48,    /**< Integer L1 ambiguity solution */
    WIDE_INT            = 49,    /**< Integer wide-lane ambiguity solution */
    NARROW_INT          = 50,    /**< Integer narrow-lane ambiguity solution */
    OMNISTAR_HP         = 64,    /**< OmniSTAR HP position */
    OMNISTAR_XP         = 65,    /**< OmniSTAR XP or OmniSTAR G2 (GPS+GLONASS) position */
    PPP_CONVERGING      = 68,    /**< Converging TerraStar-C solution */
    PPP                 = 69     /**< Converged TerraStar-C solution */
};
/* GNSS Status */
typedef struct {
    uint32_t InvalidFirmware : 1;               /**< Invalid firmware */
    uint32_t RomStatus : 1;                     /**< ROM status */
    uint32_t SupplyVoltage : 1;                 /**< Supply voltage status */
    uint32_t PllRf1Error : 1;                   /**< TrackingPLL RF1 Hardware Status - L1 */
    uint32_t PllRf2Error : 1;                   /**< TrackingPLL RF2 Hardware Status - L2 */
    uint32_t SwRessourceLimit : 1;              /**< Software Resource Status */
    uint32_t InvalidModel : 1;                  /**< Model not valid */
    uint32_t HwFailure : 1;                     /**< Component hardware failure */
    uint32_t TempError : 1;                     /**< Temperature status */
    uint32_t PrimaryAntennaPowerStatus : 1;     /**< Primary Antenna power status (0: Power ok; 1: Power not ok) */
    uint32_t PrimaryAntennaOpen : 1;            /**< Primary antenna open */
    uint32_t PrimaryAntennaShorted : 1;         /**< Primary antenna shorted */
    uint32_t CpuOverload : 1;                   /**< CPU overload */
    uint32_t Com1BufferOverrun : 1;             /**< COM1 buffer overrun */
    uint32_t Com2BufferOverrun : 1;             /**< COM2 buffer overrun */
    uint32_t Com3BufferOverrun : 1;             /**< COM3 buffer overrun */
    uint32_t AlmanacFlag : 1;                   /**< Almanac flag/UTC known */
    uint32_t SolutionInvalid : 1;               /**< Position solution flag */
    uint32_t VoltageSupply : 1;                 /**< Power supply warning */
    uint32_t UpdateInProgress : 1;              /**< Firmware update in progress */
    uint32_t SolutionCodeType : 1;              /**< 0: Some data in PVT solution from C/A-code. 1: All data in PVT solution from P-code */
    uint32_t SecondaryAntennaOpen : 1;          /**< Secondary antenna open */
    uint32_t SecondaryAntennaShorted : 1;       /**< Secondary antenna shorted */
    uint32_t SecondaryAntennaPowerStatus : 1;   /**< Secondary Antenna power status (0: Power ok; 1: Power not ok) */
    uint32_t Reserved : 8;                      /**< Reserved for further use */
} GnssStatusBits;
typedef union {
    uint32_t value;
    GnssStatusBits bits;
} GnssStatusType;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;    /**< XCOM header */
    double lon;           /**< GNSS longitude (WGS84) in [rad] */
    double lat;           /**< GNSS latitude (WGS84) in [rad] */
    float alt;            /**< GNSS mean sea level in [m]. Vertical distance above the  geoid. */
    float undulation;     /**< Undulation - the relationship between the geoid and the ellipsoid in [m] */
    float v_ned[3];       /**< GNSS north, east and down velocity in [m/s] */
    float stddev_pos[3];  /**< Standard deviation of GNSS longitude, latitude and altitude in [m] */
    float stddev_vel[3];  /**< Standard deviation of GNSS north, east and down velocity in [m/s] */
    uint16_t sol_status;  /**< GNSS Solution status (see GnssSolStatus in appendix) */
    uint16_t posvel_type; /**< GNSS position/velocity type (see GnssPosVelType in appendix) */
    float pdop;           /**< Position Dilution Of Precision - uncertainty indicator for 3D parameters (latitude, longitude, height) */
    uint8_t sats_used;    /**< Number of satellites used in GNSS solution */
    uint8_t sats_tracked; /**< Number of satellites tracked */
    uint16_t station_id;  /**< Reference station ID of differential corrections */
    float diff_age;       /**< Differential age in [sec] - differential correction indicator */
    float sol_age;        /**< Solution age in [sec] */
    GnssStatusType gnss_status; /**< GNSS receiver status (see GNSS Status) */
    XCOMFooter footer;
} XCOMmsg_GNSSSOL;
/**
 * This message contains the GNSS position in ECEF coordinates (origin is the primary antenna).
 * #domain: public
 * #rate: gnss
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;            /**< XCOM header */
    double pos[3];                /**< GNSS position in ECEF coordinates [m] */
    float stddev[3];              /**< Standard deviation of GNSS position solution [m] */
    uint16_t solution_status;     /**< GNSS Solution status (see GnssSolStatus in appendix) */
    uint16_t position_type;       /**< GNSS position/velocity type (see GnssPosVelType in appendix) */
    float pdop;                   /**< Position Dilution Of Precision - uncertainty indicator for 3D parameters (latitude, longitude, height) */
    uint8_t sats_used;            /**< Number of satellites used in GNSS solution */
    uint8_t sats_tracked;         /**< Number of satellites tracked */
    uint16_t station_id;          /**< Reference station ID of differential corrections */
    float diff_age;               /**< Differential age in [sec] - differential correction indicator */
    float solution_age;           /**< Solution age in [sec] */
    GnssStatusType gnss_status;   /**< GNSS receiver status (see GNSS Status) */
    XCOMFooter footer;
} XCOMmsg_GNSSPOSECEF;
/**
 * This message contains the GNSS velocity in ECEF coordinates (origin is the primary antenna).
 * #domain: public
 * #rate: gnss
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;            /**< XCOM header */
    double vel[3];                /**< GNSS velocity in ECEF coordinates [m/s] */
    float stddev[3];              /**< Standard deviation of GNSS velocity solution [m/s] */
    uint16_t solution_status;     /**< GNSS Solution status (see GnssSolStatus in appendix) */
    uint16_t velocity_type;       /**< GNSS position/velocity type (see GnssPosVelType in appendix) */
    float pdop;                   /**< Position Dilution Of Precision - uncertainty indicator for 3D parameters (latitude, longitude, height) */
    uint8_t sats_used;            /**< Number of satellites used in GNSS solution */
    uint8_t sats_tracked;         /**< Number of satellites tracked */
    uint16_t station_id;          /**< Reference station ID of differential corrections */
    float diff_age;               /**< Differential age in [sec] - differential correction indicator */
    float solution_age;           /**< Solution age in [sec] */
    GnssStatusType gnss_status;   /**< GNSS receiver status (see GNSS Status) */
    XCOMFooter footer;
} XCOMmsg_GNSSVELECEF;
/**
 * This message contains satellite specific information. Please note that the message length is variable.
 * XCOM_MAX_SATCHANNELS defines the max. number of supported satellite channels.
 * #domain: public
 * #rate: gnss
 * #name: XCOMmsg_GNSSSATINF
 */
#define XCOM_MAX_SATCHANNELS               128  /**< Max. number of supported satellite channels */
typedef struct {
    uint32_t sat_system; /**< GNSS satellite system identifier */
    uint16_t sat_id;     /**< Space Vehicle ID. SVID 1-32 are GPS, 38-70 are GLONASS */
    int16_t glo_freq;    /**< GLONASS frequency */
    float cn0;           /**< Satellite carrier to noise ratio in dB-Hz for L1, L2 and L5 carrier */
    uint32_t reject;     /**< Range reject code from pseudorange filter */
    float azimuth;       /**< Satellite azimuth angle in [rad] */
    float elevation;     /**< Satellite elevation angle in [rad] */
} SatelliteInfo;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                      /**< XCOM header */
    uint32_t num_obs;                                       /**< Number of satellite observations */
    SatelliteInfo sat_observations[XCOM_MAX_SATCHANNELS];   /**< see XCOM_MAX_SATCHANNELS in appendix */
    XCOMFooter footer;
} XCOMmsg_GNSSSATINF;
/**
 * This message provides several time related pieces of information including receiver clock offset and UTC time and offset.
 * #domain: public
 * #rate: gnss
 * #name: XCOMmsg_GNSSTIME
 */
#define XCOMMSG_GNSSTIME_STATUS_INVALID 0 /**< Status is invalid */
#define XCOMMSG_GNSSTIME_STATUS_VALID   1 /**< Status is valid */
#define XCOMMSG_GNSSTIME_STATUS_WARNING 2 /**< Indicates that the leap seconds value is used as a default due to the lack of an almanac */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;   /**< XCOM header */
    double utc_offset;   /**< The offset of GPS reference time from UTC time, computed using almanac parameters. UTC time is GPS reference
                              time plus the current UTC offset plus the receiver clock offset:
                              UTC time = GPS reference time + offset + UTC offset */
    double offset;       /**< Receiver clock offset, in seconds from GPS reference time. A positive offset implies that the receiver clock
                              is ahead of GPS reference time. To derive GPS reference time, use the following formula:
                              GPS reference time = receiver time - offset */
    uint32_t utc_year;   /**< UTC year */
    uint8_t utc_month;   /**< UTC month (0-12). If UTC time is unknown, the value is 0 */
    uint8_t utc_day;     /**< UTC day (0-31). If UTC time is unknown, the value is 0 */
    uint8_t utc_hour;    /**< UTC hour (0-23). If UTC time is unknown, the value is 0 */
    uint8_t utc_min;     /**< UTC min (0-59). If UTC time is unknown, the value is 0 */
    uint32_t utc_ms;     /**< UTC milliseconds (0-60999). Maximum of 60999 when leap second is applied. */
    uint32_t utc_status; /**< UTC Status (see XCOMMSG_GNSSTIME_STATUS_* in appendix) */
    XCOMFooter footer;
} XCOMmsg_GNSSTIME;
/**
 * This message contains the dual antenna GNSS heading information (angles of the vector from the primary to the secondary anenna).
 * #domain: public
 * #rate: gnss
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;        /**< XCOM header */
    float heading;            /**< Heading in [rad] */
    float heading_stddev;     /**< Heading standard deviation in [rad] */
    float pitch;              /**< Pitch angle in [rad] */
    float pitch_stddev;       /**< Pitch standard deviation in [rad] */
    uint16_t solution_status; /**< GNSS Solution status (see GnssSolStatus in appendix) */
    uint16_t position_type;   /**< GNSS position/velocity type (see GnssPosVelType in appendix) */
    uint16_t reserved;        /**< Reserved for further use */
    uint8_t sats_used;        /**< Number of satellites used in dual-antenna solution */
    uint8_t sats_tracked;     /**< Number of satellites tracked */
    uint32_t gnss_status;     /**< GNSS receiver status (see GNSS Status) */
    XCOMFooter footer;
} XCOMmsg_GNSSHDG;
/**
 * This message contains the GNSS antenna lever arm in body frame being estimated by the EKF.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                    /**< XCOM header */
    float primary_ant_offset[3];          /**< Estimated x, y and z primary antenna offset in IMU frame in [m] */
    float primary_ant_offset_stddev[3];   /**< Primary antenna standard deviation of estimated x, y and z offset in [m] */
    float secondary_ant_offset[3];        /**< Estimated x, y and z secondary antenna offset in IMU frame in [m] */
    float secondary_ant_offset_stddev[3]; /**< Secondary antenna standard deviation of estimated x, y and z offset in [m] */
    XCOMFooter footer;
} XCOMmsg_GNSSLEVERARM;
/**
 * This message contains DOP values for the satellites used in the GNSS solution
 * #domain: public
 * #rate: gnss
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;   /**< XCOM header */
    float pdop;          /**< Position dilution of precision - assumes 3D position is unknown and receiver clock offset is known */
    float hdop;          /**< Horizontal dilution of precision. */
    float vdop;          /**< Vertical dilution of precision */
    float gdop;          /**< Geometric dilution of precision - assumes 3D position and receiver clock offset (all 4 parameters) are unknown */
    float htdop;         /**< Horizontal position and time dilution of precision. */
    float tdop;          /**< Time dilution of precision - assumes 3D position is known and  only the receiver clock offset is unknown */
    XCOMFooter footer;
} XCOMmsg_GNSSDOP;
/**
 * This message log contains information regarding the GNSS voter module.
 * #domain: public
 * #rate: gnss
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;     /**< XCOM header */
    uint8_t sats_used_int; /**< Number of satellites used for internal receiver */
    uint8_t sats_used_ext; /**< Number of satellites used for external receiver */
    uint16_t reserved;     /**< Reserved for further use */
    float stddev_hdg_int;  /**< Standard deviation heading of internal receiver in  [rad] */
    float stddev_hdg_ext;  /**< Standard deviation heading of external receiver in  [rad] */
    float stddev_pos_int;  /**< Standard deviation position of internal receiver in  [m] */
    float stddev_pos_ext;  /**< Standard deviation position of external receiver in  [m] */
    uint32_t status;       /**< GNSS Voter Status Mask, */
    XCOMFooter footer;
} XCOMmsg_GNSSVOTER;
/**
 * This message contains the hardware monitor status information of the GNSS receiver (temperature, antenna current and voltages).
 * Each item consists of a float value and a related status.
 * #domain: public
 * #rate: gnss
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;   /**< XCOM header */
    struct {
        float val;       /**< Value related to the status information */
        uint32_t status; /**< The related status */
    } GnssHwMonitor[16];
    XCOMFooter footer;
} XCOMmsg_GNSSHWMON;
/**
 * This log contains port statistics of the integrated GNSS receiver COM ports and may be used to debug issues with e.g. correction data
 * input.
 * #domain: public
 * #rate: gnss
 * #name: XCOMmsg_PORTSTATS
 */
#define XCOMMSG_PORTSTATS_MAXPORTS 3 /**< Number of supported ports */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    int32_t number_of_elements;     /**< Number of status elements */
    struct {
        uint32_t port;              /**< COM port */
        uint32_t rx_chars;          /**< Total number of characters received through this port */
        uint32_t tx_chars;          /**< Total number of characters transmitted through this port */
        uint32_t acc_rx_chars;      /**< Total number of accepted characters received through this port */
        uint32_t droppped_rx_chars; /**< Number of software overruns in receive */
        uint32_t interrupts;        /**< Number of interrupts on this port */
        uint32_t breaks;            /**< Number of breaks (only for serial ports) */
        uint32_t parity_errors;     /**< Number of parity errors (only for serial ports) */
        uint32_t frame_errors;      /**< Number of framing errors (only for serial ports) */
        uint32_t rx_overrun;        /**< Number of hardware overruns in receive */
    } stats[XCOMMSG_PORTSTATS_MAXPORTS];
    XCOMFooter footer;
} XCOMmsg_PORTSTATS;
/**
 * GNSS raw data stream in proprietary binary protocol (depends on internal receiver type). Please note that the message length is variable.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< Generic XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_GNSSSTREAM;
/**
 * GNSS correction data in proprietary binary protocol (depends on the selected data format). Please note that the message length is variable.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_GNSSOBSSTREAM;
/**
 * External sensor raw data stream. Please note that the message length is variable.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_EXTSTREAM;
/**
 * This message contains the odometer measurements
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float speed;        /**< Odometer velocity in vehicle frame [m/s], obtained from the time measurement between observed odometer counts */
    int32_t ticks;      /**< Up/Down counter of odometer pulses. The counter will never be reset. */
    XCOMFooter footer;  /**< XCOM footer */
} XCOMmsg_WHEELDATA;
/**
 * This message contains the odometer lever arm in body frame being estimated by the EKF.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;               /**< XCOM header */
    float lever_arm_est[3];          /**< Estimated x, y and z lever arm in IMU frame in [m] */
    float lever_arm_stddev_est[3];   /**< Standard deviation of estimated x, y and z lever arm in [m] */
    XCOMFooter footer;              /**< XCOM footer */
} XCOMmsg_ODOLEVERARM;
/**
 * The POSTPROC log message is an in-house extension of the iXCOM protocol. It is open to future additions while keeping a fixed set of
 * sensor and navigation data that can be used internally within iMAR’s hardware and software products. The parameter contains post process
 * data solutions for inertial and odometer raw data, as well as EKF-based navigation results. The inertial raw data includes day-to-day
 * bias and scale factor as well as sensor mounting corrections. However, unlike IMURAW, POSTPROC data is not affected by the configured IMU
 * misalignment.
 *
 * To support future extensions, this message is of variable length. It consists of at least 252 bytes; the actual length is given by
 * the “Msg Length” field of the message header.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;            /**< XCOM header */
    float accel[3];               /**< Inertial data, calibrated acceleration along IMU’s x-, y- and z-axis in [m/s2] */
    float omg[3];                 /**< Inertial data, calibrated angular rate along IMU’s x-, y-, and z-axis in [rad/s] */
    float delta_theta[12];        /**< Inertial data, reserved for angular increments (≤ four measurement samples per axis) */
    float delta_velocity[12];     /**< Inertial data, reserved for velocity increments (≤ four measurement samples per axis) */
    double q_nb[4];               /**< Navigation data, body frame to NED quaternion, L-, I-, J- and K-component */
    double pos_llh[3];            /**< Navigation data, longitude and latitude position in WGS84 in [rad], altitude position in [m] */
    double vel_ned[3];            /**< Navigation data, north, east and down velocity in [m/s] */
    uint32_t sys_stat;            /**< Extended System Status data */
    uint32_t ekf_stat_low;        /**< Status data, EKF aiding status low */
    uint32_t ekf_stat_hi;         /**< Status data, EKF aiding status hi */
    float odo_speed;              /**< Odometer / Timing data, velocity in vehicle frame in [m/s], obtained from the time measurement between
                                       observed odometer counts */
    int32_t odo_ticks;            /**< Odometer / Timing data, up/down counter of odometer pulses which will never be reset */
    uint32_t odo_interval;        /**< Odometer / Timing data, FPGA ticks for one measurement   period (applies to inertial data as well) */
    uint32_t odo_trig_event;      /**< Odometer / Timing data, ticks elapsed until first odometer event */
    uint32_t odo_trig_next_event; /**< Odometer / Timing data, ticks from last odometer event until the end of interval */
    XCOMFooter footer;            /**< XCOM footer */
} XCOMmsg_POSTPROC;
/**
 * This message contains the statistics of the integrated CAN controller.
 * #domain: public
 * #rate: event
 * #name: XCOMmsg_CANSTATUS2
 */
#define XCOMMSG_CANSTATUS2_IDX_TRANSMITTED_FRAMES           0  /**< transmit mailbox processing */
#define XCOMMSG_CANSTATUS2_IDX_RECEIVED_FRAMES              1  /**< receive event processing */
#define XCOMMSG_CANSTATUS2_IDX_MISSING_ACK                  2  /**< missing acknowledges: The message this CAN core transmitted was not acknowledged by another node. */
#define XCOMMSG_CANSTATUS2_IDX_FRAME_ERRORS                 3  /**< frame error counter */
#define XCOMMSG_CANSTATUS2_IDX_STUFF_ERRORS                 4  /**< stuff error. More than five equal bits in a row have been detected in a part of a received message where this is not allowed. */
#define XCOMMSG_CANSTATUS2_IDX_FORM_ERRORS                  5  /**< form error. A fixed format part of a received frame has the wrong format. */
#define XCOMMSG_CANSTATUS2_IDX_DOM_BIT_RECESS_ERRORS        6  /**< during the transmission of a message (or acknowledge bit, or active error flag, or overload flag), the device
                                                                    wanted to send a dominant level (logical value '0'), but the monitored bus level was recessive.
                                                                    During Bus-Off recovery, this status is set each time a sequence of 11 recessive bits has been monitored.
                                                                    This enables the CPU to monitor the proceeding of the Bus-Off recovery sequence (indicating the bus is not
                                                                    stuck at dominant or continuously disturbed). */
#define XCOMMSG_CANSTATUS2_IDX_RECESS_BIT_DOM_ERRORS        7  /**< during the transmission of a message (with the exception of the arbitration field), the device wanted to send
                                                                    a recessive level (bit of logical value '1'), but the monitored bus value was dominant. */
#define XCOMMSG_CANSTATUS2_IDX_PARITY_ERRORS                8  /**< the parity check mechanism has detected a parity error in the Message RAM. */
#define XCOMMSG_CANSTATUS2_IDX_CRC_ERRORS                   9  /**< CRC error. In a received message, the CRC check sum was incorrect. (CRC received for an incoming message does
                                                                    not match the calculated CRC for the received data). */
#define XCOMMSG_CANSTATUS2_IDX_HW_RECEIVE_OVERFLOWS         10 /**< hardware buffer overflow */
#define XCOMMSG_CANSTATUS2_IDX_SW_RECEIVE_OVERFLOWS         11 /**< software buffer overflow */
#define XCOMMSG_CANSTATUS2_IDX_ERROR_WARN_STATE_COUNTER     12 /**< at least one of the error counters has reached the error warning limit of 96. */
#define XCOMMSG_CANSTATUS2_IDX_ERROR_PASSIVE_STATE_COUNTER  13 /**< the CAN core is in the error passive state as defined in the CAN Specification. */
#define XCOMMSG_CANSTATUS2_IDX_BUS_OFF_STATE_COUNTER        14 /**< the CAN module is in bus-off state. */
#define XCOMMSG_CANSTATUS2_IDX_BUS_IDLE_COUNTER             15 /**< no CAN bus event was detected since the last time the CPU read the error and status register.
                                                                    Any read access to the error and status register re-initializes the LEC to value '7.' */
#define XCOMMSG_CANSTATUS2_IDX_POWER_DOWN_COUNTER           16 /**< application request counter for setting DCAN to local power-down mode was successful. DCAN is in local
                                                                    power-down mode. */
#define XCOMMSG_CANSTATUS2_IDX_WAKE_UP_COUNTER              17 /**< wake-up counter of the system due to dominant CAN bus while module power down. */
#define XCOMMSG_CANSTATUS2_IDX_RX_INTERRUPTS                18 /**< successful receive events */
#define XCOMMSG_CANSTATUS2_IDX_TX_INTERRUPTS                19 /**< successful transmit events */
#define XCOMMSG_CANSTATUS2_IDX_TOTAL_INTERRUPTS             20 /**< successful receive and transmit events */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;             /**< XCOM header */
    uint8_t device;                /**< device ID (can be CAN0 or CAN1) */
    uint8_t reserved[3];           /**< reserved for further use */
    uint32_t status_registers[32]; /**< see XCOMMSG_CANSTATUS2_IDX_* in appendix */
    XCOMFooter footer;
} XCOMmsg_CANSTATUS2;
/**
 * This message contains status information of the integrated CAN controller
 * @deprecated
 * #domain: deprecated
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    uint32_t error_mask;        /**< CAN status error mask */
    uint8_t controller_status;  /**< CAN controller status information */
    uint8_t transceiver_status; /**< CAN transceiver status information */
    uint8_t protocol_status;    /**< CAN protocol status information */
    uint8_t protocol_location;  /**< CAN protocol location */
    XCOMFooter footer;
} XCOMmsg_CANSTATUS;
/**
 * The EVENTTIME message contains the time measurement of the external event inputs. This log can only be triggered via an external event.
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;      /**< XCOM header */
    double time_event[2];   /**< Measured GPS timestamp in [sec] for external event detection via EVENT_CHANNEL_0 and EVENT_CHANNEL_1 */
    XCOMFooter footer;      /**< XCOM footer */
} XCOMmsg_EVENTTIME;
/**
 * This message contains the received CAN messages.
 * #domain: public
 * #rate: event
 * #name: XCOMmsg_CANGATEWAY
 */
#define XCOMMSG_CANGATEWAY_MAXBUFFER    512         /**< Max. size of payload buffer */
#define XCOMMSG_CANGATEWAY_PAYLOAD_LEN    8         /**< Size of CAN payload */
typedef struct XCOM_STRUCT_PACK {
    uint8_t data[XCOMMSG_CANGATEWAY_PAYLOAD_LEN];   /**< CAN receive buffer */
    uint8_t length;                                 /**< Number of received bytes */
    uint8_t is_extended_mid;                        /**< 1=29-bit MID, 0=11-bit MID */
    uint8_t is_remote_frame;                        /**< 1=remote frame request, 0=data frame */
    uint8_t reserved;                               /**< Reserved for further use */
    uint32_t mid;                                   /**< CAN message identifier */
    double timestamp;                               /**< Receive timestamp the the CAN message */
} XCOMmsg_CANGATEWAY_MsgType;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                              /**< XCOM header */
    XCOMCanGatewayHeader subheader;                 /**< CAN gateway sub-header (contains interface information) */
    uint8_t buffer[XCOMMSG_CANGATEWAY_MAXBUFFER];   /**< CANGateway payload. This buffer contains the reveived CAN bus message in
                                                         XCOMmsg_CANGATEWAY_MsgType format. The length of the buffer is a multiple of
                                                         sizeof(XCOMmsg_CANGATEWAY_MsgType). Please check the XCOM message length while
                                                         decoding this message */
    XCOMFooter footer;                              /**< XCOM footer */
} XCOMmsg_CANGATEWAY;
/**
 * This message contains the NMEA0183 GGA sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_GGA;
/**
 * This message contains the NMEA0183 GLL sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_GLL;
/**
 * This message contains the NMEA0183 GSA sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_GSA;
/**
 * This message contains the NMEA0183 HDT sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_HDT;
/**
 * This message contains the NMEA0183 RMC sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_RMC;
/**
 * This message contains the NMEA0183 VTG sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_VTG;
/**
 * This message contains the NMEA0183 ZDA sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_ZDA;
/**
 * This message contains the NMEA0183 GST sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_GST;
/**
 * This message contains the NMEA0183 PIAHS sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_PIAHS;
/**
 * This message contains the NMEA0183 PISTATUS1 sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_PISTATUS1;
/**
 * This message contains the NMEA0183 PISTATUS2 sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_PISTATUS2;
/**
 * This message contains the NMEA0183 PIARTS1 sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_PIARTS1;
/**
 * This message contains the NMEA0183 PAPBN sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                               /**< XCOM header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
    XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_PAPBN;
/**
 * This message contains the NMEA0183 PASHR sentence as XCOM payload. The message length depends on the NMEA0183 sentence
 * length.
 * #domain: public
 * #rate: full
 */
typedef struct XCOM_STRUCT_PACK {
  XCOMHeader header;                               /**< XCOM header */
  uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH]; /**< XCOM payload buffer */
  XCOMFooter footer;                               /**< XCOM footer */
} XCOMmsg_NMEA0183_PASHR;
/**
 * This message contains battery status data.
 * #name: XCOMmsg_BATSTAT2
 */
typedef struct XCOM_STRUCT_PACK {
    float temperatureInCelsius;
    float voltageInVolt;
    float currentInAmpere;
    uint16_t alarmWarning;
    uint16_t averageTimeTillFullyLoadedInMinutes;
    uint16_t averageTimeTillEmptyInMinutes;
    uint8_t chargeInPercent;
    uint16_t CycleCount;
} XCOMBattery2DataStruct;
typedef struct XCOM_STRUCT_PACK {
    uint16_t system_state;
    uint16_t system_state_continue;
    XCOMBattery2DataStruct battery1;
    XCOMBattery2DataStruct battery2;
} XCOMBattery2State;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMBattery2State bat_state;
    XCOMFooter footer;
} XCOMmsg_BATSTAT2;
/**
 * The EXTPOSLLH message contains the position measurements captured by any external sensor
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    double time;                        /**< External position sensor's timestamp [s] */
    double latitude;                    /**< Latitude in [rad] */
    double longitude;                   /**< Longitude in [rad] */
    float altitude;                     /**< Altitude in [m] */
    float position_stddev[3];           /**< Latitude, longitude and altitude standard deviation in [m] */
    uint8_t solution_valid;             /**< Position solution valid/invalid (0: invalid - 1: valid) */
    uint8_t obs_used;                   /**< Number of observations used in external position solution */
    uint8_t reserved[6];                /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_EXTPOSLLH;
/**
 * The EXTVELNED message contains the velocity  measurements (in NED frame) captured by any external sensor
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    double time;                        /**< External velocity sensor's timestamp [s] */
    float vel_ned[3];                   /**< Velocity in NED frame [m/s] */
    float vel_ned_stddev[3];            /**< Velocity standard deviation in [m/s] */
    uint8_t solution_valid;             /**< Velocity solution valid/invalid */
    uint8_t obs_used;                   /**< Observations used in velocity solution */
    uint8_t reserved[6];                /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_EXTVELNED;
/**
 * The EXTVELBODY message contains the velocity  measurements (in body frame) captured by any external sensor
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    double time;                        /**< External velocity sensor's timestamp [s] */
    float vel_body[3];                  /**< Velocity in body frame [m/s] */
    float vel_body_stddev[3];           /**< Velocity standard deviation in [m/s] */
    uint8_t solution_valid;             /**< Velocity solution valid/invalid */
    uint8_t reserved[7];                /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_EXTVELBODY;
/**
 * The EXTHEIGHT message contains the height measurements captured by any external sensor
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    double time;                       /**< External height sensor's timestamp [s] */
    float height;                      /**< Height in WGS84 frame [m] */
    float height_stddev;               /**< Height standard deviation in [m] */
    uint8_t solution_valid;            /**< Heigt data valid/invalid */
    uint8_t reserved[7];               /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_EXTHEIGHT;
/**
 * The EXTHEADING message contains the heading measurements captured by any external sensor
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                 /**< XCOM header */
    double time;                       /**< External height sensor's timestamp [s] */
    float heading;                     /**< Heading measurement [rad] */
    float heading_stddev;              /**< Heading standard deviation in [rad] */
    uint8_t measurement_valid;         /**< Heading measurement valid/invalid */
    uint8_t reserved[7];               /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_EXTHEADING;
/**
 * The EXTMAGNETOMETER message contains the megnetometer measurements captured by any external sensor
 * #domain: public
 * #rate: event
 * #name: XCOMmsg_EXTMAGNETOMETER
 */
#define XCOMMSG_EXTMAG_CALSTATE_IDLE    0   /**< 3D calibration is idle */
#define XCOMMSG_EXTMAG_CALSTATE_RUNNING 1   /**< 3D calibration is running */
#define XCOMMSG_EXTMAG_CALSTATE_DONE    2   /**< 3D calibration done */
#define XCOMMSG_EXTMAG_CALSTATE_ERROR   3   /**< 3D calibration error */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                 /**< XCOM header */
    double time;                       /**< External magnetic sensor's timestamp [s] */
    float magnetic_field[3];           /**< Magnetic field vector [uT] */
    float magnetic_heading;            /**< Magnetic heading (calculated from magnetic field values) [rad] */
    uint8_t measurement_valid;         /**< Magnetic measurements valid/invalid */
    uint8_t calib_state;               /**< 3D-calibration state */
    uint8_t reserved[6];               /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_EXTMAGNETOMETER;
/**
 * The SYNCCOUNTER message contains external synchronization counter values and timestamp
 * #name: XCOMmsg_SYNCCOUNTER
 */
#define XCOMMSG_SYNCCOUNTER_MAX      3              /**< Number of supported synchronization counters */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                              /**< XCOM header */
    double system_time;                             /**< iNAT system time in [sec] */
    int32_t sync_counter[XCOMMSG_SYNCCOUNTER_MAX];  /**< External synchronization counter values */
    XCOMFooter footer;
} XCOMmsg_SYNCCOUNTER;
/**
 * *****************************************************************************************************************************************
 * XCOM command definition
 *
 * The iXCOM protocol allows sending commands via the command message ID 0xFD. The command message payload consists of a header
 * selecting a specific command and a variable payload.
 * *****************************************************************************************************************************************
 */
typedef struct XCOM_STRUCT_PACK {
    uint16_t cmd_id;   /**< Command ID (see XCOMcmdCategory) */
    uint16_t specific; /**< Meaning is dependent on CmdID (reserved) */
} XCOMCmdHeader;
/**
 * iXCOM LOG command:
 * ------------------
 * The LOG command manipulates the log list of the currently active channel. Besides a specific message ID, a Trigger, a Command Parameter
 * and a Divider value are expected.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint8_t msg_id;             /**< Defines the log which is manipulated. */
    uint8_t trigger;            /**< see XComLogTrigger */
    uint16_t log_cmd;           /**< see XComLogCmdParam */
    uint16_t divider;           /**< The data output rate divider for synchronous logs generation is relative to INS navigation rate */
    XCOMFooter footer;
} XCOMCmd_LOG;
/**
 * iXCOM LOG2 command:
 * ------------------
 * The LOG command manipulates the log list of the currently active channel. Besides a specific message ID, a Trigger, a Command Parameter
 * and a Divider value are expected.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint8_t msg_id;             /**< Defines the log which is manipulated. */
    uint8_t trigger;            /**< see XComLogTrigger */
    uint16_t log_cmd;           /**< see XComLogCmdParam */
    uint32_t divider;           /**< The data output rate divider for synchronous logs generation is relative to INS navigation rate */
    XCOMFooter footer;
} XCOMCmd_LOG2;
/**
 * iXCOM XCOM command:
 * -------------------
 * The iXCOM commands must be used to initialize/open or close the iXCOM data interface. The iXCOM command needs as parameter the
 * corresponding channel number.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint16_t xcom_cmd;          /**< see XCOMcmd_XCom */
    uint16_t channel;           /**< Number of communication channel */
    XCOMFooter footer;
} XCOMCmd_XCOM;
/**
 * This command sets the reference input/output time. If GNSS is available, the GPS time will always be preferred.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint16_t xcom_cmd;          /**< see XCOMcmd_XCom */
    uint16_t gps_week;          /**< Reserved for further use */
    uint32_t gps_tow_ms;        /**< GPS week second in [ms] */
    XCOMFooter footer;
} XCOMCmd_SETTIME;
/**
 * This command starts the gyro bias calibration.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint16_t xcom_cmd;
    uint16_t runtime;           /**< Calibration runtime in [sec] */
    XCOMFooter footer;
} XCOMCmd_GYROCALIB;
/**
 * This command configures one XCOM channel with a predefined loglist (without starting the system recorder)
 * #name: XCOMCmd_LOGLISTPRESET
 */
enum XCOM_ENUM_EXT XCOMLoglistPreset {
    Rawdata = 0,        /**< IMU/GNSS/Odometer rawdata (usefully for post-processing without having an INS online solution) */
    NavSolution = 1,    /**< INS online solution without rawdata acquisition  */
    Support = 2,        /**< Combination of Rawdata and NavSolution */
    Qualification = 3   /**< Similar to Rawdata but with additional hardware monitoring messages */
};
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint16_t xcom_cmd;          /**< XCOM command header */
    uint8_t reserved;           /**< Reserved for further use */
    uint8_t channel;            /**< XCOM channel to be configured */
    uint32_t preset;            /**< Loglist preset (see XCOMLoglistPreset) */
    XCOMFooter footer;
} XCOMCmd_LOGLISTPRESET;
/**
 * iXCOM CONF command:
 * -------------------
 * The CONF command can be used to store or load the system configuration to or from internal storage.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint32_t conf_cmd;          /**< see ixcomCommandParameterConf */
    XCOMFooter footer;
} XCOMCmd_CONF;
/**
 * iXCOM FPGA command:
 * -------------------
 * This command is not intended to be used by the common operator! Wrong usage may damage or destroy the system
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint16_t fpga_cmd;          /**< see XCOMcmd_Fpga */
    uint16_t param;
    XCOMFooter footer;
} XCOMCmd_FPGA;
/**
 * Reset external synchronization counter
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    uint16_t fpga_cmd;                  /**< see XCOMcmd_FPGA */
    uint16_t reserved;                  /**< Reserved for further use */
    XCOMFooter footer;
} XCOMCmdFPGA_RESETSYNCCOUNTER;
/**
 * iXCOM EKF command:
 * -------------------
 * Set of commands to save the current position/heading and to start an alignment.
 * #name: XCOMCmd_EKF
 */
typedef struct XCOM_STRUCT_PACK {
    uint16_t ekf_cmd_id;
    uint16_t number_of_arguments;
} XCOMCmdEkfHeader;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint16_t ekf_cmd;           /**< see ixcomCommandParameterEkf */
    uint16_t number_of_param;   /**< Number of parameter */
} XCOMCmd_EKF;
/**
 * Starting an alignment.
 * Note: If the parameter PAREKF_ALIGNTIME is set to 0, the IMS will perform an alignment until the specified attitude accuracy is met.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF command arguments */
    XCOMFooter footer;
} XCOMCmdEKF_STARTALIGN;
/**
 * The IMS will save the current position to the internal storage. The saved position will be loaded  when the STOREDPOS mode is selected
 * in the PAREKF_STARTUPV2 parameter.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEKF_STOREPOS;
/**
 * The IMS will save the current heading to the internal storage. The saved heading will be loaded when the STOREDHDG mode is selected
 * in the PAREKF_STARTUP parameter.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEKF_STOREHDG;
/**
 * The IMS will save the current state to the internal storage.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEKF_STORESTATE;
/**
 * The INS will perform zero velocity updates. The corresponding parameter has to be set to ’0’ (disable) or ’1’ (enable).
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    float enable;                       /**< Enables/Disables forced ZUPTs */
    XCOMFooter footer;
} XCOMCmdEKF_FORCEDZUPT;
/**
 * If this command is received the INS will complete the static alignment. Motion is allowed after issuing this command.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEKF_ALIGNCOMPLETE;
/**
 * Save estimated odometer scale factor to the con figuration loaded in RAM. To make this setting persist after a reboot, the CONF Command
 * has to be issued with the SAVE parameter.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEKF_COPYODOSCF;
/**
 * If the EKF is initialized successfully, start Zero Velocity Update calibration. Calibration parameter can be set via the PAREKF_ZUPTCALIB
 * parameter
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEKF_ZUPTCALIBRATION;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEkf_FORCEDSINGLEZUPT;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEkf_ABORTVERIFIEDALIGN;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMCmdHeader cmd_header;           /**< XCOM command header */
    XCOMCmdEkfHeader ekf_arguments;     /**< XCOM EKF arguments */
    XCOMFooter footer;
} XCOMCmdEkf_LOADPARFROMFILE;
/**
 * iXCOM EXTAID command:
 * -------------------
 * Set of commands send external aidings to the navigation engine.
 * The EXTAID command controls the external aiding sources inside the INS. This command can be used to send measurement
 * updates directly to the EKF module. In addition to that forced aiding commands are also supported.
 */
#define XCOM_EXTAID_TIMEMODE_ABSOLUTE 0
#define XCOM_EXTAID_TIMEMODE_LATENCY  1
/**
 * External position aiding in WGS84
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_POS_LLH */
    double position[3];             /**< longitude, latitude, altitude in [rad] and [m] */
    double position_stddev[3];      /**< standard deviation in [m] (longitude, latitude, altitude) */
    double lever_arm[3];            /**< lever_arm in  [m] */
    double lever_arm_stddev[3];     /**< lever_arm standard deviation in  [m] */
    uint32_t enable_msl_altitude;   /**< Switch between altitude and height */
    XCOMFooter footer;
} XCOMCmd_EXTAID_POSLLH;
/**
 * External position aiding in ECEF frame
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_POS_ECEF */
    double position[3];             /**< External position in ECEF frame [m] */
    double position_stddev[3];      /**< Standard deviation of the external position [m] */
    double lever_arm[3];            /**< Lever arm in x,y,z-direction [m] */
    double lever_arm_stddev[3];     /**< Standard deviation of lever arm in x,y,z-direction [m] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_POSECEF;
/**
 * External position aiding in UTM
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_POS_UTM */
    int32_t zone;                   /**< UTM zone (zero means UPS) */
    uint8_t north_hp;               /**< Hemisphere (true means north, false means south). */
    uint8_t reserved[3];            /**< Reserved for further use */
    double easting;                 /**< East component in [m] */
    double northing;                /**< North component in [m] */
    double altitude;                /**< Altitude in [m] */
    double position_stddev[3];      /**< Standard deviation of the external position in [m] (easting, northing, altitude) */
    double lever_arm[3];            /**< Lever arm in x,y,z-direction [m] */
    double lever_arm_stddev[3];     /**< Standard deviation of lever arm in x,y,z-direction [m] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_POSUTM;
/**
 * External position aiding in MGRS.
 * The following MGRS string formatting is valid:
 *      32U LV 66136 59531
 *      32U LV 6613612 5953112
 *      32ULV 66136 59531
 *      32ULV66136 59531
 *      32ULV6613659531
 * #name: XCOMCmd_EXTAID_POSMGRS
 */
#define XCOM_EXTAID_POSMGRS_MAX_LENGTH  128
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_POS_UTM */
    int8_t mgrs[XCOM_EXTAID_POSMGRS_MAX_LENGTH];    /**< MGRS string (e.g. 32U LV 66136 59531) */
    double altitude;                /**< Altitude in [m] */
    double position_stddev[3];      /**< Standard deviation of the external position in [m] (easting, northing, altitude) */
    double lever_arm[3];            /**< Lever arm in x,y,z-direction [m] */
    double lever_arm_stddev[3];     /**< Standard deviation of lever arm in x,y,z-direction [m] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_POSMGRS;
/**
 * External heading aiding
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_HDG */
    double heading;                 /**< External heading [rad] */
    double heading_stddev;          /**< Standard deviation of external heading [rad] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_HDG;
/**
 * External velocity aiding
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_VEL */
    double velocity[3];             /**< Veast, Vnorth, Vdowm [m/s] */
    double velocity_stddev[3];      /**< Standard deviation of external velocity aiding [m/s] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_VEL;
typedef XCOMCmd_EXTAID_VEL XCOMCmd_EXTAID_VELNED;  /**< External velocity aiding in NED frame */
typedef XCOMCmd_EXTAID_VEL XCOMCmd_EXTAID_VELECEF; /**< External velocity aiding in ECEF frame */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_VEL */
    double velocity[3];             /**< Veast, Vnorth, Vdowm [m/s] */
    double velocity_stddev[3];      /**< Standard deviation of external velocity aiding [m/s] */
    double lever_arm[3];            /**< Lever arm in x,y,z-direction [m] */
    double lever_arm_stddev[3];     /**< Standard deviation of lever arm in x,y,z-direction [m] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_VEL2;
typedef XCOMCmd_EXTAID_VEL2 XCOMCmd_EXTAID_VELNED2;  /**< External velocity aiding in NED frame */
typedef XCOMCmd_EXTAID_VEL2 XCOMCmd_EXTAID_VELECEF2; /**< External velocity aiding in ECEF frame */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;        /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_VEL_BODY */
    double velocity[3];             /**< External velocity in body x-,y-,z-direction [m/s] */
    double velocity_stddev[3];      /**< Standard deviation of external velocity aiding [m/s] */
    double lever_arm[3];            /**< Lever arm in x,y,z-direction [m] */
    double lever_arm_stddev[3];     /**< Standard deviation of lever arm in x,y,z-direction [m] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_VELBODY;
/**
 * External height aiding
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_HGT */
    double height;                  /**< External height [m] */
    double height_stddev;           /**< Standard deviation of external height aiding [m] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_HEIGHT;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_HGT */
    double height;                  /**< External height [m] */
    double height_stddev;           /**< Standard deviation of external height aiding [m] */
    double lever_arm[3];            /**< Lever arm in x,y,z-direction [m] */
    double lever_arm_stddev[3];     /**< Standard deviation of lever arm in x,y,z-direction [m] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_HEIGHT2;
/**
 * External barometric height aiding
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_HGT */
    double height;                  /**< External barometric height [m] */
    double height_stddev;           /**< Standard deviation of external barometric height aiding [m] */
    double lever_arm[3];            /**< Lever arm in x,y,z-direction [m] */
    double lever_arm_stddev[3];     /**< Standard deviation of lever arm in x,y,z-direction [m] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_BAROALT;
/**
 * External magnetic field aiding
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMCmdHeader cmd_header;       /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_HGT */
    double magnetic_field[3];       /**< Magnetic field vector [uT] */
    double magnetic_field_stddev[3];/**< Standard deviation of magnetic field vector [uT] */
    XCOMFooter footer;
} XCOMCmd_EXTAID_MAGFIELD;
/**
 * ******************************************************************************************
 * XCOM parameter definition
 * ******************************************************************************************
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                                          /**< XCOM header */
    XCOMParHeader param_header;                                                 /**< XCOM parameter header */
    XCOMPluginHeader plugin_header;                                             /**< XCOM plugin header */
    uint8_t payload_buffer[XCOM_MAX_MSG_PAYLOAD_LENGTH + sizeof(XCOMFooter) - sizeof(XCOMPluginHeader)];   /**< Generic payload buffer */
} XCOMPluginGeneric;
/**
 * This parameter can be used for communicating with the plugins. Please note that the parameter length is variable.
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                                       /**< XCOM header */
    XCOMParHeader param_header;                                              /**< XCOM parameter header */
    uint8_t payload_buffer[XCOM_MAX_PAR_PAYLOAD_LENGTH];                     /**< XCOM payload buffer */
    XCOMFooter footer;                                                       /**< XCOM footer */
} XCOMParPLUGIN_GENERIC;
/**
 * This parameter configures the external NMEA0183 input interface
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMParHeader param_header;         /**< XCOM parameter header */
    uint8_t enable;                     /**< Enable DVL interface */
    uint8_t port;                       /**< COM port of the DVL interface (external cable description) */
    uint16_t reserved;                  /**< Reserved for further use */
    uint32_t baud;                      /**< Baud rate of the external DVL sensor */
    struct vel_data {
        float lever_arm[3];             /**< Lever arm in body frame [m] */
        float lever_arm_stddev[3];      /**< Standard deviation of the lever arm [m] */
        float stddev[3];                /**< Velocity standard deviation [m/s] */
        float latency;                  /**< Sensor latency [s] */
        uint8_t enable;                 /**< Enable/Disable body velocity aiding */
        uint8_t reserved[3];            /**< Reserved for further use */
    } body_vel;
    struct height_data {
        float lever_arm[3];             /**< Lever arm in body frame [m] */
        float lever_arm_stddev[3];      /**< Standard deviation of the lever arm [m] */
        float stddev;                   /**< Height standard deviation [m] */
        float latency;                  /**< Sensor latency [s] */
        uint8_t enable;                 /**< Enable/Disable height aiding */
        uint8_t reserved[3];            /**< Reserved for further use */
    } height;
    XCOMFooter footer;
} XCOMParEXTSENSOR_NMEA0183;
/**
 * This parameter configures the external VIPS interface
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMParHeader param_header;         /**< XCOM parameter header */
    uint8_t enable;                     /**< Enable VIPS interface */
    uint8_t port;                       /**< COM port of the VIPS interface (external cable description) */
    uint16_t reserved;                  /**< Reserved for further use */
    uint32_t baud;                      /**< Baud rate of the external VIPS sensor */
    float lever_arm[3];                 /**< Lever arm in body frame [m] */
    float lever_arm_stddev[3];          /**< Standard deviation of the lever arm [m] */
    float latency;                      /**< Sensor latency [s] */
    float period;                       /**< Sensor period [s] */
    float min_stddev_pos;               /**< Min. position standard deviation [m] */
    float scale_stddev_pos;             /**< Position standard deviation scaling */
    float min_stddev_vel;               /**< Min. velocity standard deviation [m/s] */
    float scale_stddev_vel;             /**< Velocity standard deviation scaling */
    XCOMFooter footer;
} XCOMParEXTSENSOR_VIPS;
/**
 * This parameter defines the rotation matrix from the external sensor frame to the iNAT standard enclosure frame.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMParHeader param_header;         /**< XCOM parameter header */
    double c_body_sensor[9];            /**< Rotation matrix from the external sensor frame to the iNAT standard enclosure frame, column-wise */
    XCOMFooter footer;
} XCOMParEXTSENSOR_VBODYMISALIGN;
/**
 * This parameter contains the iMAR project number under which the system has been manufactured.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< iMAR project number */
    XCOMFooter footer;
} XCOMParSYS_PRJNUM;
/**
 * This parameter contains the part number (article number) including revision letter of the INS.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< System’s part number (P/N) including revision letter */
    XCOMFooter footer;
} XCOMParSYS_PARTNUM;
/**
 * This parameter contains the serial number of the INS.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< System’s serial number (S/N) */
    XCOMFooter footer;
} XCOMParSYS_SERIALNUM;
/**
 * This parameter contains the manufacturing date of the INS.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< System’s manufacturing date with format jjjj-mm[-dd] */
    XCOMFooter footer;
} XCOMParSYS_MFG;
/**
 * This parameter contains the NAVLIB version currently installed on the INS.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< System’s navigation/mathematical library version */
    XCOMFooter footer;
} XCOMParSYS_NAVLIB;
/**
 * This parameter contains the version number of the EKFLIB currently installed on the device.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< System’s EKF library version */
    XCOMFooter footer;
} XCOMParSYS_EKFLIB;
/**
 * This parameter contains the name of the EKF parameter set being used currently.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< System’s EKF parameter set */
    XCOMFooter footer;
} XCOMParSYS_EKFPARSET;
/**
 * This parameter contains the version number of the strapdown navigation module installed on the system.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< System’s Navigator version */
    XCOMFooter footer;
} XCOMParSYS_NAVNUM;
/**
 * This parameter contains the name of the Nav parameter set being used currently.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< System’s navigator parameter set */
    XCOMFooter footer;
} XCOMParSYS_NAVPARSET;
/**
 * This parameter contains the version number of the host operating system.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[32];        /**< Version number of the host operating system. */
    XCOMFooter footer;
} XCOMParSYS_OSVERSION;
/**
 * This parameter contains the product name, e.g. iNAT-FSSG-01.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t payload[64];        /**< Product name of the device */
    XCOMFooter footer;
} XCOMParSYS_SYSNAME;
/**
 * This parameter is read-only and will be factory set during calibration. It contains the last calibration date of the INS.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t password;          /**< Reserved for further use */
    uint16_t reserved1;         /**< Reserved for further use */
    uint8_t payload[32];        /**< System’s calibration date with format jjjj-mm[-dd] */
    XCOMFooter footer;
} XCOMParSYS_CALDATE;
/**
 * This parameter is read-only.
 * The parameter contains the system’s current uptime (time since last reboot) in [sec].
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float up_time;              /**< Current uptime of the system in [sec] */
    XCOMFooter footer;
} XCOMParSYS_UPTIME;
/**
 * This parameter is read-only. It contains the accumulated operating time in [sec] over the lifetime of the system.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;               /**< XCOM header */
    XCOMParHeader param_header;      /**< XCOM parameter header */
    uint32_t operation_hour_counter; /**< Current operational time in [sec] */
    XCOMFooter footer;
} XCOMParSYS_OPERATIONHOUR;
/**
 * This parameter is read-only and will be factory set during production. This parameter contains the nominal sampling rate in Hz of
 * the integrated inertial sensors. Writing of this parameter is password-protected.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t main_timing;       /**< Nominal IMU sampling rate in [Hz] */
    uint16_t password;
    XCOMFooter footer;
} XCOMParSYS_MAINTIMING;
/**
 * This parameter defines the time synchronization source
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t mode;              /**< Time synchronization mode */
    uint16_t reserved1;
    XCOMFooter footer;
} XCOMParSYS_SYNCMODE;
/**
 * This parameter is read-only and will be factory set during production. It contains the downsampling factor which is used in the internal
 * compensation methods, such as the coning and sculling correction module. The prescaler also affects the maximum rate which is applicable
 * through the message log mechanism. Writing this parameter is password-protected.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t prescaler;         /**< Even-numbered prescaler */
    uint16_t password;
    XCOMFooter footer;
} XCOMParSYS_PRESCALER;
/**
 * This parameter defines the boot behavior of the system. In the default setting (= 0), the system will start in normal operational mode.
 * When setting this parameter to standby (= 1), the system will boot into power saving mode and to enter normal operational mode,
 * the wakeup command will have to be sent.
 * #domain: public
 * #scope: read and write
 *
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t boot_mode;         /**< Selected Boot Mode */
    XCOMFooter footer;
} XCOMParSYS_BOOTMODE;
/**
 * This parameter contains the calibration ID
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t calib_id;          /**< Calibration ID */
    XCOMFooter footer;
} XCOMParSYS_CALIBID;
/**
 * This read-only parameter defines the minimum required FPGA firmware version for the firmware installed on the device. If the actual
 * FPGA version is lower than the version given in this parameter, some firmware features may not be available.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t major;              /**< Major release version */
    uint8_t minor;              /**< Minor release version */
    uint16_t imu_type;
    XCOMFooter footer;
} XCOMParSYS_MINFPGAVER;
/**
 * This read-only parameter contains the FPGA firmware version.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t major;              /**< Major release version */
    uint8_t minor;              /**< Minor release version */
    uint16_t imu_type;
    XCOMFooter footer;
} XCOMParSYS_FPGAVERSION;
/**
 * This read-only parameter holds the checksums calculated over the internal configuration binary files.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t rom_crc;           /**< CRC16 calculated over ROM content */
    uint16_t ram_crc;           /**< CRC16 calculated over RAM content */
    XCOMFooter footer;
} XCOMParSYS_CONFIGCRC;
/**
 * This parameter reads/writes the internal configuration EEPROM. The EEPROM is set during production and contains information about
 * the hardware platform and sensor configuration
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;            /**< XCOM header */
    XCOMParHeader param_header;   /**< XCOM parameter header */
    uint32_t imu_type;            /**< IMU type: This parameter will be set during manufacturing */
    uint32_t gnss_type;           /**< GNSS type: This parameter will be set during manufacturing */
    uint32_t hw_type;             /**< Hardware platform: This parameter will be set during manufacturing */
    int8_t mac_addr[16];          /**< read-only: This parameter can not be changed */
    int8_t cpu_serial_number[16]; /**< read-only: This parameter can not be changed */
    int8_t cpu_order_code[64];    /**< read-only: This parameter can not be changed */
    XCOMFooter footer;
} XCOMParSYS_EEPROM;
/**
 * This parameter contains information about the internal GNSS engine
 * #name: XCOMParSYS_GNSSTYPE
 * #domain: public
 * #scope: read only
 */
#define PARSYS_GNSSTYPE_INFOLENGTH 64
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                  /**< XCOM header */
    XCOMParHeader param_header;                         /**< XCOM parameter header */
    int8_t gnss_type_info[PARSYS_GNSSTYPE_INFOLENGTH];  /**< Hardware specific string */
    uint8_t has_auth_code;                              /**< Is set when autorithation code is available and valid */
    uint8_t reserved[3];                                /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParSYS_GNSSTYPE;
/**
 * This parameter is read-only and holds the git hash of the current commit of the firmware source code.
 * #name: XCOMParSYS_GITSHA
 * #domain: public
 * #scope: read only
 */
#define GITSHA_STRLENGTH 128
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMParHeader param_header;         /**< XCOM parameter header */
    int8_t sha[GITSHA_STRLENGTH];       /**< Short version of the git hash */
    XCOMFooter footer;
} XCOMParSYS_GITSHA;
/**
 * This parameter is read-only and holds the git branch name of the firmware source code.
 * #name: XCOMParSYS_GITBRANCH
 * #domain: public
 * #scope: read only
 */
#define GITBRANCH_STRLENGTH 128
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                          /**< XCOM header */
    XCOMParHeader param_header;                 /**< XCOM parameter header */
    int8_t branch_name[GITBRANCH_STRLENGTH];    /**< git branch name */
    XCOMFooter footer;
} XCOMParSYS_GITBRANCH;
/**
 * This parameter is read-only and holds the ci/cd pipeline id
 * #name: XCOMParSYS_CIPIPELINEID
 * #domain: public
 * #scope: read only
 */
#define CIPIPELINEID_STRLENGTH 128
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                             /**< XCOM header */
    XCOMParHeader param_header;                    /**< XCOM parameter header */
    int8_t pipeline_id[CIPIPELINEID_STRLENGTH];    /**< CI pileline id */
    XCOMFooter footer;
} XCOMParSYS_CIPIPELINEID;
/**
 * This parameter configures the USB interface
 * #name: XCOMParSYS_USBCONFIG
 * #domain: public
 * #scope: read and write
 */
#define PARSYS_USBCONFIG_DEVICE_MIL     0           /**< Route USB interface (device mode) to MIL connector */
#define PARSYS_USBCONFIG_DEVICE_FISCHER 1           /**< Route USB interface (device mode) to FISCHER connector */
#define PARSYS_USBCONFIG_HOST           2           /**< Route USB interface (host mode) to FISCHER connector */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                              /**< XCOM header */
    XCOMParHeader param_header;                     /**< XCOM parameter header */
    uint8_t usb_config;                             /**< USB peripherie configuration (see PARSYS_USBCONFIG_* in appendix) */
    uint8_t reserved[3];                            /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParSYS_USBCONFIG;
/**
 * This parameter contains the firmware version
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                             /**< XCOM header */
    XCOMParHeader param_header;                    /**< XCOM parameter header */
    uint8_t payload[32];                           /**< null terminated version string <MAJOR>.<MINOR>.<PATCH> */
    XCOMFooter footer;
} XCOMParSYS_FWVERSION;
/**
 * This parameter configures the internal NTP server
 * #domain: public
 * #scope: read and write
 */
#define PARTIMESYNC_MODE_GPS    0
#define PARTIMESYNC_MODE_FPGA   1
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                              /**< XCOM header */
    XCOMParHeader param_header;                     /**< XCOM parameter header */
    uint8_t enable;                                 /**< Enables/Disables NTP server services */
    uint8_t log_statistics;                         /**< Enables/Disables loggin of internal statistics */
    uint8_t enable_gnss_ntp_server;                 /**< If set, the NTP server of the GNSS engine will be used */
    uint8_t mode;                                   /**< Selects the NTP reference clock (FPGA or GPS) */
    uint32_t reserved2;                             /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParTIMESYNC_NTP;
/**
 * This parameter sets the installation misalignment between the INS enclosure and the vehicle frame.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;             /**< XCOM header */
    XCOMParHeader param_header;    /**< XCOM parameter header */
    float xyz[3];                  /**< Installation misalignment (rotation around the INS enclosure frame axis) in [rad]
                                        between the INS enclosure and the vehicle frame */
    XCOMFooter footer;
} XCOMParIMU_MISALIGN;
/**
 * This parameter is read-only and will be factory set during production.
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t type;              /**< Internal IMU Type */
    XCOMFooter footer;
} XCOMParIMU_TYPE;
/**
 * This parameter stores the offset of the enclosure reference point with respect to the center of navigation.
 * It is used internally to adjust position offset coordinates (e.g. XCOMParGNSS_ANTOFFSET).
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double refpoint_offset[3];  /**< Reference point offset with respect to the navigation center (intersection point of accelerometers) in [m] */
    XCOMFooter footer;
} XCOMParIMU_REFPOINTOFFSET;
/**
 * This parameter configures the bandstop filter which can be enabled for the  accelerations and angular rates which are contained in
 * the INSSOL log. This can be used to remove sinusoidal noise due to e.g. gyro dithering from the measured inertial data.
 * The default values are set so this apparent noise is filtered out. Output in the INSRPY log has to be enabled using the PARDAT_IMU
 * parameter.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float bandwidth;            /**< Bandwidth of the bandstop in [Hz] */
    float center;               /**< Center frequency of the bandstop in [Hz] */
    XCOMFooter footer;
} XCOMParIMU_BANDSTOP;
/**
 * This parameter selects the secondary IMU if available
 * #domain: hidden
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t use_secondary;     /**< enable/disable secondary IMU */
    uint16_t reserved;          /**< reserved for further use */
    XCOMFooter footer;
} XCOMParIMU_SECONDARY;
/**
 * This parameter enables or disables GNSS dual antenna mode inside the GNSS module.
 * #name: XCOMParGNSS_DUALANTMODE
 * #domain: hidden
 * #scope: read and write
 */
#define PARGNSS_DUALANTMODE_DISABLE 0   /**< Disable dual antenna module */
#define PARGNSS_DUALANTMODE_ENABLE  1   /**< Enable dual antenna module */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< GNSS dual antenna mode */
    XCOMFooter footer;
} XCOMParGNSS_DUALANTMODE;
/**
 * This parameter configures the GNSS lever arm of the primary or secondary antenna. The measurement should be done as accurately as
 * possible and the standard deviations should over bound the actual error to be expected from the measurement. The antenna offset is the
 * vector from the enclosure reference point to the antenna phase center in INS enclosure frame coordinates. For every standard deviation,
 * only values > 0 are accepted.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParGNSS_ANTOFFSET
 */
#define PARGNSS_ANTOFFSET_SEL_PRIM_INT 0 /**< Primary internal antenna */
#define PARGNSS_ANTOFFSET_SEL_SEC_INT  1 /**< Secondary internal antenna */
#define PARGNSS_ANTOFFSET_SEL_PRIM_EXT 2 /**< Primary external antenna (optional) */
#define PARGNSS_ANTOFFSET_SEL_SEC_EXT  3 /**< Secondary external antenna (optional) */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    union {
        XCOMParHeader param_header;     /**< XCOM parameter header */
        struct {
            uint16_t param_id;
            uint8_t rx_selection;        /**< Selection between primary and secondary antenna */
            uint8_t is_request;
        };
    };
    float lever_arm[3];                  /**< Distance between IMU’s reference point of measurement and GNSS antenna in x-, y- and z-direction in [m] */
    float stddev[3];                     /**< Uncertainty of the measured lever arm in x-, y- and z-direction in [m] */
    XCOMFooter footer;
} XCOMParGNSS_ANTOFFSET;
/**
 * This parameter configures the internal GNSS gateway.
 * The GNSS gateway provides a bi-directional bridge between the internal COM port (to the GNSS receiver) and an external UDP/TCP port
 * (to the XCOM client).
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t udp_enable;         /**< Enable/disable UDP interface */
    uint8_t reserved1[3];       /**< Reserved for further use */
    uint32_t udp_addr;          /**< UDP destination address */
    uint32_t udp_port;          /**< UDP destination port */
    uint32_t tcp_port;          /**< TCP destination port */
    XCOMFooter footer;
} XCOMParGNSS_GATEWAYCFG;
/**
 * This parameter enables/disables the RTK mode. If RTKMODE is set to Auto-Detection, the systems will check the GNSS receiver’s model
 * during configuration phase. It is recommended to use the Auto-Detection mode.
 * #domain: deprecated
 * #scope: read only
 * #name: XCOMParGNSS_RTKMODE
 */
#define GNSS_RTKMODE_MODESELECTION_AUTO 2
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< GNSS receiver selection for real time kinematic measurement
                                        0: Disable RTK mode
                                        1: Enable RTK mode
                                        2: Auto-Detection (receiver specific) */
    XCOMFooter footer;
} XCOMParGNSS_RTKMODE;
/**
 * This parameter enables or disables the RTCMv3 forwarding module. Using this module, correction data can be sent to the system by
 * wrapping it in the PARGNSS_RTCMV3AIDING parameter. The INS forwards the received data stream to the internal GNSS receiver after
 * stripping the iXCOM frame wrapper.
 * #domain: hidden
 * #scope: read only
 * #name: XCOMParGNSS_RTCMV3CONFIG
 */
#define PARGNSS_RTCMV3CONFIG_DISABLE 0
#define PARGNSS_RTCMV3CONFIG_ENABLE  1
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t reserved1;          /**< Reserved for further use */
    uint8_t enable;             /**< Enables/Disables RTCM forwarding */
    uint16_t reserved2;         /**< Reserved for further use */
    uint32_t reserved3;         /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParGNSS_RTCMV3CONFIG;
/**
 * This parameter can be used to exclude certain satellite constellations from the GNSS solution computation.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParGNSS_LOCKOUTSYSTEM
 */
#define PARGNSS_LOCKOUTSYSTEM_MASK_GPS     (1 << 0)
#define PARGNSS_LOCKOUTSYSTEM_MASK_GLONASS (1 << 1)
#define PARGNSS_LOCKOUTSYSTEM_MASK_SBAS    (1 << 2)
#define PARGNSS_LOCKOUTSYSTEM_MASK_GALILEO (1 << 3)
#define GNSS_LOCKOUTSYSTEM_MASK_BEIDOU     (1 << 4)
#define GNSS_LOCKOUTSYSTEM_MASK_QZSS       (1 << 5)
#define GNSS_LOCKOUTSYSTEM_MASK_NAVIC      (1 << 6)
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t lockoutsystem_mask; /**< Lockout System Mask */
    uint8_t reserved1;          /**< Reserved for further use */
    uint16_t reserved2;         /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParGNSS_LOCKOUTSYSTEM;
/**
 * This parameter enables or disables the supply of electrical power from the internal power source of the receiver.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParGNSS_ANTENNAPOWER
 */
#define GNSS_ANTENNAPOWER_SWITCH_OFF            0 /**< Disables antenna power */
#define GNSS_ANTENNAPOWER_SWITCH_ON             1 /**< Enables antenna power */
#define GNSS_ANTENNAPOWER_SWITCH_PRIMON_SECOFF  3 /**< Enables primary antenna power and disables secondary antenna power */
#define GNSS_ANTENNAPOWER_SWITCH_PRIMOFF_SECON  4 /**< Disables primary antenna power and enables secondary antenna power */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t power_mask;        /**< Antenna power selection mask (see GNSS_ANTENNAPOWER_SWITCH_* in appendix)*/
    XCOMFooter footer;
} XCOMParGNSS_ANTENNAPOWER;
/**
 * This parameter controls the PPS line of the internal GNSS receiver.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParGNSS_PPSCONTROL
 */
#define GNSS_PPSCONTROL_SWITCH_DISABLE                      0 /**< Disable PPS */
#define GNSS_PPSCONTROL_SWITCH_ENABLE                       1 /**< Enable PPS */
#define GNSS_PPSCONTROL_SWITCH_ENABLE_FINETIME              2 /**< Enable the PPS only when FINE or FINESTEERING time status has  been reached (hw dependent) */
#define GNSS_PPSCONTROL_SWITCH_ENABLE_FINETIME_MINUTEALIGN  3 /**< Enable the PPS only when FINE or FINESTEERING time status has
                                                                   been reached AND the start of the next 60 seconds (1 minute modulus) has occurred (hw dependent) */
#define GNSS_PPSCONTROL_POLARITY_NEGATIVE                   0 /**< Polarity of PPS */
#define GNSS_PPSCONTROL_POLARITY_POSITIVE                   1 /**< Polarity of PPS */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMParHeader param_header;     /**< XCOM parameter header */
    uint32_t pps_switch;            /**< PPS control switch */
    uint32_t polarity;              /**< Field to specify the polarity of the pulse to be generated on the PPS output. */
    double period;                  /**< Field to specify the period of the pulse, in [s] */
    uint32_t pulse_width_us;        /**< Field to specify the pulse width of the PPS signal in microseconds. */
    XCOMFooter footer;
} XCOMParGNSS_PPSCONTROL;
/**
 * This parameter set the update period of the GNSS engine.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float period;               /**< Update period of GNSS raw data in [sec] */
    XCOMFooter footer;
} XCOMParGNSS_PERIOD;
/**
 * This parameter contains the installed GNSS models
 * #domain: public
 * #scope: read only
 * #name: XCOMParGNSS_MODEL
 */
typedef struct XCOM_STRUCT_PACK {
    uint8_t model_name[16];     /**< GNSS model name */
    uint32_t year;              /**< Expiry year */
    uint32_t month;             /**< Expiry month */
    uint32_t day;               /**< Expiry day */
} XCOMParGNSS_MODELENTRIES;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMParHeader param_header;         /**< XCOM parameter header */
    uint8_t rtk_code;                   /**< Set when model includes RTK capabilities */
    uint8_t reserved1;                  /**< Reserved for further use */
    uint16_t reserved2;                 /**< Reserved for further use */
    XCOMParGNSS_MODELENTRIES models[6]; /**< Available GNSS models */
    XCOMFooter footer;
} XCOMParGNSS_MODEL;
/**
 * This parameter contains the selected GNSS model
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t component_type;    /**< Component type */
    uint8_t model[16];          /**< Model name */
    uint8_t psn[16];            /**< Part number */
    uint8_t hw_version[16];     /**< Hardware version */
    uint8_t sw_version[16];     /**< Software version */
    uint8_t boot_version[16];   /**< Boot version */
    uint8_t comp_date[16];      /**< Compilation date */
    uint8_t comp_time[16];      /**< Compilation time */
    XCOMFooter footer;
} XCOMParGNSS_VERSION;
/**
 * This parameter configures the correction data port of the internal GNSS receiver (@deprecated)
 * #domain: deprecated
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t rx_type;           /**< Receive interface mode */
    uint32_t tx_type;           /**< Transmit interface mode */
    uint32_t baudrate;          /**< Baud rate of GNSS correction port */
    uint32_t enable;            /**< Enable correction port */
    float period_gga;           /**< Setup GGA period in [s]. */
    XCOMFooter footer;
} XCOMParGNSS_CORPORTCFG;
/**
 * This parameter configures the dual antenna heading offset
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float offset;               /**< Offset in [rad] */
    XCOMFooter footer;
} XCOMParGNSS_HDGOFFSET;
/**
 * This parameter is used to customize the receiver's NMEA0183-GGA quality indicator
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMParHeader param_header;         /**< XCOM parameter header */
    uint8_t enable_legacy_indicator;    /**< if set, WAAS position type is translated into GGA quality indicator 2 (differential GNSS) */
    uint8_t reserved[3];                /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParGNSS_GGAQUALITY;
/**
 * This parameter configures the serial port of the magnetometer module.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t port;               /**< Serial port which is used for iMAG communication */
    uint8_t reserved2[3];       /**< Reserved for further use */
    uint32_t baudrate;          /**< Baud rate of the serial port */
    XCOMFooter footer;
} XCOMParMAG_PORT;
/**
 * This parameter configures the sampling rate of the magnetometer module
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t period;            /**< Sampling period of the magnetometer module in [ms] */
    XCOMFooter footer;
} XCOMParMAG_RATE;
/**
 * This parameter changes the misalignment of the magnetometer reference frame with respect to the IMU reference frame.
 * The angles contained in this parameter are used to set up the IMU to magnetometer misalignment coordinate transformation matrix.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float misalign[3];          /**< Rotation around x,y, and z-axis, respectively in [rad] */
    XCOMFooter footer;
} XCOMParMAG_MISALIGN;
/**
 * This parameter holds the on-board magnetometer calibration procedure status of the 2D magnetometer calibration routine.
 * A more accurate 3D calibration routine is also available. See, for instance, the manual of your iMAG series magnetometer device.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParMAG_CALSTATE
 */
#define PARMAG_CALSTATE_NONE     0    /**< No calibration in progress */
#define PARMAG_CALSTATE_START    1    /**< Start calibration, writable status */
#define PARMAG_CALSTATE_RUNNING  2    /**< Calibration is currently running */
#define PARMAG_CALSTATE_RESERVED 3    /**< Reserved for further use */
#define PARMAG_CALSTATE_BREAK    4    /**< Stop calibration, writable status */
#define PARMAG_CALSTATE_SAMPLE   5    /**< Sample current magnetometer measurement */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    int32_t state;              /**< see PARMAG_CALSTATE_* in appendix */
    XCOMFooter footer;
} XCOMParMAG_CALSTATE;
/**
 * This parameter contains the Figure Of Merit (FOM) of the 2D magnetometer calibration method. It is read only.
 * The lower the resulting FOM value, the better is the qualitative compensation.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float fom;                  /**< Figure of merit (FOM) */
    XCOMFooter footer;
} XCOMParMAG_FOM;
/**
 * This parameter activates/deactivates the external magnetometer
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable;            /**< Magnetometer activation */
    XCOMFooter footer;
} XCOMParMAG_ENABLE;
/**
 * This parameter configures the air data computer module
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable;            /**< Enables/Disables air data computer module */
    XCOMFooter footer;
} XCOMParMADC_ENABLE;
/**
 * This parameter configures the air data computer lever arm
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMParHeader param_header;     /**< XCOM parameter header */
    float lever_arm_xyz[3];         /**< Lever arm in body x, y, z direction in [m] */
    float lever_arm_xyz_stddev[3];  /**< Standard deviation of the lever arm in [m] */
    XCOMFooter footer;
} XCOMParMADC_LEVERARM;
/**
 * This patameter configures the MADC lowpass filter
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float cutoff_freq;          /**< Cutoff frequency in [Hz] */
    uint32_t enable_filter;     /**< Enables/Disables lowpass filter */
    XCOMFooter footer;
} XCOMParMADC_LOWPASS;
/**
 * This parameter configures the GPS quality indicator and the INS/GNSS source selection within the NMEA module.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParNMEA0183_ENABLE
 */
#define PARNMEA_ENABLE_QUALITY_MODE_OFF     0 /**< Use standard NMEA1803 quality mode indicator */
#define PARNMEA_ENABLE_QUALITY_MODE_IMAR    1 /**< Use iMAR’s GPS quality indicator */
#define PARNMEA_ENABLE_SOURCE_INS           0 /**< Use INS/GNSS source */
#define PARNMEA_ENABLE_SOURCE_GNSS          1 /**< Use GNSS source */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t reserved1;          /**< Reserved for further use */
    uint8_t gps_quality_mode;   /**< GPS quality indicator selection */
    uint8_t source_selection;   /**< NMEA0183 source selection (see PARNMEA_ENABLE_SOURCE_* in appendix) */
    uint8_t reserved2;          /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParNMEA0183_ENABLE;
/**
 * This parameter configures the NMEA com port.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t port;               /**< COM port of the NMEA module */
    uint8_t enable;             /**< Enables/Disables NMEA module */
    uint16_t reserved1;         /**< Reserved for further use */
    uint32_t baud_rate;         /**< Baud rate of serial NMEA output */
    XCOMFooter footer;
} XCOMParNMEA0183_COM;
/**
 * This parameter selects the messages to transmit (UDP or UART) within the NMEA module.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParNMEA0183_TXMASK
 */
#define PARNMEA_TXMASK_GGA                (1 << 0)  /**< NMEA-GGA message */
#define PARNMEA_TXMASK_GLL                (1 << 1)  /**< NMEA-GLL message */
#define PARNMEA_TXMASK_GSA                (1 << 2)  /**< NMEA-GSA message */
#define PARNMEA_TXMASK_HDT                (1 << 3)  /**< NMEA-HDT message */
#define PARNMEA_TXMASK_RMC                (1 << 4)  /**< NMEA-RMC message */
#define PARNMEA_TXMASK_VTG                (1 << 5)  /**< NMEA-VTG message */
#define PARNMEA_TXMASK_ZDA                (1 << 6)  /**< NMEA-ZDA message */
#define PARNMEA_TXMASK_PIAHS              (1 << 7)  /**< NMEA-PIAHS message */
#define PARNMEA_TXMASK_PISTATUS1          (1 << 8)  /**< NMEA-PISTATUS1 message */
#define PARNMEA_TXMASK_GST                (1 << 9)  /**< NMEA-GST message */
#define PARNMEA_TXMASK_PAPBN              (1 << 10) /**< NMEA-PAPBN message */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t tx_mask[2];        /**< transmit mask (Idx0: UART; Idx1: UDP; see PARNMEA_TXMASK_* in appendix) */
    XCOMFooter footer;
} XCOMParNMEA0183_TXMASK;
/**
 * This parameter sets the data rate (UDP or UART) within the NMEA module.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParNMEA0183_RATE
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t divisor[2];        /**< prescaler for data output (Idx0: UART; Idx1: UDP) */
    XCOMFooter footer;
} XCOMParNMEA0183_RATE;
/**
 * This parameter configures the UDP transmission within the NMEA module.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t server_addr;       /**< UDP destination address */
    uint32_t port;              /**< UDP destination port */
    uint8_t enable;             /**< Enables/Disables UDP output module */
    uint8_t reserved0;          /**< Reserved for further use */
    uint16_t reserved1;         /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParNMEA0183_UDP;
/**
 * This parameter configures the source of the heading of the VTG message.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;           /**< XCOM header */
    XCOMParHeader param_header;  /**< XCOM parameter header */
    uint32_t selector;           /**< 0: INS/GNSS COG; 1: INS/GNSS Yaw */
    XCOMFooter footer;
} XCOMParNMEA0183_VTGSELECT;
/**
 * This parameter defines the precision of the NMEA0183 protocol.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParNMEA0183_PRECISION
 */
#define PARNMEA0183_PRECISION_MSGSELECT_GGA 0 /**< sets the precision of latitude, longitude, undulation and differential-age for GGA sentence */
#define PARNMEA0183_PRECISION_MSGSELECT_GLL 1 /**< sets the precision of latitude and longitude for GLL sentence */
#define PARNMEA0183_PRECISION_MSGSELECT_RMC 2 /**< sets the precision of latitude and longitude for RMC sentence */
#define PARNMEA0183_PRECISION_MSGSELECT_MAX PARNMEA0183_PRECISION_MSGSELECT_RMC
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;               /**< XCOM header */
    XCOMParHeader param_header;      /**< XCOM parameter header */
    uint8_t message_select;          /**< NMEA sentence selection (must be set to PARNMEA0183_PRECISION_MSGSELECT_GGA,
                                          PARNMEA0183_PRECISION_MSGSELECT_GLL or PARNMEA0183_PRECISION_MSGSELECT_RMC,
                                          see also appendix) */
    uint8_t latitude_precision;      /**< Definition of decimal places for latitude component */
    uint8_t longitude_precision;     /**< Definition of decimal places for longitude component */
    uint8_t undulation_precision;    /**< Definition of decimal places for undulation field (0 = remove dot and decimal places). This field
                                          is only valid for message_select=PARNMEA0183_PRECISION_MSGSELECT_GGA. This field will be ignored if
                                          message_select != PARNMEA0183_PRECISION_MSGSELECT_GGA */
    uint8_t diffage_precision;       /**< Definition of decimal places for diff. age field (0 = remove dot and decimal places). This field is only
                                          valid for message_select=PARNMEA0183_PRECISION_MSGSELECT_GGA.
                                          This field will be ignored if message_select != PARNMEA0183_PRECISION_MSGSELECT_GGA */
    uint8_t reserved[3];             /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParNMEA0183_PRECISION;
/**
 * This parameter defines the local zone description of the ZDA sentence
 * #domain: public
 * #scope: read and write
 * #name: XCOMParNMEA0183_LOCALZONE
 */
#define PARNMEA0183_LOCALZONE_ENABLE       1   /**< Enable local zone description output */
#define PARNMEA0183_LOCALZONE_DISABLE      0   /**< Disable local zone description output */
#define PARNMEA0183_LOCALZONE_HOURS_MIN  (-13) /**< Min allowed local zone hours */
#define PARNMEA0183_LOCALZONE_HOURS_MAX    13  /**< Max allowed local zone hours */
#define PARNMEA0183_LOCALZONE_MINUTES_MIN  0U  /**< Min allowed local zone minutes */
#define PARNMEA0183_LOCALZONE_MINUTES_MAX 59U  /**< Max allowed local zone minutes */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t enable_local_zone;              /**< Enable/Disable local zone description */
    uint8_t reserved[3];                    /**< Reserved for further use */
    int32_t local_zone_hours;               /**< Local time zone offset from GMT, ranging from 00 through ±13 hours */
    uint32_t local_zone_minutes;            /**< Local time zone offset from GMT, ranging from 00 through 59 minutes */
    XCOMFooter footer;
} XCOMParNMEA0183_LOCALZONE;
/**
 * This parameter defines the odometer scale factor in [meter/tick].
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float scf_odo;              /**< Odometer scaling factor in [meter/ticks], user-configured. */
    float scf_odo_est;          /**< Odometer scaling factor in [meter/ticks], EKF-estimated. The EKF slightly adjusts the configured scale factor
                                     to compensate for wear and tear and temperature effects. */
    uint32_t reserved1;         /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParODO_SCF;
/**
 * This parameter defines the odometer timeout in [sec]. If the odometer module does not detect any tick during the configured timeout
 * value PARODO_TIMEOUT, a ZUPT will be performed (only if odometer is enabled via the activation mask of the PAREKF_ZUPT2 parameter.
 * As timeout entry, only values ≥ 0 seconds are accepted.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float timeout;              /**< Odometer timeout in [sec], default: 5.0 */
    XCOMFooter footer;
} XCOMParODO_TIMEOUT;
/**
 * This parameter defines the odometer mode and its related items. For Deglitcher A and B, only values ≤ 255 internal ticks are accepted
 * (an internal tick has typically a value of 25ns). In general, there is no need to change one of these values.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParODO_MODE
 */
#define PARODO_SWITCH_DISABLE         0 /**< Disable odometer module */
#define PARODO_SWITCH_ENABLE          1 /**< Enable odometer module */
#define PARODO_MODE_A_COUNTER         0 /**< Rising edges on channel A are counted as ticks. */
#define PARODO_MODE_A_COUNTER_B_DIR   1 /**< Rising edges on channel A are counted as ticks, the level of B channel at the the tick event indicates direction. */
#define PARODO_MODE_AB_COUNTER_2QUAD  2 /**< Rising and falling edges on channel A are counted as ticks, the level of B channel at the
                                             tick event indicates direction. This doubles the resolution of the odometer compared to mode 1. */
#define PARODO_MODE_AB_COUNTER_4QUAD  3 /**< Rising and falling edges on both channels are counted as ticks, the level of the respective
                                             other channel at the tick event indicates the direction. This quadruples the resolution of
                                             the odometer compared to the first mode. */
#define PARODO_MODE_A_UP_B_DOWN       4 /**< Rising edges on channel A are counted up. Rising edges on channel B are counted down. */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t enable;            /**< Odometer enable/disable switch (see PARODO_SWITCH_* in appendix) */
    uint16_t mode;              /**< Odometer mode (see PARODO_MODE_* in appendix) */
    uint16_t deglitcher[2];     /**< Deglitcher value of channel A/B in [internal ticks], default: 10 */
    XCOMFooter footer;
} XCOMParODO_MODE;
/**
 * This parameter defines the mode and its related items of one or more odometers (see PARODO_MODE2_MAX_ODOMETERS
 * in appendix). For Deglitcher A and B, only values ≤ 255 internal ticks are accepted
 * (an internal tick has typically a value of 25 ns). In general, there is no need to change one of these values.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParODO_MODE2
 */
typedef struct XCOM_STRUCT_PACK {
  uint16_t enable;            /**< Odometer enable/disable switch (see PARODO_SWITCH_* in appendix) */
  uint16_t mode;              /**< Odometer mode (see PARODO_MODE_* in appendix) */
} XCOMParODO_MODE2Type;
#define PARODO_MODE2_MAX_ODOMETERS  3
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    XCOMParODO_MODE2Type odometer[PARODO_MODE2_MAX_ODOMETERS];
    uint16_t deglitcher[2];     /**< Deglitcher value of channel A/B in [internal ticks], default: 10 */
    XCOMFooter footer;
} XCOMParODO_MODE2;
/**
 * This parameter adjusts the timestamp of the odometer data. It can be used to correct for known and constant
 * group or transmission delays.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParODO_LATENCY
 */
#define PARODO_LATENCY_MAX_ODOMETERS 3
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                              /**< XCOM header */
    XCOMParHeader param_header;                     /**< XCOM parameter header */
    float latency[PARODO_LATENCY_MAX_ODOMETERS];    /**< Odometer latency in [sec], default: 0.0 */
    XCOMFooter footer;
} XCOMParODO_LATENCY;
/**
 * This parameter swaps the odometer inputs
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t swap_inputs;       /**< swap A/B inputs */
    XCOMFooter footer;
} XCOMParODO_SWAP;
/**
 * This parameter defines the PPD divider
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t ppd_divider[3];    /**< PPD output divider [ticks] */
    uint16_t ppd_reserved;      /**< reserved for further use */
    XCOMFooter footer;
} XCOMParODO_PPDDIVIDER;
/**
 * This parameter configures the lever arm between the INS and the odometer sensor. The measurement should be done as accurately as
 * possible. The x, y and z values of the parameter represent the vector from the INS enclosure reference point to the odometer sensor.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float lever_arm[3];         /**< Distance between IMU’s center of measurement and odometer sensor in x, y and z direction in [m] */
    XCOMFooter footer;
} XCOMParODO_LEVERARM;
/**
 * This parameter configures the odometer velocity standard deviation used for EKF aiding. The standard deviation entry only accepts
 * values > 0.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float stddev;               /**< Odometer velocity standard deviation in [m/s] */
    XCOMFooter footer;
} XCOMParODO_VELSTDDEV;
/**
 * This parameter configures the odometer mounting direction. The direction vector must be normalized to 1
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float direction[3];         /**< Odometer mounting direction vector for x, y and z-axis */
    XCOMFooter footer;
} XCOMParODO_DIRECTION;
/**
 * This parameter configures the odometer constraints. If the constraints are enabled, the EKF will perform ZUPTS aiding for these
 * axis that are not selected via the PARODO_DIRECTION parameter. The parameter entry for the standard deviation of the odometer constraint
 * only accepts values > 0.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t enable;             /**< 0: Body velocity aiding along PARODO_DIRECTION axis; 1: ZUPT for other axis (default) */
    uint8_t standalone;         /**< Enable/Disable standalone mode for constraints aiding */
    uint16_t reserved2;         /**< Reserved for further use */
    float constraints_stddev;   /**< Standard deviation of the odometer constraints in [m/s], default: 2.0 */
    XCOMFooter footer;
} XCOMParODO_CONSTRAINTS;
/**
 * This parameter configures the odometer velocity sign
 * #domain: public
 * #name: XCOMParODO_VELSIGN
 */
#define PARODO_VELSIGN_MODE_POSITIVE       0 /**< Sets positive odometer velocity */
#define PARODO_VELSIGN_MODE_NEGATIVE       1 /**< Sets negative odometer velocity */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t sign;               /**< Odometer velocity mode (see PARODO_VELSIGN_MODE_* in appendix) */
    uint8_t reserved[3];        /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParODO_VELSIGN;
/**
 * This parameter configures the maximal odometer measurement update rate. The rate entry only accepts values > 0 Hz.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float update_rate;          /**< Maximal measurement update rate in [Hz], default: 1.0 */
    XCOMFooter footer;
} XCOMParODO_RATE;
/**
 * This parameter contains the threshold values. Processing velocity via odometer measurement will be disabled if values exceed these
 * limitations. As threshold parameter entries, only positive values ≥ 0 are allowed
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float thr_acc;              /**< Acceleration threshold in [m/s/s], default: 300.0 */
    float thr_omg;              /**< Angular rate threshold in [rad/s], default: 3.0 */
    XCOMFooter footer;
} XCOMParODO_THR;
/**
 * This parameter configures the odometer inverter for the FPGA288 variants. The default value of this parameter makes the odometer
 * interface compliant to previous generations of FPGA288 hardware.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParODO_INVERTER
 */
#define PARODO_INVERTER_DISABLE 0 /**< Disable input inverter */
#define PARODO_INVERTER_ENABLE  1 /**< Enable input inverter */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< inverter mode */
    XCOMFooter footer;
} XCOMParODO_INVERTER;
/**
 * ARINC825 is currently not supported by the iNAT-FW
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t port;              /**< ARINC825 communication port */
    XCOMFooter footer;
} XCOMParARINC825_PORT;
/**
 * This parameter configures the baud rate of the ARINC825 module
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t baud;              /**< ARINC825 communication baud rate */
    XCOMFooter footer;
} XCOMParARINC825_BAUD;
/**
 * This parameter configures the ARINC825 module
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t backport247;       /**< backport 247 */
    uint16_t tx_enable;         /**< enable data transmission */
    XCOMFooter footer;
} XCOMParARINC825_ENABLE;
/**
 * This parameter configures the ARINC825 transmit objects
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    uint16_t param_id;              /**< XCOM parameter ID */
    uint8_t number_of_frames;       /**< Number of ARINC825 configuration frames */
    uint8_t is_request;             /**< Request/Set parameter */
    struct {
        uint16_t divider;           /**< Output divider */
        uint16_t reserved;          /**< Reserved for further use */
        uint32_t doc;               /**< ARINC825 DOC */
    } frame[XCOM_MAX_NUMBER_OF_ARINC825_FRAMES];
    XCOMFooter footer;
} XCOMParARINC825_LOGLIST;
/**
 * This parameter configures bus recovery
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t bus_recovery;      /**< Enables/Disables bus recovery */
    uint16_t reserved;          /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParARINC825_BUSRECOVERY;
/**
 * This parameter resets the bus statistics
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t reset;             /**< Reset bus */
    uint16_t bus;               /**< Bus selection */
    XCOMFooter footer;
} XCOMParARINC825_RESETSTATUS;
/**
 * This parameter configures the ARINC825 object scaling
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    union {
        XCOMParHeader param_header;
        struct {
            uint16_t param_id;
            uint8_t number_of_scalefactors;
            uint8_t is_request;
        };
    };
    double scf_acc;
    double scf_omg;
    double scf_rpy[3];
    double scf_vel;
    double scf_time;
    double scf_pos[3];
    double scf_rpy_stddev;
    double scf_ins_pos_stddev;
    double scf_vel_stddev;
    double scf_gnss_pos_stddev;
    double scf_side_slip;
    XCOMFooter footer;
} XCOMParARINC825_SCALEFACTOR;
/**
 * This parameter is deprecated.
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMParHeader param_header;
    uint32_t tx_mask;
    uint32_t divider;
    uint32_t base_id;
    uint16_t source;
    uint16_t use_extended_id;
    XCOMFooter footer;
} XCOMParARINC825_GENERALCAN;
/**
 * This parameter is deprecated.
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMParHeader param_header;
    uint32_t protocol_selection;
    XCOMFooter footer;
} XCOMParARINC825_PROTOCOL;
/**
 * This parameter is deprecated.
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMParHeader param_header;
    uint32_t enable;
    XCOMFooter footer;
} XCOMParARINC825_OBDII;
/**
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t event_mask;
    XCOMFooter footer;
} XCOMParARINC825_EVENTMASK;
/**
 * This parameter configures the internal CAN controller.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParCAN_DEVICECONFIG
 */
#define PARCAN_CONFIGOPTION_ENABLE_GATEWAY      0x0001    /**< This option enables the rx processing chain. The received CAN frames
                                                               can be read via XCOM message XCOM_MSGID_CANGATEWAY */
#define PARCAN_CONFIGOPTION_ENABLE_BUSRECOVERY  0x0002    /**< This option enables automatic recovery from Bus-Off State. If the CAN controller
                                                               goes to Bus-Off state due to a massive occurrence of CAN bus errors,
                                                               it stops all bus activities and automatically starts reinitialisation */
enum XCOM_PARCAN_DEVICECONFIG_MODE {
    XCOM_PARCAN_DEVICECONFIG_MODE_NONE      = 0,          /**< The mode disables the CAN device */
    XCOM_PARCAN_DEVICECONFIG_MODE_SIMPLECAN = 1,          /**< This mode enables the SimpleCAN protocol stack */
    XCOM_PARCAN_DEVICECONFIG_MODE_ARINC825  = 2,          /**< This mode is currently not implemented */
    XCOM_PARCAN_DEVICECONFIG_MODE_VRUCAN    = 3           /**< This mode enables the VRU-CAN protocol stack */
};
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t device_idx;         /**< Selection of the internal CAN controller (0 or 1) */
    uint8_t reserved[3];        /**< Reserved for further use */
    uint32_t baud_rate;         /**< Available baud rates: 10K, 20K, 25K, 50K, 100K, 125K, 250K, 500K, 1M */
    uint16_t config_options;    /**< Configuration options */
    uint16_t mode;              /**< CAN operation mode (see XCOM_PARCAN_DEVICECONFIG_MODE_* in appendix) */
    XCOMFooter footer;
} XCOMParCAN_DEVICECONFIG;
/**
 * This parameter configures the SimpleCAN protocol stack.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParCAN_SIMPLECAN
 */
typedef struct {
    uint32_t AngularRateCompensated : 1;    /**<  */
    uint32_t AccelCompensated : 1;          /**<  */
    uint32_t AccelCorrected : 1;            /**<  */
    uint32_t Attitude : 1;                  /**<  */
    uint32_t VelocityEast : 1;              /**<  */
    uint32_t VelocityNorth : 1;             /**<  */
    uint32_t VelocityUp : 1;                /**<  */
    uint32_t Longitude : 1;                 /**<  */
    uint32_t Latitude : 1;                  /**<  */
    uint32_t Altitude : 1;                  /**<  */
    uint32_t AttitudeStdDev : 1;            /**<  */
    uint32_t VelocityStdDev : 1;            /**<  */
    uint32_t PositionStdDev : 1;            /**<  */
    uint32_t UtcData : 1;                   /**<  */
    uint32_t ImuStatus : 1;                 /**<  */
    uint32_t SystemStatus : 1;              /**<  */
    uint32_t PPS : 1;                       /**<  */
    uint32_t VelocityBodyX : 1;             /**<  */
    uint32_t VelocityBodyY : 1;             /**<  */
    uint32_t VelocityBodyZ : 1;             /**<  */
    uint32_t Temperature : 1;               /**<  */
    uint32_t SideslipAngle : 1;             /**<  */
    uint32_t PPS2 : 1;                      /**<  */
    uint32_t CourseOverGround : 1;          /**<  */
    uint32_t InversePathRadius : 1;         /**<  */
    uint32_t OutputFrame : 1;               /**<  */
    uint32_t reserved : 6;                  /**<  */
} ParCAN_SIMPLECAN_TxMaskBitType;
typedef union {
  uint32_t value;
  ParCAN_SIMPLECAN_TxMaskBitType bits;
} ParCAN_SIMPLECAN_TxMaskType;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    ParCAN_SIMPLECAN_TxMaskType tx_mask;  /**< Activation mask */
    uint32_t divider;           /**< Output divider */
    uint32_t base_id;           /**< CAN base ID */
    uint8_t source;             /**< Source selection */
    uint8_t use_extended_id;    /**< Set when extended identifier should be used */
    uint8_t device_idx;         /**< CAN device index */
    uint8_t reserved;           /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParCAN_SIMPLECAN;
/**
 * This parameter configures the VRU-CAN protocol stack.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParCAN_VRUCAN
 */
#define PARCAN_VRUCAN_MAX_BASEID        256 /**< Max allowed base-id */
typedef struct {
    uint32_t Info         : 1;
    uint32_t Acceleration : 1;
    uint32_t AngularRate  : 1;
    uint32_t Attitude     : 1;
    uint32_t reserved     : 28;
} ParCAN_VRUCAN_TxMaskType;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t device_idx;         /**< CAN device index */
    uint8_t reserved[2];        /**< Reserved for further use */
    uint8_t imu_mode;           /**< 0 = Calibrated 1 = Calibrated gyro+acc bias scalefactor 2 = Calibrated gyro bias scalefactor */
    union mask {
        uint32_t value;
        ParCAN_VRUCAN_TxMaskType bits;
    } tx_mask;                  /**< Transmission mask */
    uint32_t base_id;           /**< CAN base ID */
    uint32_t sync_id;           /**< CAN SYNC ID */
    uint32_t emcy_id;           /**< CAN EMCY base ID */
    uint32_t divider;           /**< Output divider */
    float scale_attitude[3];    /**< Scaling of attitude output (roll, pitch, yaw) */
    float scale_accel[3];       /**< Scaling of acceleration output (body x-, y-, z-axis) */
    float scale_omg[3];         /**< Scaling of angular rate output (body x-, y-, z-axis) */
    float scale_temperature;    /**< Scaling of temperature output */
    XCOMFooter footer;
} XCOMParCAN_VRUCAN;
/**
 * This parameter configures the bus termination resistors.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParCAN_BUSTERMINATION
 */
#define PARCAN_BUSTERMINATION_CAN0      0x01    /**< Enables/Disables CAN0 bus termination (120 ohm) */
#define PARCAN_BUSTERMINATION_CAN1      0x02    /**< Enables/Disables CAN1 bus termination (120 ohm) */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t activation_mask;    /**< Bus termination activation mask (see PARCAN_BUSTERMINATION_CAN* in appendix) */
    uint8_t reserved[3];        /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParCAN_BUSTERMINATION;
/**
 * This parameter specifies the behaviour of the recording module.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t xcom_channel;       /**< iXCOM communication channel to be recorded */
    uint8_t enable;             /**< Enable/disable recorder */
    uint16_t reserved;          /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParREC_CONFIG;
/**
 * This parameter is read-only and returns the path to the current log file
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    int8_t file_name[256];      /**< Path to the current log file */
    XCOMFooter footer;
} XCOMParREC_CURRENTFILE;
/**
 * This parameter starts the recording module if the “enable” field of PARREC_CONFIG is set to 0.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParREC_START
 */
#define PARREC_START_BUFFERSIZE 128
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t path[PARREC_START_BUFFERSIZE];  /**< File name of the current log file */
    XCOMFooter footer;
} XCOMParREC_START;
/**
 * This parameter stops the recording module and is write-only.
 * #domain: public
 * #scope: write only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    XCOMFooter footer;
} XCOMParREC_STOP;
/**
 * This parameter contains a user defined suffix string, which will be attached to the default log file name.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParREC_SUFFIX
 */
#define PARREC_SUFFIX_BUFFERSIZE 128
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                          /**< XCOM header */
    XCOMParHeader param_header;                 /**< XCOM parameter header */
    uint8_t suffix[PARREC_SUFFIX_BUFFERSIZE];   /**< User defined log file suffix */
    XCOMFooter footer;
} XCOMParREC_SUFFIX;
/**
 * This parameter enables/disables the rotation of the log files. If this parameter is set, the oldest log file will be deleted first,
 * when the disk is out of space.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable;            /**< Enable/disable log file rotation */
    XCOMFooter footer;
} XCOMParREC_LOGROTATE;
/**
 * This parameter selects the recording device.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParREC_DEVICE
 */
#define PARREC_DEVICE_SDCARD        0   /**< Use sd-card as recording device */
#define PARREC_DEVICE_USBSTICK      1   /**< Use usb-stick as recording device */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t device;             /**< Recording device (0: SD-Card;  1: USB-Stick) */
    uint8_t reserved[3];        /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParREC_DEVICE;
/**
 * This parameter defines the max. file size
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint64_t max_file_size;     /**< Max. file size in [MB]. A new File will be created if max_file_size
                                     is reached, as long as storage space is available. */
    XCOMFooter footer;
} XCOMParREC_FILESIZE;
/**
 * This parameter configures the motion-dependent data recording
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t enable;             /**< Enable/Disable motion dependent data recording */
    uint8_t reserved[3];        /**< Reserved for further use */
    uint32_t motion_delay;      /**< Defines the standstill time needed to pause the system recorder in [s] */
    XCOMFooter footer;
} XCOMParREC_MOTION;
/**
 * This parameter contains the disk space, which is left over on the system e.g. for saving logs.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double free_space;          /**< Free disk space in [MB] */
    XCOMFooter footer;
} XCOMParREC_DISKSPACE;
/**
 * This parameter configures the internal power logging module
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable;            /**< enable/disable power logger */
    XCOMFooter footer;
} XCOMParREC_POWERLOGGER;
/**
 * This parameter configures the auxiliary logging module
 * #domain: hidden
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t port;               /**< serial com port */
    uint8_t enable;             /**< enable/disable module */
    uint16_t reserved;          /**< reserved for further use */
    uint32_t baud_rate;         /**< serial baud rate */
    uint32_t ip_addr;           /**< IP address */
    uint32_t udp_port;          /**< UDP port */
    XCOMFooter footer;
} XCOMParREC_AUXILIARY;
/**
 * This parameter specifies a virtual measurement point. The output position, velocity and acceleration may be transformed to
 * another point with fixed coordinates in the INS enclosure frame with respect to the INS enclosure reference point (e.g. to a wheel or a
 * camera mounted on top of the vehicle).
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_VMP
 */
#define PAREKF_VMP_ALLCHANNELS        0xFF
#define PAREKF_VMP_MASK_POSITION      0x0001
#define PAREKF_VMP_MASK_VELOCITY      0x0002
#define PAREKF_VMP_MASK_SPECIFICFORCE 0x0004
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    union {
        XCOMParHeader param_header;     /**< XCOM parameter header */
        struct {
            uint16_t param_id;
            uint8_t channel_number;     /**< Channel number to configure. If channel_number is set to PAREKF_VMP_ALLCHANNELS, all
                                             channels will be configured with the same settings */
            uint8_t is_request;
        };
    };
    float leverarm[3]; /**< Distance between enclosure reference point and virtual measurement point in x-, y-, and z-direction in [m] */
    uint16_t mask;     /**< Activation mask (see PAREKF_VMP_* in appendix) */
    uint16_t cut_off;  /**< This parameter specifies the cutoff frequency in [Hz]  of the first order lowpass, which is used to
                            filter ω for  the transformation. Due to the noise, the derivative of ω  will not be used in the
                            acceleration transformation. */
    XCOMFooter footer;
} XCOMParEKF_VMP;
/**
 * This parameter defines the used aiding sources of the extended Kalman filter. The measurements of those bits, which are set in the mode
 * field, are used as aiding source. All other data sources are ignored in the data fusion. Per default, all aiding sources are activated,
 * resulting in a aiding mode mask of 0xFF FF FF FF.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_AIDING
 */
#define PAREKF_AIDING_MODEMASK_GNSSPOS     0x00000001 /**< GNSS Position Measurements */
#define PAREKF_AIDING_MODEMASK_BAROALT     0x00000002 /**< Barometer Altitude Measurements */
#define PAREKF_AIDING_MODEMASK_HEIGHT      0x00000004 /**< Height Measurements */
#define PAREKF_AIDING_MODEMASK_GNSSVEL     0x00000008 /**< GNSS Velocity Measurements */
#define PAREKF_AIDING_MODEMASK_BODYVEL     0x00000010 /**< 3D Body Velocity Measurements */
#define PAREKF_AIDING_MODEMASK_ODOMETER    0x00000020 /**< Odometer Measurements */
#define PAREKF_AIDING_MODEMASK_TAS         0x00000040 /**< True Air Speed Measurements */
#define PAREKF_AIDING_MODEMASK_TRUEHDG     0x00000080 /**< True Heading Measurements */
#define PAREKF_AIDING_MODEMASK_MAGHDG      0x00000100 /**< Magnetic Heading Measurements (from Magnetic Field Measurements) */
#define PAREKF_AIDING_MODEMASK_MAGFIELD    0x00000200 /**< Magnetic Field Measurements */
#define PAREKF_AIDING_MODEMASK_BSLXYZ      0x00000400 /**< Baseline XYZ Measurements  (from dual antenna GNSS) */
#define PAREKF_AIDING_MODEMASK_GNSSPSR     0x00000800 /**< GNSS Pseudorange Measurements */
#define PAREKF_AIDING_MODEMASK_GNSSRR      0x00001000 /**< GNSS Range Rate Measurements */
#define PAREKF_AIDING_MODEMASK_TCPD        0x00002000 /**< TCPD Measurements */
#define PAREKF_AIDING_MODEMASK_TCPDDD      0x00004000 /**< TCPD_DD Measurements */
#define PAREKF_AIDING_MODEMASK_ODOALONGTRK 0x00008000 /**< Odometer Along-Track Measurement */
#define PAREKF_AIDING_MODEMASK_EXTPOS      0x00010000 /**< External Position Measurements */
#define PAREKF_AIDING_MODEMASK_EXTVEL      0x00020000 /**< External Velocity Measurements */
#define PAREKF_AIDING_MODEMASK_GRAVITY     0x00040000 /**< Gravity Aiding */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< Bitfield with aiding sources to use */
    uint32_t mask;              /**< An activation mask for changing the AidingMode bitfield. Only these values are changed for which
                                     the mask is enabled (see PAREKF_AIDING_MODEMASK_* in appendix). */
    XCOMFooter footer;
} XCOMParEKF_AIDING;
/**
 * This parameter defines the delay in [ms] of the delayed navigation module against the real-time navigator. Since the aiding data
 * received by the IMS is generally only available delayed, the delay must be handled by a special buffering mechanism. With this parameter,
 * the buffer depth and thus maximum delay can be adjusted. The buffer depth should be kept as low as necessary. Due to the buffer size of
 * the realtime navigator, the delay value is limited to < 490 ms.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t delay;             /**< Delay in [ms] */
    XCOMFooter footer;
} XCOMParEKF_DELAY;
/**
 * This parameter defines the thresholds of the Alignment Status and Position Accuracy fields of the global status. The configurable
 * thresholds specify the flagged position and heading accuracies. The thresholds are internally compared against the standard deviations
 * obtained from the extended Kalman filter. If the heading standard deviation falls below thr_heading during static alignment,
 * alignment will automatically be terminated and the INS is ready to move.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float thr_heading;          /**< Threshold for heading fields of alignment status in [rad] */
    float thr_pos_med;          /**< Threshold for POS_MEDIUM_ACCURACY field of position accuracy in [m] */
    float thr_pos_hi;           /**< Threshold for POS_HIGH_ACCURACY field of position accuracy in [m] */
    XCOMFooter footer;
} XCOMParEKF_HDGPOSTHR;
/**
 * A smoothed position is needed by many control systems. The IMS offers the possibility to smooth the output of the real-time navigation
 * task. The PAREKF_SMOOTH parameter defines the smoothing factor in [samples]. If a position/velocity/attitude jump is detected by the IMS,
 * the jump will be distributed over SMOOTH samples
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t slew_samples;      /**< Smoothing factor in [samples] */
    XCOMFooter footer;
} XCOMParEKF_SMOOTH;
/**
 * The PAREKF_ZUPT parameter contains the configuration of the zero velocity detector. Possible sources for the zero velocity detection are
 * wheel speed measurements, angular rate and acceleration measurements. The respective sources are activated by the Activation mask field
 * (field 14) of this parameter. The angular rate and acceleration measurement are low-pass filtered prior to comparing them to the
 * detection threshold. The cut-off frequency for this low-pass filter may bet set by the CutOffFreq field (field 8) of this parameter. A
 * zero velocity condition is detected if the values of all selected sources have been below their respective threshold for longer than the
 * delay field (field 13) of this parameter. The status of zero velocity detection will be reported in the In-Motion bit of the EXTSYSSTAT
 * field of the SYS_STAT message. A detected zero velocity condition will lead to aiding the EKF with zero velocity updates if the AutoZUPT
 * field (field 15) of this parameter is set. The rate for this zero velocity aiding is determined by the ZUPTRate field (field 9) of this
 * parameter.
 * #domain: deprecated
 * #scope: read and write
 * #name: XCOMParEKF_ZUPT
 */
#define PAREKF_ZUPT_MASK_NONE          0x00 /**< Standstill detection deactivated */
#define PAREKF_ZUPT_MASK_ACCEL         0x01 /**< Acceleration standard deviation */
#define PAREKF_ZUPT_MASK_GYRO          0x02 /**< Angular rate standard deviation */
#define PAREKF_ZUPT_MASK_ODO           0x04 /**< Odometer measurements */
#define PAREKF_ZUPT_MASK_ENC           0x08 /**< Absolute encoder */
#define PAREKF_ZUPT_MASK_NED_VELOCITY  0x10 /**< NED Velocity */
#define PAREKF_ZUPT_MASK_ACCEL_SUM     0x20 /**< Summated Acceleration */
#define PAREKF_ZUPT_MASK_GYRO_SUM      0x40 /**< Summated angular rate */
#define PAREKF_ZUPT_MASK_DOWN_VELOCITY 0x80 /**< NED Down Velocity */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double thr_acc;             /**< Acceleration standard deviation threshold in [m/s2] */
    double thr_omg;             /**< Angular Rate standard deviation threshold in [rad/s] */
    double thr_vel;             /**< Odometer velocity threshold in [m/s] */
    float cutoff;               /**< Accelerations and angular rates are low-pass filtered and the consequential data will be compared
                                     to the configured thresholds. The parameter unit is [Hz] */
    float zupt_period;          /**< ZUPT interval in [s] */
    float min_stddev_zupt;      /**< Final ZUPT StdDev in [m/s]. Starting with a higher  standard deviation, it will decrease until
                                     hitting  this limitation. */
    float weighting_factor;     /**< Weighting factor of ZUPT StdDev filter */
    float time_constant;        /**< Time constant in [s] */
    uint16_t delay;             /**< Delay in [ms] */
    uint8_t mask;               /**< Activation mask */
    uint8_t automatic_zupt;     /**< Automatic ZUPT aiding */
    XCOMFooter footer;
} XCOMParEKF_ZUPT;
/**
 * The PAREKF_ZUPT2 parameter contains the configuration of the zero velocity detector. Possible sources for the zero velocity detection are
 * wheel speed measurements, angular rate and acceleration measurements. The respective sources are activated by the Activation Mask field
 * (mask) of this parameter. The angular rate and acceleration measurement are low-pass filtered prior to comparing them to the
 * detection threshold. The cut-off frequency for this low-pass filter may be set by the cutoff field of this parameter. A zero velocity
 * condition is detected if the values of all selected sources have been below their respective threshold for longer than the delay field
 * of this parameter. The status of zero velocity detection will be reported in the In-Motion bit of the EXTSYSSTAT field of the SYS_STAT
 * message. A detected zero velocity condition will lead to aiding the EKF with zero velocity updates if the automatic_zupt field of this
 * parameter is set. The rate for this zero velocity aiding is determined by the zupt_period field of this parameter.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_ZUPT2
 */
#define PAREKF_ZUPT2_MASK_NONE          0x00 /**< Standstill detection deactivated */
#define PAREKF_ZUPT2_MASK_ACCEL         0x01 /**< Acceleration standard deviation */
#define PAREKF_ZUPT2_MASK_GYRO          0x02 /**< Angular rate standard deviation */
#define PAREKF_ZUPT2_MASK_ODO           0x04 /**< Odometer measurements */
#define PAREKF_ZUPT2_MASK_ENC           0x08 /**< Absolute encoder */
#define PAREKF_ZUPT2_MASK_NED_VELOCITY  0x10 /**< NED Velocity */
#define PAREKF_ZUPT2_MASK_ACCEL_SUM     0x20 /**< Summated Acceleration */
#define PAREKF_ZUPT2_MASK_GYRO_SUM      0x40 /**< Summated angular rate */
#define PAREKF_ZUPT2_MASK_DOWN_VELOCITY 0x80 /**< NED Down Velocity */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                 /**< XCOM header */
    XCOMParHeader param_header;        /**< XCOM parameter header */
    double thr_acc;                    /**< Acceleration standard deviation threshold in [m/s2] */
    double thr_omg;                    /**< Angular Rate standard deviation threshold in [rad/s] */
    double thr_vel;                    /**< Odometer velocity threshold in [m/s] */
    float cutoff;                      /**< Accelerations and angular rates are low-pass filtered and the consequential data will be compared
                                            to the configured thresholds. The parameter unit is [Hz] */
    float zupt_period;                 /**< ZUPT interval in [s] */
    float min_stddev_zupt;             /**< Final ZUPT StdDev in [m/s]. Starting with a higher  standard deviation, it will decrease until
                                            hitting  this limitation. */
    float weighting_factor;            /**< Weighting factor of ZUPT StdDev filter. Plays a role in ZUPT StdDev calculation, together with
                                            time_constant, if a standstill is detected, to avoid a hard switch to the
                                            configured ZUPT StdDev (min_stddev_zupt) */
    float time_constant;               /**< Time constant in [s]. Plays a role in ZUPT StdDev calculation, together with
                                            weighting_factor, if a standstill is detected, to avoid a hard switch to the
                                            configured ZUPT StdDev (min_stddev_zupt) */
    float velocity_stddev_factor;      /**< Velocity standard deviation factor */
    float summated_rate_limit;         /**< summated angular rate Limit in [rad/s] */
    float summated_acceleration_limit; /**< summated acceleration Limit in [m/s/s] */
    uint16_t delay;                    /**< Delay in [ms] */
    uint8_t automatic_zupt;            /**< Automatic ZUPT aiding */
    uint8_t reserved;                  /**< reserved for further use */
    uint32_t mask;                     /**< Activation mask (see PAREKF_ZUPT2_MASK_* in appendix) */
    XCOMFooter footer;
} XCOMParEKF_ZUPT2;
/**
 * This parameter controls the internal forced ZUPT module. The forced ZUPT module has a higher priority than the motion detector.
 * This means in case the motion detector detects motion, the forced ZUPT module is able to force the Kalman filter to perform zero velocity
 * updates.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_FORCEDZUPT
 */
#define PAREKF_FORCEDZUPT_DISABLE 0     /**< Enable forced ZUPTs */
#define PAREKF_FORCEDZUPT_ENABLE  1     /**< Disable forced ZUPTs */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t enable;             /**< enable/disable forced zero velocity updates. The internal motion detector has a lower priority
                                     compared to the forced zupt mechanism */
    uint8_t reserved[3];        /**< reserved for further use */
    XCOMFooter footer;
} XCOMParEKF_FORCEDZUPT;
/**
 * This parameter defines the outlier rejection mask of the integrated Kalman filter. The outliers of the measurements selected in the mode
 * field, are detected and excluded from EKF aiding. Per default, all outlier detections are activated, resulting in a outlier mode mask
 * of 0xFF FF FF FF.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_OUTLIER
 */
#define PAREKF_OUTLIER_MODEMASK_GNSSPOS     0x00000001 /**< GNSS Position Measurements */
#define PAREKF_OUTLIER_MODEMASK_BAROALT     0x00000002 /**< Barometer Altitude Measurements */
#define PAREKF_OUTLIER_MODEMASK_HEIGHT      0x00000004 /**< Height Measurements */
#define PAREKF_OUTLIER_MODEMASK_GNSSVEL     0x00000008 /**< GNSS Velocity Measurements */
#define PAREKF_OUTLIER_MODEMASK_BODYVEL     0x00000010 /**< 3D Body Velocity Measurements */
#define PAREKF_OUTLIER_MODEMASK_ODOMETER    0x00000020 /**< Odometer Measurements */
#define PAREKF_OUTLIER_MODEMASK_TAS         0x00000040 /**< True Air Speed Measurements */
#define PAREKF_OUTLIER_MODEMASK_DUALANTHDG  0x00000080 /**< Heading Measurements (from dual antenna GNSS) */
#define PAREKF_OUTLIER_MODEMASK_MAGHDG      0x00000100 /**< Magnetic Heading Measurements (from Magnetic Field Measurements) */
#define PAREKF_OUTLIER_MODEMASK_MAGFIELD    0x00000200 /**< Magnetic Field Measurements */
#define PAREKF_OUTLIER_MODEMASK_BSLXYZ      0x00000400 /**< Baseline XYZ Measurements */
#define PAREKF_OUTLIER_MODEMASK_GNSSPSR     0x00000800 /**< GNSS Pseudorange Measurements */
#define PAREKF_OUTLIER_MODEMASK_GNSSRR      0x00001000 /**< GNSS Range Rate Measurements */
#define PAREKF_OUTLIER_MODEMASK_TCPD        0x00002000 /**< TCPD Measurements */
#define PAREKF_OUTLIER_MODEMASK_TCPDDD      0x00004000 /**< TCPD_DD Measurements */
#define PAREKF_OUTLIER_MODEMASK_ODOALONGTRK 0x00008000 /**< Odometer Along-Track Measurement */
#define PAREKF_OUTLIER_MODEMASK_EXTPOS      0x00010000 /**< External Position Measurements */
#define PAREKF_OUTLIER_MODEMASK_EXTVEL      0x00020000 /**< External Velocity Measurements */
#define PAREKF_OUTLIER_MODEMASK_GRAVITY     0x00040000 /**< Gravity Aiding */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< Outlier selection */
    uint32_t mask;              /**< An activition mask for changing the 'mode' bitfield. Only these values are changed for
                                     which the mask is enabled (see PAREKF_OUTLIER_MODEMASK_* in appendix). */
    XCOMFooter footer;
} XCOMParEKF_OUTLIER;
/**
 * This parameter can be used to define the initialization behavior of the  Kalman filter. A state chart detailing the navigation
 * startup process is available in appendix.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_STARTUPV2
 */
#define PAREKF_STARTUPV2_POSMODE_GNSS        0 /**< The initial position will be set via the GNSS solution. If no valid GNSS data are
                                                    available, the position will be loaded from the internal memory, when the GNSS
                                                    timeout has expired. If the INS cannot load the stored position, the indicator
                                                    'EKF SavedPos Error' will be set and the default position at PAREKF_DEFPOS will
                                                    be loaded (see behaviour at 'STOREDPOS'). */
#define PAREKF_STARTUPV2_POSMODE_STORED      1 /**< The initial position will be loaded from the internal storage. If the IMS cannot
                                                    load the saved position, the indicator 'EKF SavedPos Error' will be set.
                                                    The default position PAREKF_DEFPOS will be loaded as the best of all worse
                                                    solutions. */
#define PAREKF_STARTUPV2_POSMODE_FORCED      2 /**< The transmitted position will be used to set the initial position of the Kalman
                                                    filter. If the transmitted position is out of range, the stored position will be
                                                    loaded from the internal memory. If the IMS cannot load the stored position,
                                                    the indicator 'EKF SavedPos Error' will be set and the default position
                                                    PAREKF_DEFPOS will be used (see behaviour at 'STOREDPOS'). */
#define PAREKF_STARTUPV2_POSMODE_CURRENT     3 /**< The current position (last result of the navigation filter) will be used as
                                                    initial position. This option should only be selected for a realignment. */
#define PAREKF_STARTUPV2_HDGMODE_UNKNOWN     0 /**< The initial heading is unknown */
#define PAREKF_STARTUPV2_HDGMODE_STORED      1 /**< The initial heading will be loaded from the internal storage. If the IMS cannot
                                                    load the saved heading, the indicator 'EKF SavedHdg Error' will be set. In this
                                                    case, the IMS will be set to the default heading 'DEFAULTHDG'. */
#define PAREKF_STARTUPV2_HDGMODE_FORCED      2 /**< The initial heading will be transmitted via the PAREKF_STARTUP parameter.
                                                    If the IMS does not receive a valid heading, the system will answer with an error
                                                    message and the heading will be set to 'STOREDHDG'. */
#define PAREKF_STARTUPV2_HDGMODE_MAG         3 /**< The initial heading will be set by the magnetometer value (magnetic heading).
                                                    If the magnetic is not available, the heading will be set to 'STOREDHDG'. */
#define PAREKF_STARTUPV2_HDGMODE_DUALANT     4 /**< The initial heading will be set by the dual antenna GNSS receiver with the first
                                                    valid GNSS sample. Until the GNSS data are valid, the heading will remain
                                                    uninitialized; after the GNSS timeout has expired without a valid solution,
                                                    the 'STOREDHDG' will be used. */
#define PAREKF_STARTUPV2_ALTMSL_DISABLE      0 /**< Disabled */
#define PAREKF_STARTUPV2_ALTMSL_ENABLE       1 /**< Enabled, regarding geoid undulation */
#define PAREKF_STARTUPV2_RESTART_DISABLE     0 /**< Disable automatic restart */
#define PAREKF_STARTUPV2_RESTART_ENABLE      1 /**< Enable automatic restart */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double initpos_lon;         /**< Initial longitude position in WGS84 in [rad] */
    double initpos_lat;         /**< Initial latitude position in WGS84 in [rad] */
    float initpos_alt;          /**< Initial altitude in WGS84 in [m] */
    float initpos_stddev[3];    /**< Standard deviation of initial position and altitude in [m] */
    float inithdg;              /**< Initial heading in [rad] (in NED) Range: ±π */
    float inithdg_stdddev;      /**< Standard deviation of initial heading in [rad] */
    float lever_arm[3];         /**< Lever arm in x-, y- and z-direction in [m] */
    float lever_arm_stddev[3];  /**< Lever arm standard deviation in x-, y- and z-direction in [m] */
    uint8_t position_mode;      /**< Position initialization mode (see PAREKF_STARTUPV2_POSMODE_* in appendix). */
    uint8_t hdg_mode;           /**< Heading initialization mode (see PAREKF_STARTUPV2_HDGMODE_* in appendix). */
    uint16_t gnss_timeout;      /**< GNSS timeout in [sec] Set to ’0’ to wait forever */
    uint8_t enable_alt_msl;     /**< Alternative Mean Sea Level (see PAREKF_STARTUPV2_ALTMSL_* in appendix). */
    uint8_t realignment;        /**< 0: Store parameters without execution of an alignment;
                                     1: Start alignment immediately with the transmitted alignment parameters */
    uint8_t forced_inmotion;    /**< During alignment, system is
                                     0: Static;
                                     1: In-motion */
    uint8_t automatic_restart;  /**< The levelling will be restarted if movement of the system is detected during the levelling
                                     phase. The fine alignment will be finished if movement of the system is detected
                                     (see PAREKF_STARTUPV2_RESTART_* in appendix). */
    XCOMFooter footer;
} XCOMParEKF_STARTUPV2;
/**
 * #domain: deprecated
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double initpos_lon;         /**< Initial longitude position in WGS84 in [rad] */
    double initpos_lat;         /**< Initial latitude position in WGS84 in [rad] */
    float initpos_alt;          /**< Initial altitude in WGS84 in [m] */
    float initpos_stddev[3];    /**< Standard deviation of initial position and altitude in [m] */
    float inithdg;              /**< Initial heading in [rad] (in NED) Range: ±π */
    float inithdg_stdddev;      /**< Standard deviation of initial heading in [rad] */
    uint8_t position_mode;      /**< Position initialization mode */
    uint8_t hdg_mode;           /**< Heading initialization mode */
    uint16_t gnss_timeout;      /**< GNSS timeout in [sec] Set to ’0’ to wait forever */
    uint8_t reserved;           /**< reserved for further use */
    uint8_t realignment;        /**< 0: Store parameters without execution of an alignment;
                                     1: Start alignment immediately with the transmitted alignment parameters */
    uint8_t forced_inmotion;    /**< During alignment, system is
                                     0: Static;
                                     1: In-motion */
    uint8_t automatic_restart;  /**< The levelling will be restarted if movement of the system is detected during the levelling
                                     phase. The fine alignment will be finished if movement of the system is detected */
    XCOMFooter footer;
} XCOMParEKF_STARTUP;
/**
 * This parameter defines the default position. The circumstances under which the default position will be loaded are detailed in the state
 * charts in appendix. The parameter entries only accept values of a certain range, for a detailed description see the PAREKF_STARTUPV2
 * parameter.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double lon;                 /**< Default longitude position in WGS84 in [rad] */
    double lat;                 /**< Default latitude position in WGS84 in [rad] */
    float alt;                  /**< Default altitude in [m] */
    XCOMFooter footer;
} XCOMParEKF_DEFPOS;
/**
 * This parameter defines the default heading. The default heading will be loaded, if the stored heading is invalid.
 * The parameter entries only accept values of a certain range, for a detailed description see the PAREKF_STARTUPV2 parameter
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float default_hdg;          /**< Default heading in NED frame in [rad] */
    XCOMFooter footer;
} XCOMParEKF_DEFHDG;
/**
 * This parameter defines the power-down behavior of the system. If the 'store_state' field is set to "1", the systems will store its current
 * position and attitude state to the internal storage. The stored position/attitude can be loaded during the next alignment.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_POWERDOWN
 */
#define PAREKF_POWERDOWN_STATE_NOTSTORE 0 /**< Do not save position and attitude */
#define PAREKF_POWERDOWN_STATE_STORE    1 /**< Save position and attitude to the internal storage */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t store_state;       /**< power-down behavior of the system */
    XCOMFooter footer;
} XCOMParEKF_POWERDOWN;
/**
 * This parameter is read-only and contains the earth radii.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float radii[2];             /**< Earth radii for M and N configuration in [m]. */
    XCOMFooter footer;
} XCOMParEKF_EARTHRAD;
/**
 * This read-only parameter provides the stored position and the associated standard deviation.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double lon;                 /**< Longitude in [rad] */
    double lat;                 /**< Latitude in [rad] */
    double alt;                 /**< Altitude in [m] */
    double lon_stddev;          /**< Longitude standard deviation in [m] */
    double lat_stddev;          /**< Latitude standard deviation in [m] */
    double alt_stddev;          /**< Altitude standard deviation in [m] */
    XCOMFooter footer;
} XCOMParEKF_STOREDPOS;
/**
 * This parameter is read-only and contains the stored attitude.
 * #domain: public
 * #scope: read only
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float rpy[3];               /**< Stored angels for roll, pitch and yaw in [rad] */
    float rpy_stddev[3];        /**< Standard deviation of stored angels roll, pitch and yaw in [rad] */
    XCOMFooter footer;
} XCOMParEKF_STOREDATT;
/**
 * This parameter configures the standard deviation threshold of the position aiding module. GNSS solutions with a standard deviation greater than this
 * threshold will be discarded from aiding. The standard deviation value only accepts positive values > 0.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double thr_stddev_pos;      /**< Standard deviation threshold in [m] of the position aiding module. If the standard deviation is greater than
                                     the configured threshold, the measurement update will be refused. */
    XCOMFooter footer;
} XCOMParEKF_POSAIDSTDDEVTHR;
/**
 * This parameter provides advanced odometer configuration options which can be used to accommodate for odometer signal abnormalities.
 * Please contact iMAR support (support@imar-navigation.de) to assist you with the right settings should there be any problems with your signal.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_ODOMETER
 */
#define PAREKF_ODOMETER_INNOAVG_DISABLE    0 /**< Disable innovation averaging */
#define PAREKF_ODOMETER_INNOAVG_ENABLE     1 /**< Enable innovation averaging (default) */
#define PAREKF_ODOMETER_COARSCALIB_DISABLE 0 /**< Disable coarse calibration mode */
#define PAREKF_ODOMETER_COARSCALIB_ENABLE  1 /**< Enable coarse calibration mode */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    float scale_factor_error;               /**< Scale factor error in [%] */
    float scale_factor_stddev;              /**< Scale factor error standard deviation in [%] */
    float scale_factor_rw;                  /**< Random walk scale factor in [1/s/sqrt(Hz)] */
    float misalignment[2];                  /**< Start values of misalignment in [rad] */
    float misalignment_stddev[2];           /**< Standard deviation of start values of misalignment in [rad] */
    float misalignment_rw[2];               /**< Random walk misalignment in [rad/s/sqrt(Hz)] */
    float min_velocity;                     /**< Minimum velocity in [m/s], default: 0.1 */
    float max_velocity;                     /**< Maximum velocity in [m/s], default: 100 */
    uint8_t inno_avg_enable;                /**< Innovation averaging */
    uint8_t misalignment_estimation_enable; /**< Enable/Disable odometer misalignment estimation */
    uint8_t coarse_calib_enable;            /**< Coarse calibration mode */
    uint8_t reserved2;                      /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParEKF_ODOMETER;
/**
 * This parameter configures the odometer data processing module for railway applications. By enabling this parameter, the carriage box to
 * bogie misalignment angles which occur in curves, will be compensated. The distance between the two bogies of the carriage box where the
 * INS is installed has to be configured. As Bogie Distance, only non-negative values ≥ 0 are accepted.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_ODOBOGIE
 */
#define PAREKF_ODOBOGIE_COMP_DISABLE 0 /**< Disable bogie compensation */
#define PAREKF_ODOBOGIE_COMP_ENABLE  1 /**< Enable bogie compensation */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMParHeader param_header;         /**< XCOM parameter header */
    float bogie_distance;               /**< Bogie distance in [m] */
    uint32_t enable_bogie_compensation; /**< Enable/Disable bogie compensation */
    XCOMFooter footer;
} XCOMParEKF_ODOBOGIE;
/**
 * This parameter configures GNSS lever arm estimation. GNSS lever arm estimation is only possible if the GNSS receiver supports RTK
 * operation and is able to fix the RTK solution. RTK capability can be checked with the PARGNSS_MODEL parameter.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_GNSSLEVERARMEST
 */
#define PAREKF_GNSSLEVERARMEST_PRIM_DISABLE 0 /**< Disable lever arm estimation for primary antenna */
#define PAREKF_GNSSLEVERARMEST_PRIM_ENABLE  1 /**< Enable lever arm estimation for primary antenna */
#define PAREKF_GNSSLEVERARMEST_SEC_DISABLE  0 /**< Disable lever arm estimation for secondary antenna */
#define PAREKF_GNSSLEVERARMEST_SEC_ENABLE   1 /**< Enable lever arm estimation for secondary antenna */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t primary_antenna;   /**< Primary antenna configuration */
    uint16_t secondary_antenna; /**< Secondary antenna configuration */
    XCOMFooter footer;
} XCOMParEKF_GNSSLEVERARMEST;
/**
 * This parameter configures odometer lever arm estimation.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t lever_arm_estimation_enable;    /**< Enable/Disable odometer lever arm estimation */
    uint8_t reserved[3];                    /**< Reserved for further use */
    float lever_arm_stddev[3];              /**< Odometer lever arm standard deviation in [m] */
    float lever_arm_rw;                     /**< Odometer lever arm random-walk in [m/s/sqrt(Hz)] */
    XCOMFooter footer;
} XCOMParEKF_ODOLEVERARMEST;
/**
 * This parameter configures the reference frame for GNSS RTK updates.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_REFFRAME
 */
#define PAREKF_REFFRAME_UNKNOWN     0       /**< Reference frame is unknown */
#define PAREKF_REFFRAME_WGS84       1       /**< Reference frame is WGS84 */
#define PAREKF_REFFRAME_ETRF2014    2       /**< Reference frame is ETRF2014 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t reference_frame;                /**< Reference frame (see PAREKF_REFFRAME_* in appendix)*/
    uint8_t reserved[3];                    /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParEKF_REFFRAME;
/**
 * This parameter configures the reference frame for external position updates.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_EXTAIDREFFRAME
 */
#define PAREKF_EXTAIDREFFRAME_UNKNOWN     0       /**< Reference frame is unknown */
#define PAREKF_EXTAIDREFFRAME_WGS84       1       /**< Reference frame is WGS84 */
#define PAREKF_EXTAIDREFFRAME_ETRF2014    2       /**< Reference frame is ETRF2014 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t reference_frame;                /**< Reference frame (see PAREKF_EXTAIDREFFRAME_* in appendix) */
    uint8_t reserved[3];                    /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParEKF_EXTAIDREFFRAME;
/**
 * This parameter configures the automatic height updates
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t enable;                         /**< Enable/disable automatic height updates */
    uint8_t enable_frozen_height_update;    /**< Enable/disable frozen height updates */
    uint8_t reserved[2];                    /**< Reserved for further use */
    double height_stddev;                   /**< Height standard deviation in [m] */
    double aiding_interval;                 /**< Aiding interval in [sec] */
    double timeout;                         /**< Aiding timeout in [sec] */
    XCOMFooter footer;
} XCOMParEKF_AUTOHEIGHT;
/**
 * This parameter configures the automatic ZUPT request feature.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_ZUPTREQUEST
 */
#define PAREKF_ZUPTREQUEST_MASK_POSITION    0x00000001  /**< Enables/Disables position test */
#define PAREKF_ZUPTREQUEST_MASK_VELOCITY    0x00000002  /**< Enables/Disables velocity test */
#define PAREKF_ZUPTREQUEST_MASK_DISTANCE    0x00000004  /**< Enables/Disables distance test */
#define PAREKF_ZUPTREQUEST_MASK_TIME        0x00000008  /**< Enables/Disables time test */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    float pos_stddev_thr;                   /**< Threshold for position standard deviation test in [m] */
    float vel_stddev_thr;                   /**< Threshold for velocity standard deviation test in [m/s] */
    float distance_thr;                     /**< Threshold for distance test in [m] */
    float time_thr;                         /**< Threshold for time test in [s] */
    uint32_t activation_mask;               /**< see PAREKF_ZUPTREQUEST_MASK_* in appendix */
    XCOMFooter footer;
} XCOMParEKF_ZUPTREQUEST;
/**
 * This parameter configures the automatic position update request feature.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_POSUPDATEREQUEST
 */
#define PAREKF_POSUPDATEREQUEST_MASK_POSITION    0x00000001  /**< Enables/Disables position test */
#define PAREKF_POSUPDATEREQUEST_MASK_VELOCITY    0x00000002  /**< Enables/Disables velocity test */
#define PAREKF_POSUPDATEREQUEST_MASK_DISTANCE    0x00000004  /**< Enables/Disables distance test */
#define PAREKF_POSUPDATEREQUEST_MASK_TIME        0x00000008  /**< Enables/Disables time test */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    float pos_stddev_thr;                   /**< Threshold for position standard deviation test in [m] */
    float vel_stddev_thr;                   /**< Threshold for velocity standard deviation test in [m/s] */
    float distance_thr;                     /**< Threshold for distance test in [m] */
    float time_thr;                         /**< Threshold for time test in [s] */
    uint32_t activation_mask;               /**< see PAREKF_POSUPDATEREQUEST_MASK_* in appendix */
    XCOMFooter footer;
} XCOMParEKF_POSUPDATEREQUEST;
/**
 * This parameter configures the verification process during stored heading alignment
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t verified_mode;                  /**< Enable/Disable verification mode */
    uint8_t reserved[3];                    /**< Reserved for further use */
    double verification_time;               /**< Max. verification time in [s] */
    double heading_stddev_thr;              /**< Threshold for heading standard deviation test in [rad] */
    XCOMFooter footer;
} XCOMParEKF_STOREDALIGNMODE;
/**
 * This parameter configures the GNSS aiding rates inside the Kalman filter.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t psrpos_period;     /**< Aiding interval pseudorange position in [sec] */
    uint16_t psrvel_period;     /**< Aiding interval pseudorange velocity in [sec] */
    uint16_t rtk_period;        /**< Aiding interval RTK in [sec] */
    uint16_t rtk_timeout;       /**< Aiding interval RTK timeout in [sec]. If RTK is still unavailable after this timeout, PSR position will be used. */
    uint16_t hdg_period;        /**< Dual antenna heading update interval in [sec] */
    uint16_t zupt_timeout;      /**< Zero Velocity Updates will be done after this timeout in [sec] which reduces the aiding rate of position updates at a standstill. */
    XCOMFooter footer;
} XCOMParEKF_GNSSAIDRATE;
/**
 * This parameter configures the PDOP threshold inside the Kalman filter. GNSS solutions with a PDOP worse than this threshold will be
 * discarded from aiding. As PDOP threshold only positive values > 0 are accepted.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float pdop;                 /**< PDOP Threshold */
    XCOMFooter footer;
} XCOMParEKF_GNSSPDOP;
/**
 * This parameter configures the dual antenna aiding inside the Kalman filter.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_DUALANTAID
 */
#define EKF_DUALANTAID_MODE_INIT        0 /**< Initialization mode: The GNSS heading will be used to set the initial heading. */
#define EKF_DUALANTAID_MODE_INTERVAL    1 /**< Interval mode: The GNSS heading aiding will be performed with a fixed rate. */
#define EKF_DUALANTAID_MODE_AUTO        2 /**< Automatic mode (recommended): The GNSS heading aiding will be  performed with a fixed rate
                                               until the INS yaw standard deviation reaches the configured threshold. */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMParHeader param_header;     /**< XCOM parameter header */
    float threshold_heading_stddev; /**< GNSS heading standard deviation threshold in [rad]. If the received GNSS heading standard deviation is
                                         larger than the configured threshold, the measurement update will not be performed. */
    float threshold_pitch_stddev;   /**< GNSS pitch standard deviation threshold in [rad]. If the received GNSS pitch
                                         standard deviation is larger than the configured threshold, the measurement update will not be performed. */
    float threshold_ins_yaw_stddev; /**< INS yaw standard deviation threshold in [rad]. If aiding mode 2 is configured, GNSS heading updates
                                         will be performed until the yaw standard deviation reaches the configured threshold. */
    uint32_t aiding_mode;           /**< The configured aiding mode (see EKF_DUALANTAID_MODE_* in appendix). */
    XCOMFooter footer;
} XCOMParEKF_DUALANTAID;
/**
 * This parameter configures the iMAG parameters inside the Kalman filter.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_MAGATTAID
 */
#define EKF_MAGATTAID_MODE_INIT     0 /**< Initialization mode: The iMAG heading will be used to set the initial heading */
#define EKF_MAGATTAID_MODE_INTERVAL 1 /**< Interval mode: The iMAG heading aiding will be performed with a fixed rate. */
#define EKF_MAGATTAID_MODE_AUTO     2 /**< Automatic mode (recommended): The iMAG heading aiding will be performed with a fixed rate until
                                           the INS yaw standard. deviation reaches the configured threshold */
#define EKF_MAGATTAID_MODE_NAVAID   3 /**< Navigation mode: iMAG heading aiding will be performed once the EKF was able to determine initial
                                           heading to an accuracy better than 2 degrees (0.03491 rad). This prevents faulty initialization
                                           in case of possible unknown magnetic disturbances. */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t sample_period;     /**< Sampling period of the magnetometer measurements in [ms] */
    float hdg_stddev;           /**< Standard deviation of the magnetometer heading aiding in [rad] */
    float latency;              /**< Latency of magnetometer measurement in [s] */
    float ins_yaw_thr;          /**< Threshold of the integrated heading solution (used for automatic aiding mode) in [rad] */
    uint8_t aiding_mode;        /**< Configures the aiding behavior (see EKF_DUALANTAID_MODE_* in appendix). */
    uint8_t update_mode;        /**< EKF update mode (see EKF_MAGATTAID_MODE_* in appendix) */
    uint16_t aiding_interval;   /**< Magnetometer aiding interval in [s] */
    float mag_field_stddev[3];  /**< Magnetic field standard deviation in [mG] */
    XCOMFooter footer;
} XCOMParEKF_MAGATTAID;
/**
 * This parameter configures the iMADC parameters inside the Kalman filter.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float altitude_stddev;      /**< Altitude standard deviation in [m] */
    float latency;              /**< Latency in [sec] */
    uint16_t aiding_interval;   /**< Aiding interval in [sec] */
    uint16_t reserved;          /**< Reserved for further use */
    float scale_factor_error;   /**< Baro scale factor error */
    float scale_factor_stddev;  /**< Baro scale factor error standard devation */
    float scale_factor_rw;      /**< Random walk of scale factor in [1/s/sqrt(Hz)] */
    float bias;                 /**< Barometric offset in [m] */
    float bias_stddev;          /**< Barometric offset standard deviation in [m] */
    float bias_rw;              /**< Random walk of barometric offset in [m/s/sqrt(Hz)] */
    XCOMFooter footer;
} XCOMParEKF_MADCAID;
/**
 * This parameter configures the alignment method of the INS. The default mode is different for every device and should only be changed by advanced users.
 *
 * The INS supports two general modes of alignment:
 *
 * Stationary alignment:
 * Requires the INS to be in a stationary condition during alignment (small disturbances e.g. due to wind gusts are allowed) The
 * alignment starts with a levelling phase with a duration controlled by the LevellingDuration field of this parameter. Afterwards, zero velocity updates
 * are performed to refine the estimates of attitude (and heading for device with gyro compassing capability). After a duration controlled by the
 * StationaryDuration field of this parameter, these automated zero velocity updates are terminated and the INS may be moved. The standard deviation in
 * field 7 of this parameter may be adjusted for e.g. high vibration environments.
 *
 * In-Motion alignment:
 * Does not impose any requirements for the motion experienced by the INS during alignment, i.e. the INS may be in motion during
 * alignment. GNSS position and velocity have to be available during alignment. The Alignment routine automatically completes once heading and attitude have
 * been determined with sufficient accuracy (depending on the device and trajectory dynamics, the attitude standard deviations for all axes have to be
 * between 0 and 5 degrees (0 and 0.08727 rad)). After transition to navigation mode, heading and attitude will be refined further.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_ALIGNMENT
 */
#define EKF_ALIGNMENT_METHOD_STATIONARY     0 /**< Stationary alignment mode */
#define EKF_ALIGNMENT_METHOD_DYNAMIC        1 /**< Dynamic alignment mode */
#define EKF_ALIGNMENT_GYROAVG_DISABLE       0 /**< Disable gyro averaging */
#define EKF_ALIGNMENT_GYROAVG_ENABLE        1 /**< Enable gyro averaging */
#define EKF_ALIGNMENT_TRACKALIGN_DISABLE    0 /**< Disable track alignment */
#define EKF_ALIGNMENT_TRACKALIGN_ENABLE     1 /**< Enable track alignment */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                 /**< XCOM header */
    XCOMParHeader param_header;        /**< XCOM parameter header */
    uint32_t align_method;             /**< Alignment mode */
    uint16_t levelling_duration;       /**< Levelling duration in [sec] */
    uint16_t stationaryalign_duration; /**< Stationary duration in [sec] */
    double align_zupt_stddev;          /**< Alignment ZUPT standard deviation in [m/s] */
    uint8_t enable_gyro_avg;           /**< Gyro averaging */
    uint8_t enable_track_align;        /**< Track alignment */
    float track_align_thr;             /**< Track alignment threshold in [m/s] */
    float track_align_direction[3];    /**< Track alignment direction for x-, y- and z-axis in [m/s] */
    uint16_t reserved_1;               /**< Reserved for further use */
    uint32_t reserved_2[3];            /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParEKF_ALIGNMENT;
/**
 * This parameter configures the coarse alignment time
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t coarse_time;       /**< Coarse alignment time in [s] */
    XCOMFooter footer;
} XCOMParEKF_COARSETIME;
/**
 * This parameter configures the fine alignment time
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t align_time;        /**< Fine alignment time in [s] */
    XCOMFooter footer;
} XCOMParEKF_ALIGNTIME;
/**
 * This parameter configures the alignment mode
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t align_mode;        /**< Alignment mode */
    XCOMFooter footer;
} XCOMParEKF_ALIGNMODE;
/**
 * This parameter configures the gravity aiding inside the Kalman filter.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_GRAVITYAIDING
 */
#define EKF_GRAVITYAIDING_DISABLE 0 /**< Disable gravity aiding */
#define EKF_GRAVITYAIDING_ENABLE  1 /**< Enable gravity aiding */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable;            /**< Enable/Disable gravity aiding */
    float omg_thr;              /**< Angular rate sensor threshold in [rad/s] */
    float acc_thr;              /**< Accelerometer threshold in [m/s2] */
    float stddev;               /**< Standard deviation in [rad] */
    float gnss_timeout;         /**< GNSS timeout in [sec] */
    float aiding_interval;      /**< Aiding interval in [sec] */
    XCOMFooter footer;
} XCOMParEKF_GRAVITYAIDING;
/**
 * This parameter enables or disables the error compensation in the real-time navigation module. Setting a certain bit in this mask to ’0’
 * disables the feedback. Per default, all feedbacks to realtime navigation are activated, resulting in a feedback mode mask of ’0xFFFF’.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_FEEDBACK
 */
#define EKF_FEEDBACK_MASK_POSITION    0x00000001  /**< Enable/disable position error feedback to realtime navigation module */
#define EKF_FEEDBACK_MASK_VELOCITY    0x00000002  /**< Enable/disable velocity error feedback to realtime navigation module */
#define EKF_FEEDBACK_MASK_ATTITUDE    0x00000004  /**< Enable/disable attitude error feedback to realtime navigation module */
#define EKF_FEEDBACK_MASK_SENSORERORS 0x00000008  /**< Enable/disable sensor error feedback to realtime navigation module */
#define EKF_FEEDBACK_MASK_HEADING     0x00000010  /**< Enable/disable heading error feedback to realtime navigation module */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t feedback;          /**< Feedback mask (see EKF_FEEDBACK_MASK_* in appendix) */
    XCOMFooter footer;
} XCOMParEKF_FEEDBACK;
/**
 * This parameter contains the state freeze mask for realtime navigation. Setting an entry to ’1’ freezes the state. Per default, no states in
 * realtime navigation are frozen, the mask equals ’0’.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_STATEFREEZE
 */
#define EKF_STATEFREEZE_MASK_POSITION    0x00000001
#define EKF_STATEFREEZE_MASK_VELOCITY    0x00000002
#define EKF_STATEFREEZE_MASK_ATTITUDE    0x00000004
#define EKF_STATEFREEZE_MASK_ALTITUDE    0x00000008
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t state_freeze_mask; /**< Bitmask for freezing states in realtime navigation (see EKF_STATEFREEZE_MASK_* in appendix) */
    XCOMFooter footer;
} XCOMParEKF_STATEFREEZE;
/**
 * This parameter configures Zero Angular Rate Updates. Zero Angular Rate updates can be used for Land Navigation if a standstill has
 * been detected. Drift of the heading solution even during longer standstill periods can be reduced by several orders of magnitude by enabling this.
 * #domain: public
 * #scope: read and write
 * #name: XCOMParEKF_ZARU
 */
#define EKF_ZARU_DISABLE 0 /**< Disable Zero Angular Rate Update */
#define EKF_ZARU_ENABLE  1 /**< Enable Zero Angular Rate Update */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t enable;             /**< Enable/Ddisable Zero Angular Rate Update (see EKF_ZARU_* in appendix) */
    uint8_t reserved1[3];       /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParEKF_ZARU;
/**
 * This parameter configures the standard deviation of the Zero Angular Rate updated. Zero Angular Rate updates can be used for
 * Land Navigation if a standstill has been detected. Drift of the heading solution even during longer standstill periods can be reduced by
 * several orders of magnitude by enabling this.
 * #domain: public
 * #scope: read and write
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float stddev;               /**< standard deviation in [rad/s] */
    XCOMFooter footer;
} XCOMParEKF_ZARUSTDDEV;
/**
 * This parameter defines the initial standard deviations and the process noise model for the used inertial sensors.
 * #domain: public
 * #scope: read only
 * #name: XCOMParEKF_IMUCONFIG2
 */
 typedef struct {
    double root_noise_psd[3];        /**< Root Noise for x-, y- and z-axis in: accelerometers: [m/s2/sqrt(Hz)]; gyroscopes: [rad/s/sqrt(Hz)] */
    double offset_stddev[3];         /**< Bias standard deviation for x-, y- and z-axis in  accelerometers: [m/s2]; gyroscopes: [rad/s] */
    double offset_rw[3];             /**< Bias random walk for x-, y- and z-axis in accelerometers: [m/s3/sqrt(Hz)] gyroscopes: [rad/s2/sqrt(Hz)] */
    double scalefactor_stddev[3];    /**< Scale factor standard deviation for x-, y- and z-axis */
    double scalefactor_rw[3];        /**< Scale factor random walk for x-, y- and z-axis in: accelerometers: [1/s/sqrt(Hz)]; gyroscopes: [1/s/sqrt(Hz)] */
    double ma_stddev;                /**< Misalignment standard deviation in [rad] */
    double ma_rw;                    /**< Misalignment random walk in [rad] */
    double quantization[3];          /**< Quantization for x-, y- and z-axis */
} XCOMparEKF_IMUCONFIGtype2;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    union {
        XCOMParHeader param_header;     /**< XCOM parameter header */
        struct {
            uint16_t param_id;
            uint8_t estimation_config; /**< Estimated sensor misalignment configuration */
            uint8_t is_request;
        };
    };
    XCOMparEKF_IMUCONFIGtype2 acc;  /**< Deviation values related to accelerometers */
    XCOMparEKF_IMUCONFIGtype2 gyro; /**< Deviation values related to angular rate sensors */
    XCOMFooter footer;
} XCOMParEKF_IMUCONFIG2;
/**
 * This parameter defines the calibration time for the Zero Velocity Update mechanism. The ZUPT calibration mechanism can be triggered via
 * the ZUPTCALIBRATION command.
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t calib_time;        /**< ZUPT calibration time in [s] */
    uint16_t reserved1;         /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParEKF_ZUPTCALIB;
/**
 * This parameter contains the realignment condition bit mask. Setting an entry to ’1’ enables a condition in the automatic realignment
 * state machine. Per default, the realignment state machine is inactive and the the mask equals ’0’.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParEKF_RECOVERY
 */
#define EKF_RECOVERY_MASK_ACCEL 0x00000001 /**< Accelerometer overrange */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t recovery_mask;     /**< Bitmask for realignment conditions (see EKF_RECOVERY_MASK_* in appendix) */
    XCOMFooter footer;
} XCOMParEKF_RECOVERY;
/**
 * This parameter configures the frozen heading updates. Frozen heading updates are used during standstill to minimize the heading drift
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t enable;             /**< enable/disable FHUPD module */
    uint8_t reserved[3];        /**< reserved for further use */
    double delta_t;             /**< time between two consecutive frozen heading updates [s] */
    double stddev;              /**< standard deviation of the forced heading update [rad].
                                     If stdddev=0, the estimated heading standard deviation of the first standstill epoch will be used */
    XCOMFooter footer;
} XCOMParEKF_FHUPD;
/**
 * This parameter configures the frozen position updates.
 * Frozen position updates are used during standstill to minimize the position drift
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t enable;             /**< enable/disable FPUPD module */
    uint8_t reserved[3];        /**< reserved for further use */
    double delta_t;             /**< time between two consecutive frozen position updates [s] */
    double stddev[3];           /**< standard deviation of the frozen position update [m].
                                     If stdddev=0, the estimated position standard deviation of the first standstill epoch will be used */
    XCOMFooter footer;
} XCOMParEKF_FPUPD;
/**
 * This parameter configures the operation mode of the slew module.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParEKF_SLEWMODE
 */
#define EKF_SLEWMODE_DEFAULT  0 /**< Enable default mode */
#define EKF_SLEWMODE_TRIANGLE 1 /**< Enable triangle mode */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< Slew mode (see EKF_SLEWMODE_* in appendix) */
    XCOMFooter footer;
} XCOMParEKF_SLEWMODE;
/**
 * @deprecated
 * #scope: read and write
 * #domain: deprecated
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    double zupt_stddev;         /**< ZUPT standard deviation [m/s] */
    XCOMFooter footer;
} XCOMParEKF_ALIGNZUPTSTDDEV;
/**
 * @deprecated
 * #scope: read and write
 * #domain: deprecated
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    float velocity_threshold;   /**< @deprecated */
    XCOMFooter footer;
} XCOMParEKF_KINALIGNTHR;
/**
 * This parameter contains the standard deviation of the stored position and heading. These are used in stored position/heading alignment.
 * Per default, the standard deviations are not used.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParEKF_STOREDVALSTDDEV
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable_pos_stddev; /**< Enables/disables usage of position standard deviation */
    float pos_stddev;           /**< Position standard deviation in [m] */
    uint32_t enable_hdg_stddev; /**< Enables/disables usage of heading standard deviation */
    float hdg_stddev;           /**< Heading standard deviation in [rad] */
    XCOMFooter footer;
} XCOMParEKF_STOREDVALSTDDEV;
/**
 * This parameter is intended to prevent an initial backward moving being used for odometer aiding when the vehicle is
 * moving. Per default, this feature is deactivated.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParEKF_ODOREVDETECT
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable_detector;   /**< Enables/disables the reverse detector */
    float velocity_thr;         /**< Threshold in [m/s]. Velocities smaller than this threshold are treated as negative velocity. */
    XCOMFooter footer;
} XCOMParEKF_ODOREVDETECT;
/**
 * This parameter enables/disables the pseudo position. If enabled and the position has not yet been initialized, the stored
 * position is used for output via XCOM. Per default, this feature is deactivated.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParEKF_PSEUDOPOS
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable;            /**< Enables/disables the pseudo position. */
    XCOMFooter footer;
} XCOMParEKF_PSEUDOPOS;
/**
 * This parameter defines the position output frame of the INSSOL message.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParDAT_POS
 */
enum XCOM_PARDAT_POS_PosMode {
    XCOM_PARDAT_POS_Pos_WGS84 = 0, /**< The position fields of the INSSOL message shall contain the position in WGS84 frame */
    XCOM_PARDAT_POS_Pos_ECEF  = 1  /**< The position fields of the INSSOL message shall contain the position in ECEF frame */
};
enum XCOM_PARDAT_POS_AltMode {
    XCOM_PARDAT_POS_Alt_WGS84 = 0, /**< The altitude field of the INSSOL message shall contain the height in WGS84 frame */
    XCOM_PARDAT_POS_Alt_MSL   = 1, /**< The altitude field of the INSSOL message shall contain the altitude as mean sea level*/
    XCOM_PARDAT_POS_Alt_BARO  = 2  /**< The altitude field of the INSSOL message shall contain the height as barometric height */
};
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t position_mode;     /**< Position output mode (see XCOM_PARDAT_POS_Pos_* in appendix) */
    uint16_t altitude_mode;     /**< Altitude output mode (see XCOM_PARDAT_POS_Alt_* in appendix) */
    XCOMFooter footer;
} XCOMParDAT_POS;
/**
 * This parameter defines the velocity output frame of the INSSOL message.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParDAT_VEL
 */
enum XCOM_PARDAT_VEL_Mode {
    XCOM_PARDAT_VEL_NED = 0,  /**< The velocity fields of the INSSOL message contain the velocity in NED frame */
    XCOM_PARDAT_VEL_ENU = 1,  /**< The velocity fields of the INSSOL message contain the velocity in ENU frame */
    XCOM_PARDAT_VEL_ECEF = 2, /**< The velocity fields of the INSSOL message contain the velocity in ECEF frame */
    XCOM_PARDAT_VEL_BODY = 3  /**< The velocity fields of the INSSOL message contain the velocity in BODY frame */
};
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< Velocity output mode (see XCOM_PARDAT_VEL_* in appendix) */
    XCOMFooter footer;
} XCOMParDAT_VEL;
/**
 * This parameter defines the kind of inertial data of the INSSOL message.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParDAT_IMU
 */
enum XCOM_PARDAT_IMU_Mode {
    XCOM_PARDAT_IMU_IMURAW          = 0,    /**< Calibrated IMU data */
    XCOM_PARDAT_IMU_IMUCORR         = 1,    /**< Calibrated IMU data, additionally corrected for bias, scale factor and earth rate */
    XCOM_PARDAT_IMU_IMUCOMP         = 2,    /**< Calibrated IMU data, corrected for bias and scale factor; additionally compensated for earth rate and gravity */
    XCOM_PARDAT_IMU_IMUCORRFILTERED = 3,    /**< Calibrated IMU data, additionally corrected for bias, scale factor and earth rate */
    XCOM_PARDAT_IMU_IMUCOMPFILTERED = 4     /**< Calibrated IMU data, corrected for bias and scale factor; additionally compensated for earth rate and gravity */
};
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< Mode selection (see XCOM_PARDAT_IMU_* in appendix) */
    XCOMFooter footer;
} XCOMParDAT_IMU;
/**
 * This parameter defines the content of the SYS_STAT message.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParDAT_SYSSTAT
 */
#define PARDAT_SYSSTAT_MASK_IMU          0x00000001 /**< IMU status information */
#define PARDAT_SYSSTAT_MASK_GNSS         0x00000002 /**< GNSS status information */
#define PARDAT_SYSSTAT_MASK_MAG          0x00000004 /**< Magnetometer status information */
#define PARDAT_SYSSTAT_MASK_MADC         0x00000008 /**< Air data computer status information */
#define PARDAT_SYSSTAT_MASK_EKFAIDING    0x00000010 /**< EKF aiding status */
#define PARDAT_SYSSTAT_MASK_EKFGENERAL   0x00000020 /**< EKF general status */
#define PARDAT_SYSSTAT_MASK_ADDSTATUS    0x00000040 /**< Additional IMU status information */
#define PARDAT_SYSSTAT_MASK_SERVICE      0x00000080 /**< Service status information (@deprecated) */
#define PARDAT_SYSSTAT_MASK_REMALIGNTIME 0x00000100 /**< Remaining alignment time */
#define PARDAT_SYSSTAT_MASK_SYSSTAT2     0x00000200 /**< Extended system status */
#define PARDAT_SYSSTAT2_MAINTIMING       0x00000001 /**< System maintiming error  */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< Bitmask for system status message (see PARDAT_SYSSTAT_MASK_* in appendix). */
    XCOMFooter footer;
} XCOMParDAT_SYSSTAT;
/**
 * This parameter configures the output reference frame.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParDAT_REFFRAME
 */
#define PARDAT_REFFRAME_WGS84       0       /**< Reference frame is WGS84 */
#define PARDAT_REFFRAME_ETRF2014    1       /**< Reference frame is ETRF2014 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMParHeader param_header;    /**< XCOM parameter header */
    uint8_t reference_frame;       /**< Output reference frame (see PARDAT_REFFRAME_* in appendix) */
    uint8_t reserved[3];           /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParDAT_REFFRAME;
/**
 * This parameter configures the network interface
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t mode;               /**< 0: DHCP, 1: static */
    uint8_t reserved_1;         /**< Reserved for further use */
    uint8_t interface;          /**< The interface to be configured */
    uint8_t reserved_2;         /**< Reserved for further use */
    uint32_t port;              /**< TCP port */
    uint32_t ip_addr;           /**< IP address */
    uint32_t subnet_mask;       /**< Subnet mask */
    uint32_t gateway;           /**< Gateway address */
    XCOMFooter footer;
} XCOMParXCOM_NETCONFIG;
/**
 * This parameter configures the internal XCOM log list
 * @deprecated
 * #scope: read and write
 * #domain: deprecated
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    struct list {
        uint16_t divider;       /**< output divider */
        uint16_t msgID;         /**< message id */
    } LogList[XCOM_LOGS_PER_CHANNEL];
    XCOMFooter footer;
} XCOMParXCOM_LOGLIST;
/**
 * This parameter configures the internal XCOM log list.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParXCOM_LOGLIST2
 */
typedef struct XCOM_STRUCT_PACK {
  uint16_t divider;               /**< Output divider */
  uint16_t msg_id;                /**< Message ID */
  uint8_t running;                /**< Is set when channel is running */
  uint8_t trigger;                /**< see XComLogTrigger */
  uint16_t reserved1;             /**< Reserved for further use */
} XCOMParXCOM_LOGLIST2Type;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    union {
        XCOMParHeader param_header;     /**< XCOM parameter header */
        struct {
            uint16_t param_id;
            uint8_t channel;            /**< XCOM channel number */
            uint8_t is_request;
        };
    };
    XCOMParXCOM_LOGLIST2Type log_list[XCOM_LOGS_PER_CHANNEL];
    XCOMFooter footer;
} XCOMParXCOM_LOGLIST2;
/**
 * This parameter contains the interface configuration of the available serial ports. The number of serial ports may vary depending on the
 * hardware platform.
 * #scope: read and write
 * #domain: public
 * #name:XCOMParXCOM_INTERFACE
 */
#define PARXCOM_INTERFACE_MODE_NONE             0 /**< Serial port disabled */
#define PARXCOM_INTERFACE_MODE_XCOM             1 /**< Serial port configured for iXCOM protocol */
#define PARXCOM_INTERFACE_MODE_XCOMAUTOSTART    2 /**< Serial port configured for iXCOM protocol in autostart mode */
#define PARXCOM_INTERFACE_MODE_PASSTHROUGH      3 /**< Serial port configured for 'passthrough' mode (data will be  available in XCOM_MSGID_PASSTHROUGH) */
#define PARXCOM_INTERFACE_MODE_GNSSPASSTHROUGH  4 /**< Serial port routed to internal GNSS receiver */
#define PARXCOM_INTERFACE_MODE_GNSSCORRECTION   5 /**< Serial port routed to internal GNSS receiver with additional protocol options (RTCMv3, RTCMv2, etc.) */
#define PARXCOM_INTERFACE_MODE_EXTERNALSENSOR   6 /**< Serial port configured for communicating with external sensors */
#define PARXCOM_INTERFACE_LEVEL_RS232           0 /**< Configuration option of RS232 level */
#define PARXCOM_INTERFACE_LEVEL_RS422           1 /**< Configuration option of RS422 level */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t port;               /**< Port number */
    uint8_t port_mode;          /**< Port mode (see PARXCOM_INTERFACE_MODE_* in appendix) */
    uint8_t port_level;         /**< Interface level (see PARXCOM_INTERFACE_LEVEL_* in appendix) */
    uint8_t available;          /**< Port available (depending on hardware platform) */
    uint32_t baud;              /**< Baudrate */
    uint8_t reserved1[8];       /**< Additional options (depending on 'port_mode') */
    XCOMFooter footer;
} XCOMParXCOM_INTERFACE;
/**
 * @deprecated
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t channel_number;    /**< XCOM channel number */
    uint16_t auto_start;        /**< Enables/Disables auto start efeature */
    uint16_t port;              /**< Serial output port */
    uint16_t reserved;          /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParXCOM_AUTOSTART;
/**
 * This parameter configures the internal Ntrip client.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParXCOM_NTRIP
 */
#define PARXCOM_NTRIP_STRINGLENGTH  128
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                              /**< XCOM header */
    XCOMParHeader param_header;                     /**< XCOM parameter header */
    uint8_t mount_point[PARXCOM_NTRIP_STRINGLENGTH];   /**< Ntrip caster mount point */
    uint8_t user_name[PARXCOM_NTRIP_STRINGLENGTH];     /**< Ntrip caster username */
    uint8_t password[PARXCOM_NTRIP_STRINGLENGTH];      /**< Ntrip caster password */
    uint8_t server[128];                               /**< Ntrip caster (can be either an IP address or a server name) */
    uint8_t send_position_on_login;                 /**< Send current position when connecting to caster */
    uint8_t enable;                                 /**< Enable Ntrip client */
    uint16_t port;                                  /**< Ntrip caster network port */
    uint32_t gga_send_period;                       /**< GGA send period in [sec] */
    XCOMFooter footer;
} XCOMParXCOM_NTRIP;
/**
 * This parameter configures the internal Ntrip client.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParXCOM_NTRIPV2
 */
#define PARXCOM_NTRIPV2_MODE_STANDARD             0 /**< Ntrip version 2 (version 1 as fallback) */
#define PARXCOM_NTRIPV2_MODE_FORCED_V1            1 /**< Ntrip version 1 (forced) */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                 /**< XCOM header */
    XCOMParHeader param_header;                        /**< XCOM parameter header */
    uint8_t mount_point[PARXCOM_NTRIP_STRINGLENGTH];   /**< Ntrip caster mount point */
    uint8_t user_name[PARXCOM_NTRIP_STRINGLENGTH];     /**< Ntrip caster username */
    uint8_t password[PARXCOM_NTRIP_STRINGLENGTH];      /**< Ntrip caster password */
    uint8_t server[PARXCOM_NTRIP_STRINGLENGTH];        /**< Ntrip caster (can be either an IP address or a server name) */
    uint8_t send_position_on_login;                    /**< Send current position when connecting to caster */
    uint8_t enable;                                    /**< Enable Ntrip client */
    uint16_t port;                                     /**< Ntrip caster network port */
    uint32_t gga_send_period;                          /**< GGA send period in [sec] */
    uint8_t protocol_mode;                             /**< 0 = standard; 1 = force HTTP-V1 */
    uint8_t reserved;                                  /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParXCOM_NTRIPV2;
/**
 * This parameters enables the internal system recorder and configures the loglist automatically
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t enable;             /**< Start system recorder */
    uint8_t channel;            /**< Configures the given XCOM channel for post-processing logs */
    uint16_t resetved;          /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParXCOM_POSTPROC;
/**
 * This parameter configures UDP output
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t srv_addr;          /**< UDP destination address */
    uint32_t port;              /**< UDP destination port */
    uint8_t enable;             /**< Enables/Disables UDP output */
    uint8_t xcom_channel;       /**< XCOM channel selection */
    uint8_t enable_abd;         /**< Enables/Disables ABD protocol */
    uint8_t instance;           /**< Instance number */
    XCOMFooter footer;
} XCOMParXCOM_UDPCONFIG;
/**
 * This parameter is read-only and requests version information of the installed plugins.
 * #scope: read only
 * #domain: public
 * #name: XCOMParXCOM_PLUGINVERSION
 */
#define PARXCOM_PLUGINVERSION_MAX_VERSION_LEN     128                                            /**< Max. length of version string */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                                                           /**< XCOM header */
    XCOMParHeader param_header;                                                                  /**< XCOM parameter header */
    int8_t plugin_version[XCOMPAR_PLUGIN_MAX_NUM][PARXCOM_PLUGINVERSION_MAX_VERSION_LEN];       /**< Version info of plugins */
    int8_t plugin_api_version[XCOMPAR_PLUGIN_MAX_NUM][PARXCOM_PLUGINVERSION_MAX_VERSION_LEN];   /**< API version info of plugins */
    XCOMFooter footer;
} XCOMParXCOM_PLUGINVERSION;
/**
 * This parameter configures the *.dump file processing
 * #scope: read and write
 * #domain: hidden
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable;            /**< Enables/Disables processing */
    XCOMFooter footer;
} XCOMParXCOM_DUMPENABLE;
/**
 * This parameter configures the XCOM migrator module
 * #scope: read and write
 * #domain: hidden
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t enable;            /**< Enables/Disables XCOM migrator */
    XCOMFooter footer;
} XCOMParXCOM_MIGRATOR;
/**
 * This parameter configures the TCP keep alive timeout
 * @deprecated
 * #scope: read and write
 * #domain: hidden
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t timeout;
    uint16_t interval;
    uint16_t probes;
    uint16_t enable;
    XCOMFooter footer;
} XCOMParXCOM_TCPKEEPAL;
/**
 * This parameter configures the internal CAN gateway
 * @deprecated
 * #scope: read and write
 * #domain: hidden
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t src_port;          /**< Source port */
    uint32_t dst_port;          /**< Destination port */
    uint32_t dest_addr;         /**< Destination address */
    uint8_t arinc825_loopback;  /**< Enable/Disables loopback */
    uint8_t debug_enable;       /**< Enables/Disables debug output */
    uint8_t enable;             /**< Enables/Disables gateway */
    uint8_t interface;          /**< Interface selection */
    XCOMFooter footer;
} XCOMParXCOM_CANGATEWAY;
/**
 * This parameter configures the default (fallback) ip address
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t ip_addr;           /**< Fallback ip address */
    XCOMFooter footer;
} XCOMParXCOM_DEFAULTIP;
/**
 * This parameter is used to configure the ABD protocol version
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t protocol_version;   /**< ABD protocol version */
    uint8_t reserved[3];       /**< Reserved for further use */
    XCOMFooter footer;
} XCOMParXCOM_ABDVERSION;
/**
 * @deprecated
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    union {
        XCOMParHeader param_header;
        struct {
            uint16_t param_id;
            uint8_t entries;
            uint8_t is_request;
        };
    };
    struct {
        uint32_t ip_addr;
        uint32_t port;
        uint8_t enable;
        uint8_t channel;
        uint16_t connection_retries;
        struct {
            uint8_t msg_id;
            uint8_t trigger;
            uint16_t divider;
        } LOGS[XCOM_MAX_CLIENT_LOGS];
    } CONF[XCOM_MAX_CLIENT_SUPPORT];
    uint8_t use_udp_interface;
    uint8_t reserved[3];
    XCOMFooter footer;
} XCOMParXCOM_CLIENT;
/**
 * @deprecated
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t tcpdump_args[256];     /**< tcpdump arguments */
    XCOMFooter footer;
} XCOMParXCOM_TCPDUMP;
/**
 * This parameter configures the serial ports
 * #scope: read and write
 * #domain: public
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint8_t port;               /**< Serial port number */
    uint8_t options;            /**< Serial port options */
    uint16_t reserved;          /**< Reserved for further use */
    uint32_t baud_rate;         /**< Serial port baud rate */
    XCOMFooter footer;
} XCOMParXCOM_SERIALPORT;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t divider;
    uint8_t channel;
    uint8_t high_speed;
    XCOMFooter footer;
} XCOMParARINC429_CONFIG;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    union {
        XCOMParHeader param_header;
        struct {
            uint16_t param_id;
            uint8_t ppt_divider;
            uint8_t is_request;
        };
    };
    uint32_t mode;
    uint32_t mask;
    uint32_t matrix_row;
    XCOMFooter footer;
} XCOMParIO_SYNCOUT;
/**
 * This parameter configures the synchronization output module
 * #scope: read and write
 * #domain: public
 * #name: XCOMParIO_SYNCOUT2
 */
#define XCOM_SYNCOUT2_MODULE_OFF                        0   /**< Disable output synchronization module */
#define XCOM_SYNCOUT2_MODULE_ON                         1   /**< Enable output synchronization module */
#define XCOM_SYNCOUT2_PINMUX_DISCOUT                    0   /**< Use DICRETE_OUT output (only available for IEP288v5 platforms) */
#define XCOM_SYNCOUT2_PINMUX_TIMERMATRIX                1   /**< Use timer matrix output */
#define XCOM_SYNCOUT2_COMPVALUE_GLOBAL_STATUS           0   /**< Use global status as compare input value */
#define XCOM_SYNCOUT2_COMPVALUE_SYSTEM_STATUS           1   /**< Use system status as compare input value */
#define XCOM_SYNCOUT2_COMPVALUE_FPGA_STATUS             2   /**< Use fpga status as compare input value */
#define XCOM_SYNCOUT2_COMPVALUE_EKFSTATUS_LO_STATUS     3   /**< Use ekf aiding status (lo) status as compare input value */
#define XCOM_SYNCOUT2_COMPVALUE_EKF_STATUS_HI_STATUS    4   /**< Use ekf aiding status (hi) as compare input value */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t module_mode;                    /**< Enable/Disable output synchronization module */
    uint8_t sync_out_pinmux;                /**< Pinmuxing selection (DICRETE vs. Timer-Matrix) */
    uint8_t discrete_output_pin_number;     /**< DISCRETE_OUT pin number (0 or 1) */
    uint8_t timer_matrix_row_idx;           /**< Timer matrix row index (0-31) */
    uint8_t compare_value_selection;        /**< Compare input selection */
    uint8_t reserved[3];                    /**< Reserved for further use */
    uint32_t compare_mask;                  /**< Compare mask */
    XCOMFooter footer;                      /**< XCOM footer */
} XCOMParIO_SYNCOUT2;
/**
 * This parameter configures the synchronization input module
 * #scope: read and write
 * #domain: public
 * #name: XCOMParIO_SYNCIN
 */
#define XCOM_SYNCIN_MODULE_OFF                          0   /**< Disable input synchronization module */
#define XCOM_SYNCIN_MODULE_ON                           1   /**< Enable input synchronization module */
#define XCOM_SYNCIN_FUNCTION_PRIMARY_GNSS_LA_SWITCH     0   /**< Enable primary GNSS lever arm sitching via SYNC_IN */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMParHeader param_header;             /**< XCOM parameter header */
    uint8_t module_mode;                    /**< Enable/Disable input synchronization module */
    uint8_t timer_matrix_col_idx;           /**< Timer matrix column index (0-31) */
    uint8_t sync_input_function;            /**< Synchronization input selection */
    uint8_t reserved;                       /**< Reserved for further use */
    XCOMFooter footer;                      /**< XCOM footer */
} XCOMParIO_SYNCIN;
/*
 * Aliases
 */
typedef XCOMParamGeneric XCOMParINVALID;
typedef XCOMParamGeneric XCOMParSCU_NADIR;
typedef XCOMParamGeneric XCOMParSCU_STAB;
typedef XCOMParamGeneric XCOMParSCU_PARAM;
typedef XCOMParamGeneric XCOMParSCU_ERRBUF;
#include "plugin.h"
#if !defined XCOM_DISABLE_STRUCT_PACKING
#if defined __clang__ || defined __GNUC__ || defined __MINGW32__
#undef XCOM_STRUCT_PACK
#elif defined _MSC_VER
#pragma pack(pop)
#undef XCOM_STRUCT_PACK
#else
#error This compiler is currently not supported.
#endif
#endif
// NOLINTEND
// clang-format on
#endif /* XCOMDAT_H_ */
