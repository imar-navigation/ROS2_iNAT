/*.*******************************************************************
  FILENAME: XCOMdat.h
**********************************************************************
* 	PROJECT:        ROS2_iNAT
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
#if defined __clang__ || defined __GNUC__ || defined __MINGW32__
#define XCOM_STRUCT_PACK __attribute__((__packed__))
#elif defined _MSC_VER
#define XCOM_STRUCT_PACK
#pragma pack(push)
#pragma pack(1)
#else
#error This compiler is currently not supported.
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
#define XCOM_MAX_SATCHANNELS               128  /**< Max. number of supported satellite channels */
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
    XCOM_MSGID_IMUCORR              = 0x01,
    XCOM_MSGID_INSSOL               = 0x03,
    XCOM_MSGID_INSSOLECEF           = 0x47,
    XCOM_MSGID_INSDCM               = 0x05,
    XCOM_MSGID_EKFSTDDEV            = 0x0F,
    XCOM_MSGID_EKFSTDDEVECEF        = 0x48,
    XCOM_MSGID_GNSSSOL              = 0x12,
    XCOM_MSGID_GNSSTIME             = 0x14,
    XCOM_MSGID_GNSSSOLCUST          = 0x15,
    XCOM_MSGID_GNSSLEVERARM         = 0x1B,
    XCOM_MSGID_MAGDATA              = 0x18,
    XCOM_MSGID_SYSSTAT              = 0x19,
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
    XCOM_CMDXCOM_MAX                = XCOM_CMDXCOM_LOGLISTPRESET
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
                                                     perform an initial alignment and starts navigation. All internal components will be powered up*/
    XCOM_CMDFPGA_FLUSHFRAM          = 0x0006,   /**< If the command is received, the system will flush data to the FRAM if possible */
    XCOM_CMDFPGA_RESETSYNCCOUNTER   = 0x0007,   /**< Reset external synchronization counter value */
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
    XCOM_CMDEXTAID_MAX          = XCOM_CMDEXTAID_POS_MGRS
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
    XCOMPAR_PARSYS_FWVERSION        = 5,
    XCOMPAR_PARSYS_MAINTIMING       = 11,
    XCOMPAR_PARSYS_PRESCALER        = 12,
    XCOMPAR_PARGNSS_LOCKOUTSYSTEM   = 212,
    XCOMPAR_PAREKF_STARTUPV2        = 731,
    XCOMPAR_PAREKF_MAGATTAID        = 732,
    XCOMPAR_PAREKF_IMUCONFIG2       = 760,
    XCOMPAR_PARDAT_POS      = 800,
    XCOMPAR_PARDAT_VEL      = 801,
    XCOMPAR_PARDAT_SYSSTAT  = 803,
    XCOMPAR_PARXCOM_SERIALPORT      = 902,
    XCOMPAR_PARXCOM_POSTPROC        = 908,
    XCOMPAR_PARXCOM_INTERFACE       = 921,
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
};
typedef struct XCOM_STRUCT_PACK {
    uint8_t sync;               /**< Synchronization character (is always set to 0x7E;XCOM_SYNC_BYTE) */
    uint8_t msg_id;             /**< iXCOM message ID. iXCOM distinguishes between four different types of message IDs (ordinary message IDs;
                                     command ID; parameter ID, response ID) */
    uint8_t frame_counter;      /**< The frame counter counts from 0 to 255 and starts at 0 again after it has reached 255. For messages,
                                     this can be used to detect lost messages. For parameters and commands, the system response to an
                                     input parameter or command will contain the same frame counter as the input. */
    uint8_t trigger_source;     /**< See <iXCOM LOG command trigger type> */
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
    uint8_t idx;              /**< Plugin index */
    uint8_t reserved;         /**< Reserved for further use */
    uint16_t plugin_param_id; /**< Plugin specific id */
} XCOMPluginHeader;
typedef struct XCOM_STRUCT_PACK {
    uint16_t plugin_data_id; /**< Plugin specific message id */
    uint16_t reserved;       /**< Reserved for further use */
} XCOMPluginDataHeader;
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
    double pos[2];           /**< Longitude and latitude in [rad]. The selected values are masked in the DATSEL field. */
    float altitude;          /**< Altitude/Height in [m]. The content (WGS or MSL or Baro based) can be selected via the PARDAT_POS
                                  parameter. The selected value is masked in the DATSEL field. */
    int16_t undulation;      /**< Relationship between the geoid and the ellipsoid in [cm]. */
    uint16_t data_selection; /**< Data selection mask. This field contains the selected data sources for this message  */
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
    float q_nb[4];      /**< Computation frame to NED quaternion */
    XCOMFooter footer;
} XCOMmsg_INSSOLECEF;
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
 * The MAGDATA message contains the magnetometer measurements. The Extended Kalman Filter is able to process magnetic heading as well as the
 * magnetic field vector.
 * #domain: public
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;  /**< XCOM header */
    float field[3];     /**< Measured magnetic field along sensors x-, y- and z-axis */
    float heading;      /**< Obtained magnetic heading in [rad] */
    float bank;         /**< Magnetometer bank in [rad] */
    float elevation;    /**< Magnetometer elevation in [rad] */
    float deviation;    /**< Magnetic deviation in [rad] */
    uint32_t status;    /**< Magnetometer status */
    XCOMFooter footer;
} XCOMmsg_MAGDATA;
/**
 * The EKFSTDDEV message contains the standard deviation of the estimated position, velocity, attitude, heading and sensor errors.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float pos[3];      /**< Standard deviation of estimated easting, northing and height in [m] */
    float vel[3];      /**< Standard deviation of estimated north, east and down velocity in [m/s] */
    float tilt[3];     /**< Standard deviation of error tilt angles around north, east, and down axis. Down axis tilt stddev is equivalent
                            to yaw angle stddev. [rad] */
    float bias_acc[3]; /**< Standard deviation of estimated acceleration bias along IMU x-, y- and z-axis in [m/s2 ] */
    float bias_omg[3]; /**< Standard deviation of estimated angular rate bias along IMU x-, y- and z-axis in [rad/s] */
    float scf_acc[3];  /**< Standard deviation of estimated acceleration scale  factor along IMU x-, y- and z-axis */
    float scf_omg[3];  /**< Standard deviation of estimated angular rate scale  factor along IMU x-, y- and z-axis */
    float scf_odo;     /**< Standard deviation of the estimated odometer scale factor */
    XCOMFooter footer;
} XCOMmsg_EKFSTDDEV;
/**
 * This message contains the estimated sensor errors by the extended Kalman filter in the ECEF frame. In addition to the standard deviations
 * reported in the EKFSTDDEV, estimated sensor nonorthogonality and misalignment standard deviations are included in this log.
 * #domain: public
 * #rate: ekf
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header; /**< XCOM header */
    float pos[3];      /**< Standard deviation of estimated ECEF x, y and z in [m] */
    float vel[3];      /**< Standard deviation of estimated ECEF x, y and z velocity in  [m/s] */
    float tilt[3];     /**< Standard deviation of estimated tilt around north, east and down axis in [rad] */
    float bias_acc[3]; /**< Standard deviation of estimated acceleration bias along IMU x-, y- and z-axis in [m/s2 ] */
    float bias_omg[3]; /**< Standard deviation of estimated angular rate bias along IMU x-, y- and z-axis in [rad/s] */
    float ma_acc[9];   /**< Standard deviation of estimated acceleration misalignment along IMU x-, y- and z-axis */
    float ma_omg[9];   /**< Standard deviation of estimated angular rate misalignment along IMU x-, y- and z-axis */
    float scf_odo;     /**< Standard deviation of the estimated odometer scale factor */
    float ma_odo[2];   /**< Standard deviation of estimated odometer misalignment around first and second axis perpendicular to odometer
                          direction */
    XCOMFooter footer;
} XCOMmsg_EKFSTDDEVECEF;
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
    uint32_t InvalidFirmware : 1;          /**< Invalid firmware */
    uint32_t RomStatus : 1;                /**< ROM status */
    uint32_t SupplyVoltage : 1;            /**< Supply voltage status */
    uint32_t PllRf1Error : 1;              /**< TrackingPLL RF1 Hardware Status - L1 */
    uint32_t PllRf2Error : 1;              /**< TrackingPLL RF2 Hardware Status - L2 */
    uint32_t SwRessourceLimit : 1;         /**< Software Resource Status */
    uint32_t InvalidModel : 1;             /**< Model not valid */
    uint32_t HwFailure : 1;                /**< Component hardware failure */
    uint32_t TempError : 1;                /**< Temperature status */
    uint32_t AntennaPowerStatus : 1;       /**< Antenna power status */
    uint32_t PrimaryAntennaOpen : 1;       /**< Primary antenna open */
    uint32_t PrimaryAntennaShorted : 1;    /**< Primary antenna shorted */
    uint32_t CpuOverload : 1;              /**< CPU overload */
    uint32_t Com1BufferOverrun : 1;        /**< COM1 buffer overrun */
    uint32_t Com2BufferOverrun : 1;        /**< COM2 buffer overrun */
    uint32_t Com3BufferOverrun : 1;        /**< COM3 buffer overrun */
    uint32_t AlmanacFlag : 1;              /**< Almanac flag/UTC known */
    uint32_t SolutionInvalid : 1;          /**< Position solution flag */
    uint32_t VoltageSupply : 1;            /**< Power supply warning */
    uint32_t UpdateInProgress : 1;         /**< Firmware update in progress */
    uint32_t SolutionCodeType : 1;         /**< 0: Some data in PVT solution from C/A-code. 1: All data in PVT solution from P-code */
    uint32_t SecondaryAntennaOpen : 1;     /**< Secondary antenna open */
    uint32_t Reserved : 10;                /**< Reserved for further use */
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
    uint16_t sol_status;  /**< GNSS Solution status */
    uint16_t posvel_type; /**< GNSS position/velocity type */
    float pdop;           /**< Position Dilution Of Precision - uncertainty indicator for 3D parameters (latitude, longitude, height) */
    uint8_t sats_used;    /**< Number of satellites used in GNSS solution */
    uint8_t sats_tracked; /**< Number of satellites tracked */
    uint16_t station_id;  /**< Reference station ID of differential corrections */
    float diff_age;       /**< Differential age in [sec] - differential correction indicator */
    float sol_age;        /**< Solution age in [sec] */
    GnssStatusType gnss_status; /**< GNSS receiver status */
    XCOMFooter footer;
} XCOMmsg_GNSSSOL;
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
    uint32_t utc_status; /**< UTC Status */
    XCOMFooter footer;
} XCOMmsg_GNSSTIME;
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
    Rawdata,        /**< IMU/GNSS/Odometer rawdata (usefully for post-processing without having an INS online solution) */
    NavSolution,    /**< INS online solution without rawdata acquisition  */
    Support,        /**< Combination of Rawdata and NavSolution */
    Qualification   /**< Similar to Rawdata but with additional hardware monitoring messages */
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
    uint32_t conf_cmd;          /**< see XCOMcmd_Conf */
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
    uint16_t fpga_cmd;          /**< see XCOMcmd_Ekf */
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
 * Set of commands to save current position/heading and to start and alignment.
 * #name: XCOMCmd_EKF
 */
typedef struct XCOM_STRUCT_PACK {
    uint16_t ekf_cmd_id;
    uint16_t number_of_arguments;
} XCOMCmdEkfHeader;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMCmdHeader cmd_header;   /**< XCOM command header */
    uint16_t ekf_cmd;           /**< see XCOMcmd_EKF */
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
    XCOMCmdHeader cmd_header;        /**< XCOM command header */
    double time_stamp;              /**< Time at which the measurement was valid [sec]  */
    uint16_t time_mode;             /**< Time mode <time_stamp>:
                                           0 = Absolute GPS timestamp
                                           1 = Latency */
    uint16_t command_parameter_id;  /**< ID = XCOM_CMDEXTAID_VEL_BODY */
    double velocity[3];             /**< External velocity in body x-,y-,z-direction [m/s] */
    double velocity_stddev[3];      /**< Standard deviation of external velocity aiding [m/s] */
    double lever_arm[3];            /**< Lever arm in x,y,z-direction [m] */
    double lever_arm_stddev[3];     /**< tandard deviation of lever arm in x,y,z-direction [m] */
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
#define PARODO_MODE2_MAX_ODOMETERS  3
/**
 * This parameter can be used to define the initialization behavior of the  Kalman filter. A state chart detailing the navigation
 * startup process is available in appendix of DOC141126064.
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
    uint8_t position_mode;      /**< Position initialization mode */
    uint8_t hdg_mode;           /**< Heading initialization mode */
    uint16_t gnss_timeout;      /**< GNSS timeout in [sec] Set to ’0’ to wait forever */
    uint8_t enable_alt_msl;     /**< Alternate Mean Sea */
    uint8_t realignment;        /**< 0: Store parameters without execution of an alignment;
                                     1: Start alignment immediately with the transmitted alignment parameters */
    uint8_t forced_inmotion;    /**< During alignment, system is
                                     0: Static;
                                     1: In-motion */
    uint8_t automatic_restart;  /**< The levelling will be restarted if movement of the system is detected during the levelling
                                     phase. The fine alignment will be finished if movement of the system is detected */
    XCOMFooter footer;
} XCOMParEKF_STARTUPV2;
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
    uint8_t aiding_mode;        /**< Configures the aiding behavior. */
    uint8_t update_mode;        /**< EKF update mode */
    uint16_t aiding_interval;   /**< Magnetometer aiding interval in [s] */
    float mag_field_stddev[3];  /**< Magnetic field standard deviation in [mG] */
    XCOMFooter footer;
} XCOMParEKF_MAGATTAID;
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
    double ma_stddev;                /**< Misalignment standard deviation in: accelerometers: [m]; gyroscopes: [rad] */
    double ma_rw;                    /**< Misalignment random walk in: accelerometers: [m/s2/sqrt(Hz)]; gyroscopes: [rad/s/Hz] */
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
 * This parameter defines the position output frame of the INSSOL message.
 * #scope: read and write
 * #domain: public
 * #name: XCOMParDAT_POS
 */
enum XCOM_PARDAT_POS_PosMode {
    XCOM_PARDAT_POS_Pos_WGS84 = 0, /**< The position fields of the INSSOL message shall contain the position in WGS84 frame */
    XCOM_PARDAT_POS_Pos_ECEF  = 1  /**< The position fields of the INSSOL message shall contain the position in ECEF´ frame */
};
enum XCOM_PARDAT_POS_AltMode {
    XCOM_PARDAT_POS_Alt_WGS84 = 0, /**< The altitude field of the INSSOL message shall contain the height in WGS84 frame */
    XCOM_PARDAT_POS_Alt_MSL   = 1, /**< The altitude field of the INSSOL message shall contain the altitude as mean sea level*/
    XCOM_PARDAT_POS_Alt_BARO  = 2  /**< The altitude field of the INSSOL message shall contain the height as barometric height */
};
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint16_t position_mode;     /**< Position output mode */
    uint16_t altitude_mode;     /**< Altitude output mode */
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
    uint32_t mode;              /**< Velocity output mode */
    XCOMFooter footer;
} XCOMParDAT_VEL;
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
#define PARDAT_SYSSTAT_MASK_SERVICE      0x00000080 /**< Service status information (deprecated) */
#define PARDAT_SYSSTAT_MASK_REMALIGNTIME 0x00000100 /**< Remaining alignment time */
#define PARDAT_SYSSTAT_MASK_SYSSTAT2     0x00000200 /**< Extended system status */
#define PARDAT_SYSSTAT2_MAINTIMING       0x00000001 /**< System maintiming error  */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;          /**< XCOM header */
    XCOMParHeader param_header; /**< XCOM parameter header */
    uint32_t mode;              /**< Bitmask for system status message */
    XCOMFooter footer;
} XCOMParDAT_SYSSTAT;
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
    uint8_t port_mode;          /**< Port mode (see PARXCOM_INTERFACE_MODE_*) */
    uint8_t port_level;         /**< Interface level (see PARXCOM_INTERFACE_LEVEL_*) */
    uint8_t available;          /**< Port available (depending on hardware platform) */
    uint32_t baud;              /**< Baudrate */
    uint8_t reserved1[8];       /**< Additional options (depending on 'port_mode') */
    XCOMFooter footer;
} XCOMParXCOM_INTERFACE;
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
#define XCOMMSG_PLUGINID_DBXDBOUT XCOM_PLUGINID_DBXDBOUT
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_BITE_LEVEL0      0x00
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_BITE_LEVEL1      0x01
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_BITE_LEVEL2      0x02
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_BITE_LEVEL3      0x04
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_BITE_LEVEL4      0x06
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_MODE_NAV         0x00
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_MODE_ALIGN       0x01
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_MODE_MAINT       0x03  // maintenance mode
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_GNSSTIME_VALID   0x00
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_GNSSTIME_DRIFT   0x01
#define XCOMMSG_PLUGINID_DBXDBOUT_STATUS_GNSSTIME_INVALID 0x03
#define XCOMMSG_PLUGINID_DBXDBOUT_SATS_USED_MAX  15
#define XCOMMSG_PLUGINID_DBXDBOUT_POSERROR_VALUE 0xffff
typedef struct {
    uint8_t NAV_BITE           : 3;  // error level
    uint8_t NAV_MODE           : 2;  // alignment/navigation/maintenance mode
    uint8_t NAV_ATTVEL_INVALID : 1;  // attitude/velocity invalid
    uint8_t NAV_POS_INVALID    : 1;
    uint8_t NAV_INMOTION       : 1;  // system detected motion
} XCOMDbxDbOutputStatusInatBits;
typedef union {
    XCOMDbxDbOutputStatusInatBits bits;
    uint8_t value;
} XCOMDbxDbOutputStatusInatType;
typedef struct {
    uint8_t GNSS_SATS_USED     : 4;  // satellites used
    uint8_t GNSS_POS_USED      : 1;
    uint8_t GNSS_DIFF_SOL      : 1;  // 0: single point solution, 1: differential/RTK solution
    uint8_t GNSS_TIMEREF       : 2;  // see XCOMMSG_PLUGINID_DBXDBOUT_STATUS_GNSSTIME
} XCOMDbxDbOutputStatusGps1Bits;
typedef union {
    XCOMDbxDbOutputStatusGps1Bits bits;
    uint8_t value;
} XCOMDbxDbOutputStatusGps1Type;
typedef struct {
    uint8_t GNSS_NODATA        : 1;  // no data from GNSS receiver
    uint8_t GNSS_SW_RESET      : 1;  // unused
    uint8_t GNSS_MEM_ERROR     : 1;  // unused
    uint8_t GNSS_BAT_EMPTY     : 1;  // unused
    uint8_t GNSS_NODATA_DIFF   : 1;  // No DPGS data on GNSS receiver8
} XCOMDbxDbOutputStatusGps2Bits;
typedef union {
    XCOMDbxDbOutputStatusGps2Bits bits;
    uint8_t value;
} XCOMDbxDbOutputStatusGps2Type;
typedef struct {
    uint8_t HPOS_VALID    : 1;  /**< validity of horizontal position and drms values */
    uint8_t TIMEREF_VALID : 1;  /**< validity of provided timestamp */
    uint8_t DATA_SYNC     : 1;  /**< sync data with INS time */
    uint8_t HEIGHT_VALID  : 1;  /**< validity of height value */
    uint8_t RESTART       : 1;  /**< indicates if navigation should get stopped and re-alignment initiated */
    uint8_t IN_MOTION     : 1;  /**< indicates if vehicle is in motion */
    uint8_t USE_GNSS      : 1;  /**< indicates if GNSS position/velocity should be used for aiding */
    uint8_t ATT_VALID     : 1;  /**< validity of rail-pitch, alpha-c, beta-c and gamma-c values */
} XCOMDbxDbInputStatusBits;
typedef union {
    XCOMDbxDbInputStatusBits bits;
    uint8_t value;
} XCOMDbxDbInputStatusType;
#define XCOMPAR_PLUGINID_DBXDBIN XCOM_PLUGINID_DBXDBIN
#define XCOMPAR_PLUGINID_DBXDBCONF XCOM_PLUGINID_DBXDBCONF
#if defined __clang__ || defined __GNUC__ || defined __MINGW32__
#undef XCOM_STRUCT_PACK
#elif defined _MSC_VER
#pragma pack(pop)
#undef XCOM_STRUCT_PACK
#else
#error This compiler is currently not supported.
#endif
// NOLINTEND
// clang-format on
#endif /* XCOMDAT_H_ */
