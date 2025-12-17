/*.*******************************************************************
 FILENAME: plugin.h
 **********************************************************************
 * PROJECT:	    iXCOM_SDK
 *
 *
 *---------------------------------------------------------------------
 * Copyright 2025, iMAR Navigation GmbH
 *---------------------------------------------------------------------
 * MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/
// clang-format off
// NOLINTBEGIN
#ifdef XCOMDAT_H_
#ifndef PLUGIN_H
#define PLUGIN_H
#ifdef __cplusplus
#include <cstdint>
/**
 * XCOM plugin IDs
 * These IDs will be used for plugin specific messages. The plugin ID is included in the plugin header
 */
enum PluginIDs {
    XCOM_PLUGINID_ARINC429      = 0,  /**< ARINC429 plugin id */
    XCOM_PLUGINID_HEAVE         = 2,  /**< Heave plugin ID */
    XCOM_PLUGINID_ATTHEMO       = 3,  /**< Index table plugin id */
    XCOM_PLUGINID_LOOPBACK      = 4,  /**< Loopback self-test plugin id */
    XCOM_PLUGINID_TECDATA       = 5,  /**< TEC controller data package */
    XCOM_PLUGINID_TECCMD        = 6,  /**< TEC controller command package */
    XCOM_PLUGINID_TEMPSTAB      = 7,  /**< TEMPSTAB parameter id */
    XCOM_PLUGINID_IATIN         = 8,  /**< This message contains the iAT-input received from RiCS. */
    XCOM_PLUGINID_IATOUT        = 9,  /**< This message contains all the RiCS-specific iAT-output data to be sent to RiCS. */
    XCOM_PLUGINID_IATANGLIM     = 10, /**< This parameter contains the angular limits of the actuators */
    XCOM_PLUGINID_IATRATEACCLIM = 11, /**< This parameter contains rate and acceleration limits of the actuators */
    XCOM_PLUGINID_IATLEVARM     = 12, /**< This parameter contains the leverarms of the actuators */
    XCOM_PLUGINID_IATALGO       = 13, /**< This parameter contains internal paramter concerning signal processing and control */
    XCOM_PLUGINID_COBRA         = 14, /**< COBRA plugin id */
    XCOM_PLUGINID_PEGASUS       = 15, /**< PEGASUS plugin id */
    XCOM_PLUGINID_PIXHAWK       = 16, /**< Pixhawk plugin id */
    XCOM_PLUGINID_MSU           = 17, /**< Mobile Surveying Unit (MSU) plugin id */
    XCOM_PLUGINID_DBXDBOUT  = 20, /**< Rail DBX-DB plugin message containing output data */
    XCOM_PLUGINID_DBXDBIN   = 21, /**< Rail DBX-DB plugin parameter to provide input data */
    XCOM_PLUGINID_DBXDBCONF = 22, /**< Rail DBX-DB plugin parameter to modify plugin configuration */
    XCOM_PLUGINID_ATTHEMODATA = 31, /**< iATTHEMO plugin id */
    XCOM_PLUGINID_GREENLIGHT_LOCAL         = 0x70, /**< Oebb-Verschub: GREENLIGHTLOCAL message as plugin data */
    XCOM_PLUGINID_GREENLIGHT_SMARTPHONE    = 0x71, /**< Oebb-Verschub: GREENLIGHTSMARTPHONE message as plugin data */
    XCOM_PLUGINID_GREENLIGHT_ALARM         = 114,  /**< Oebb-Verschub: Parameter to trigger alarm or get last alarm triggered */
    XCOM_PLUGINID_GREENLIGHT_MODEM_INFO    = 115,  /**< Oebb-Verschub: General modem info (read-only parameter) */
    XCOM_PLUGINID_GREENLIGHT_MODEM_SERVICE = 116,  /**< Oebb-Verschub: Modem connection and internet service info */
    XCOM_PLUGINID_GREENLIGHT_MODEM_SIGQUAL = 117,  /**< Oebb-Verschub: Modem signal quality */
    XCOM_PLUGINID_GREENLIGHT_MODEM_CELL    = 118,  /**< Oebb-Verschub: Modem cell info */
    XCOM_PLUGINID_GREENLIGHT_STATUS        = 119   /**< Oebb-Verschub: General status info (gnss, gsmr, login) */
};
/**
 * XCOMGreenlightSourceIdentifier defines the data source of the transmitted data
 */
enum XCOMGreenlightSourceIdentifier {
    Locomotive = 0, /**< Data source is the integrated iNAT device of the locomotive */
    Mover      = 1, /**< Data source is the iVLOC device of the 'mover' */
    Local      = 2, /**< Greenlight Local data source */
    Central    = 3  /**< Greenlight Central data source */
};
/**
 * XCOMGreenlightHeader defines the data which are included in Greenlight specific data transfer
 */
typedef struct XCOM_STRUCT_PACK {
    uint8_t source_identifier;      /**< Data source identifier (see GreenlightSourceIdentifier) */
    uint8_t destination_identifier; /**< Data destination identifier (see GreenlightSourceIdentifier) */
    uint8_t acknowledge_number;     /**< A Random Number Token for Acknowledgement */
    uint8_t uic_number[5];          /**< UIC Number for locomotive identification */
} XCOMGreenlightHeader;
/**
 * XCOMGeoPositionType contains the position in WGS84 or ETRS89 geodetic coordinates or whatever is the base regarding RTK correction data
 */
typedef struct XCOM_STRUCT_PACK {
    double latitude;  /**< Longitude in [rad] */
    double longitude; /**< Latitude in [rad] */
    float height;     /**< Height in [m] */
} XCOMGreenlightGeoPositionType;
/**
 * XCOMGeoPositionStdDevType contains the standard deviation of the estimated position errors.
 */
typedef struct XCOM_STRUCT_PACK {
    /**< Standard deviation of estimated easting, northing and height in [m] */
    float latitude;
    float longitude;
    float height;
} XCOMGreenlightGeoPositionStdDevType;
/**
 * XCOMVelocityType contains the velocity vector in NED coordinates
 */
typedef struct XCOM_STRUCT_PACK {
    /**< Estimated north, east, down velocity in [m/s] */
    float north;
    float east;
    float down;
} XCOMGreenlightVelocityType;
/**
 * XCOMVelocityStdDevType contains the standard deviation of the estimated velocity errors.
 */
typedef struct XCOM_STRUCT_PACK {
    /**< Standard deviation of estimated north, east and down velocity in [m/s] */
    float north;
    float east;
    float down;
} XCOMGreenlightVelocityStdDevType;
/**
 * XCOMAttitudeType contains the integration filter attitude solution in Euler representation (roll, pitch and yaw).
 * The given Euler angles describe the orientation of the body frame with respect to the navigation frame (NED).
 */
typedef struct XCOM_STRUCT_PACK {
    /**< Roll, pitch and yaw in [rad] */
    float roll;
    float pitch;
    float yaw;
} XCOMGreenlightAttitudeType;
/**
 * GreenlightLocal specific status word (TBD)
 * #domain: hidden
 * #rate: event
 * #name: XCOMmsg_GREENLIGHTLOCAL
 */
typedef struct {
    uint16_t valid    : 1;  /**< Transmitted data are valid(1)/invalid(0) */
    uint16_t reserved : 15; /**< Reserved for further use */
} XCOMGreenlightStatusBitsType;
typedef union {
    XCOMGreenlightStatusBitsType bits;
    uint16_t value;
} XCOMGreenlightStatusType;
typedef struct XCOM_STRUCT_PACK {
    uint8_t satellites_used;                            /**< Number of satellites used in GNSS solution */
    uint8_t reserved;                                   /**< Reserved for further use */
    XCOMGreenlightStatusType status;                    /**< Greenlight specific status word (see XCOMGreenlightStatusBitsType) */
    XCOMGreenlightGeoPositionType llh_pos;              /**< Position solution in WGS84 or ETRS89 geodetic coordinates or whatever is
                                                             the base regarding RTK correction data */
    XCOMGreenlightGeoPositionStdDevType llh_pos_stddev; /**< Estimated position error */
    XCOMGreenlightVelocityType vel;                     /**< Velocity vector in NED coordinates */
    XCOMGreenlightVelocityStdDevType vel_stddev;        /**< Estimated velocity error */
    XCOMGreenlightAttitudeType attitude;                /**< Estimated attitude solution in Euler representation */
} XCOMGreenlightInsType;
typedef struct XCOM_STRUCT_PACK {
    uint8_t satellites_used;               /**< Number of satellites used in GNSS solution */
    uint8_t reserved;                      /**< Reserved for further use */
    XCOMGreenlightStatusType status;       /**< Greenlight specific status word (see XCOMGreenlightStatusBitsType) */
    XCOMGreenlightGeoPositionType llh_pos; /**< Position solution in WGS84 or ETRS89 geodetic coordinates or whatever is
                                                             the base regarding RTK correction data */
    float speed;                           /**< Total speed in [m/s] */
    uint64_t time;                         /**< Unix epoch time of the location fix,
                                                             in [ms] from Unix epoch (00:00:00 Jan 1, 1970 UTC) */
} XCOMGreenlightSmartphoneType;
/**
 * #domain: hidden
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMGreenlightHeader greenlight_header;
    XCOMGreenlightInsType pva;
    XCOMFooter footer;
} XCOMmsg_GREENLIGHTLOCAL;
/**
 * #domain: hidden
 * #rate: event
 */
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMGreenlightHeader greenlight_header;
    XCOMGreenlightSmartphoneType pva;
    XCOMFooter footer;
} XCOMmsg_GREENLIGHTSMARTPHONE;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMPluginDataHeader plugin_header;
    XCOMGreenlightHeader greenlight_header;
    XCOMGreenlightInsType pva;
    XCOMFooter footer;
} XCOMmsg_GREENLIGHTLOCALPLUGIN;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMPluginDataHeader plugin_header;
    XCOMGreenlightHeader greenlight_header;
    XCOMGreenlightSmartphoneType pva;
    XCOMFooter footer;
} XCOMmsg_GREENLIGHTSMARTPHONEPLUGIN;
enum XCOMmsgGreenlightModemConnectionState {
    CONNECTION_DOWN       = 0,
    CONNECTION_CONNECTING = 1,
    CONNECTION_UP         = 2,
    CONNECTION_LIMITED_UP = 3,
    CONNECTION_CLOSING    = 4,
    CONNECTION_UNKNOWN    = 5
};
enum XCOMmsgGreenlightModemServiceState {
    SERVICE_UNKNOWN    = 0,
    SERVICE_ALLOCATED  = 2,
    SERVICE_CONNECTING = 3,
    SERVICE_UP         = 4,
    SERVICE_CLOSING    = 5,
    SERVICE_DOWN       = 6
};
enum XCOMmsgGreenlightModemSocketType {
    SOCKET_UNASSIGNED_POP3 = 0,
    SOCKET_UNASSIGNED      = 1,
    SOCKET_CLIENT          = 2,
    SOCKET_LISTENER        = 3,
    SOCKET_SERVER          = 4
};
enum XCOMmsgGreenlightModemSignalRssi {
    RSSI_NOSIG     = 0,  /** < -113 dBm or less */
    RSSI_BAD       = 1,  /** < -111 dBm */
    RSSI_GOOD      = 2,  /** < 2..30: -109..-53 dBm */
    RSSI_EXCELLENT = 31, /** < -51 dBm or greater */
    RSSI_UNKNOWN   = 99  /** < not known or not detectable */
};
enum XCOMmsgGreenlightLoginState {
    GREENLIGHT_LOGIN_UNKNOWN    = 0,
    GREENLIGHT_LOGIN_REQUESTED  = 1,
    GREENLIGHT_LOGIN_COMPLETE   = 2,
    GREENLIGHT_LOGOUT_REQUESTED = 3,
    GREENLIGHT_LOGOUT_COMPLETE  = 4
};
typedef struct {
    uint8_t category;              /**< Alarm type to be triggered, see XCOMGreenlightAlarmCategory */
    uint8_t source_identifier;     /**< Source of last triggered alarm, see XCOMGreenlightSourceIdentifier */
    uint16_t gps_time_week;        /**< GPS week of last triggered alarm */
    uint32_t gps_time_sec;         /**< GPS time of week of last triggered alarm */
    uint8_t distance_threshold;    /**< Distance warning threshold (only evaluated for XCOM_ALARM_DISTANCE) */
    uint8_t reserved[3];           /**< Reserved for further use */
} XCOMPar_GREENLIGHTALARM_payload;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMParHeader param_header;     /**< XCOM parameter header */
    XCOMPluginHeader plugin_header; /**< plugin_param_id = XCOM_PLUGINID_GREENLIGHT_ALARM */
    XCOMPar_GREENLIGHTALARM_payload alarm;
    XCOMFooter footer;
} XCOMPar_GREENLIGHTALARM;
#define XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE 32
typedef struct {
    uint8_t manufacturer_id[XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE];
    uint8_t model_id[XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE];
    uint8_t revision_id[XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE];
    uint8_t imei[XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE]; /**< International Mobile Equipment Id */
    uint8_t imsi[XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE]; /**< International Mobile Subscriber Id */
    uint8_t sim_blocked;
} XCOMPar_GREENLIGHTMODEMINFO_payload;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;              /**< XCOM header */
    XCOMParHeader param_header;     /**< XCOM parameter header */
    XCOMPluginHeader plugin_header; /**< plugin_param_id = XCOM_PLUGINID_GREENLIGHT_MODEM_INFO */
    XCOMPar_GREENLIGHTMODEMINFO_payload modem_info;
    XCOMFooter footer;
} XCOMPar_GREENLIGHTMODEMINFO;
typedef struct {
    uint8_t gnss_connected;   /** < TCP Connection to GNSS receiver (1: connected) */
    uint8_t gnss_pos_type;    /** < GNSS position solution type */
    uint8_t gsmr_connection;  /** < GSMR connection state (see XCOMmsgGreenlightModemConnectionState) */
    uint8_t gsmr_service;     /** < GSRM service state (see XCOMmsgGreenlightModemServiceState) */
    uint8_t gsmr_rssi;        /** < GSMR modem signal quality (see XCOMmsgGreenlightModemSignalRssi) */
    uint8_t server_connected; /** < TCP Connection to communication server (1: connected) */
    uint8_t local_connected;  /** < TCP Connection to iQSoft CPU (1: connected) */
    uint8_t login_local;      /** < Login status of Greenlight-Local */
    uint8_t login_mobile;     /** < Login status of Greenlight-Smartphone */
} XCOMmsg_GREENLIGHTSTATUS_payload;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMPluginDataHeader plugin_header;
    XCOMmsg_GREENLIGHTSTATUS_payload status;
    XCOMFooter footer;
} XCOMmsg_GREENLIGHTSTATUS;
typedef struct {
    uint8_t connection_profile;                                      /** < Connection profile id */
    uint8_t service_profile;                                         /** < Service profile id */
    uint32_t reconnect_counter;                                      /** < Number of reconnects */
    uint8_t service_type[XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE];   /** < Service type, eg "SOCKET" */
    uint8_t local_address[XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE];  /** < Local address of the internet service */
    uint8_t remote_address[XCOMMSG_GREENLIGHTPLUGIN_MODEM_STR_SIZE]; /** < Remote address of the internet service */
    uint8_t socket_state;                                            /** < Socket state - see XCOMmsgGreenlightModemSocketType */
    uint8_t connection_state;                                        /** < Connection state - see XCOMmsgGreenlightModemConnectionState */
    uint8_t connection_services;                                     /** < Number of services for this connection */
    uint8_t service_state;                                           /** < Service state - see XCOMmsgGreenlightModemServiceState*/
    uint32_t rx_count;                                               /** < Number of bytes received */
    uint32_t tx_count;                                               /** < Number of bytes sent */
    uint32_t ack_count;                                              /** < Number of acknowledged bytes */
    uint32_t unack_count;                                            /** < Number of unacknowledged bytes */
} XCOMmsg_GREENLIGHTMODEMSERVICE_payload;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMPluginDataHeader plugin_header;
    XCOMmsg_GREENLIGHTMODEMSERVICE_payload modem_service;
    XCOMFooter footer;
} XCOMmsg_GREENLIGHTMODEMSERVICE;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMPluginDataHeader plugin_header;
    uint8_t rssi_asu;       /** < Received signal strength indicator, see XCOMmsgGreenlightModemSignalRssi */
    int8_t dbm;             /** < Calculated dBm value of rssi (-113..-51 dBm) */
    uint8_t percent;        /** < Calculated percentage of rssi */
    uint8_t bit_error_rate; /** < Bit error rate indicating quality (only appliable on calls) */
    XCOMFooter footer;
} XCOMmsg_GREENLIGHTMODEMSIGQUAL;
typedef struct XCOM_STRUCT_PACK {
    int mobile_country_code;        /** < e.g. 232 (0: Not decoded) */
    int mobile_network_code;        /** < e.g. 003 (0: Not decoded) */
    int location_area_code;         /** < e.g. 0x4EED (0: Not decoded) */
    int cell_identifier;            /** < e.g. 0x4EAF (0: Not decoded) */
    int base_station_identity_code; /** < e.g. 32 (0: Not decoded) */
    int channel_no;                 /** Absolute frequency channel number (0: Not decoded) */
    int rssi;                       /** Received signal level of the BCCH carrier */
    int c1;                         /** Coefficient for base station reselection */
    int c2;                         /** Coefficient for base station reselection */
} XCOMGreenlightCellType;
#define XCOMMSG_GREENLIGHTPLUGIN_MODEM_CELL_NUM 7
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMPluginDataHeader plugin_header;
    uint8_t cells; /**< Number of cells with available information */
    XCOMGreenlightCellType cell_info[XCOMMSG_GREENLIGHTPLUGIN_MODEM_CELL_NUM];
    XCOMFooter footer;
} XCOMmsg_GREENLIGHTMODEMCELL;
/**
 * Greenlight specific commands
 */
enum XCOMGreenlightAlarmCategory {
    XCOM_ALARM_OFF      = 0,    /**< Turn off any alarm */
    XCOM_ALARM_WARNING  = 1,    /**< Greenlight Warning */
    XCOM_ALARM_CRITICAL = 2,    /**< Greenlight Critical */
    XCOM_ALARM_OK       = 3,    /**< TIM Ok */
    XCOM_ALARM_ERROR    = 4,    /**< TIM Error */
    /**< 5 used for testing by IQSoft */
    XCOM_ALARM_DISTANCE = 6,    /**< Distance threshold warning */
};
enum XCOMGreenlightDistanceThreshold {
    XCOM_DISTANCE_THRESHOLD_CAUTION = 0,
    XCOM_DISTANCE_THRESHOLD_WARNING,
    XCOM_DISTANCE_THRESHOLD_CRITICAL
};
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMCmdHeader cmd_header;               /**< XCOM command header */
    XCOMGreenlightHeader greenlight_header; /**< Greenlight header */
    uint8_t alarm_category;                 /**< Alarm category (see XCOMGreenlightAlarmCategory) */
    uint8_t distance_threshold;             /**< Distance threshold passed, see XCOMGreenlightDistanceThreshold
                                                 (evaluated only for alarm category XCOM_ALARM_DISTANCE) */
    uint8_t reserved[2];                    /**< Reserved for further use */
    XCOMFooter footer;
} XCOMcmd_GREENLIGHTALARM;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMCmdHeader cmd_header;               /**< XCOM command header */
    XCOMGreenlightHeader greenlight_header; /**< Greenlight header */
    uint8_t state;                          /**< Acknowledge-State
                                                    1: Acknowledge
                                                    0: Not Acknowledge */
    uint8_t msg_id;                         /**< XCOM Header Message ID */
    uint16_t specific_msg_id;               /**< XCOM Specific Message ID
                                                    0: Not used
                                                    else: XCOM Specific message ID, e.g., XCOM Command ID */
    uint8_t msg_counter;                    /**< XCOM Header message counter */
    uint8_t ack_number;                     /**< Acknowledge Number */
    uint16_t reserved;                      /**< Reserved for further use */
    XCOMFooter footer;
} XCOMcmd_GREENLIGHTACK;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMCmdHeader cmd_header;               /**< XCOM command header */
    XCOMGreenlightHeader greenlight_header; /**< Greenlight header */
    uint8_t control_word;                   /**< Greenlight control word; */
    uint8_t reserved[3];                    /**< Reserved for further use */
    uint16_t element_description;           /**< Greenlight element description; */
    uint16_t element_type;                  /**< Greenlight element type; */
    XCOMFooter footer;
} XCOMcmd_GREENLIGHTSIGNALSTATE;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                      /**< XCOM header */
    XCOMCmdHeader cmd_header;               /**< XCOM command header */
    XCOMGreenlightHeader greenlight_header; /**< Greenlight header */
    uint8_t state;                          /**< Login-State
                                                   0: Logout
                                                   1: Login
                                                   2: Ready */
    uint8_t reserved[3];                    /**< Reserved for further use */
    uint32_t greenlight_token;              /**< Greenlight token for security mechanism */
    uint8_t greenlight_version[16];         /**< Greenlight local version number*/
    uint8_t inat_version[16];               /**< iNAT/Plugin version number*/
    XCOMFooter footer;
} XCOMcmd_GREENLIGHTLOGIN;
/**
 * This message contains the output of iMAR's heave algorithm.
 * #domain: public
 * #rate: event
 * #name: XCOMmsg_HEAVE
 */
#define XCOMMSG_PLUGINID_HEAVE XCOM_PLUGINID_HEAVE /**< Plugin ID of heave message */
typedef struct {
    uint32_t processing_error : 1;  /**< Processing error within the algorithm */
    uint32_t reserved         : 31; /**< Reserved for further use */
} XCOMHeaveStatusBits;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                  /**< XCOM header */
    XCOMPluginDataHeader plugin_header; /**< Plugin header */
    uint16_t gps_week;                  /**< GPS week of the processed epoch */
    double gps_time;                    /**< GPS time of week of the processed epoch [s] */
    double position_est;                /**< Estimated vertical position [m] */
    double velocity_est;                /**< Estimated vertical verlocity in [m] */
    double accel_est;                   /**< Estimated vertical acceleration [m/s/s] */
    double accel_bias_est;              /**< Estimated vertical accelerometer bias in [m/s/s] */
    union {                             /**< Heave status information */
        XCOMHeaveStatusBits bits;
        uint32_t value;
    } status;
    XCOMFooter footer;
} XCOMmsg_HEAVE;
/**
 * Status bits of Ingenia motor controller
 */
typedef struct {
    uint32_t fault_bit_set                : 1;
    uint32_t motor_quick_stop             : 1;
    uint32_t motor_ready_to_pwr_on        : 1;
    uint32_t motor_pwr_is_on              : 1;
    uint32_t motor_operation_mode_enabled : 1;
    uint32_t motor_voltage_enabled        : 1;
    uint32_t motor_target_pos_reached     : 1;
    uint32_t motor_set_point_ack          : 1;
    uint32_t motor_following_error        : 1;
} XCOMIATTHEMO_Status_bits;
/**
 * This messages includes internal values of servo controller.
 * These values are updated with a rate of 10 Hz.
 * #name: XCOMmsg_IATTHEMODATA
 */
#define XCOMMSG_PLUGINID_IATTHEMO_DATA XCOM_PLUGINID_ATTHEMODATA
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    uint16_t plugin_data_id;
    uint16_t reserved;
    uint32_t timestamp;
    int32_t servo_state;     /**< Init = 0, Homing = 1, PowerOn = 2, PowerOff = 3, Error = 4*/
    int32_t servo_ctrl_type; /**< None = 0, Current = 1, Velocity = 2, Position = 3*/
    int32_t motor_state;     /**< On = 1, Off = 0 */
    int32_t homing_state;    /**< NotActive = 0, Active = 1, WaitForConfirm = 2, Error = 3, Finished = 4 */
    int32_t pos_offset_tick; /**< Zero position offset in encoder increments */
    int32_t enc_pos_tick;    /**< Encoder increments of current position */
    uint32_t enc_max_tick;   /**< Maximum of encoder increments */
    int32_t pos_min_tick;    /**< Minimum position in encoder increments */
    int32_t pos_max_tick;    /**< Maximum position in encoder increments */
    float enc_pos_angle;     /**< Current encoder position as angle in rad */
    uint32_t error_counter;  /**< CAN Bus error counter */
    union {
        XCOMIATTHEMO_Status_bits bits;
        uint32_t value;
    } status; /**< Status bits: see above */
    XCOMFooter footer;
} XCOMmsg_IATTHEMODATA;
/**
 * TEC controller data package.
 * #name: XCOMmsg_TECDATA
 */
#define XCOMMSG_PLUGINID_TECDATA                      XCOM_PLUGINID_TECDATA /**< Plugin ID of TEC data message */
#define XCOMMSG_PLUGIN_TECDATA_NUMBER_OF_TEMPERATURES 5                     /**< Number of available temperature channels */
#define XCOMMSG_PLUGIN_TECDATA_NUMBER_OF_FANSPEEDS    4                     /**< Number of available fans */
#define XCOMMSG_PLUGIN_TECDATA_NUMBER_OF_VOLTAGES     5                     /**< Number of available voltage channels */
#define XCOMMSG_PLUGIN_TECDATA_NUMBER_OF_CURRENTS     2                     /**< Number of available current channels */
typedef struct {
    uint32_t resetcause_error              : 1; /**< Abnormal reset cause */
    uint32_t temp_ext_ovr                  : 1; /**< External temperature changed */
    uint32_t eeprom_error                  : 1; /**< EEPROM read/write error */
    uint32_t factory_reset_done            : 1; /**< Factoryreset occurred due to wrong EEPROM CRC */
    uint32_t temp_stable                   : 1; /**< Temperature is stabilized */
    uint32_t inat_timeout_error            : 1; /**< No iNAT XCOMmsg_TECCMD received for specified time */
    uint32_t over_temperature_error        : 1; /**< Set if both Peltier elements are powered on and voltage is below 5 V */
    uint32_t peltier1_temperature_error    : 1; /**< If temperature delta or maximum temperature is reached. */
    uint32_t tec_error                     : 1; /**< Not set yet */
    uint32_t current_peltier1_error        : 1; /**< When cooling/heating and current below threshold. */
    uint32_t current_peltier2_error        : 1; /**< When cooling/heating and current below threshold. */
    uint32_t peltier2_temperature_error    : 1; /**< If temperature delta or maximum temperature is reached. */
    uint32_t fan_error                     : 1; /**< If any fan tacho signal is below 10 Hz */
    uint32_t calibrating                   : 1; /**< Currently calibrating */
    uint32_t temp_runaway                  : 1; /**< Temperature was out of controlled range at least once */
    uint32_t reserved01                    : 1; /**< +- 5 % tolerance */
    uint32_t voltage_3v3_error             : 1; /**< +- 5 % tolerance */
    uint32_t voltage_5v_error              : 1; /**< +- 5 % tolerance */
    uint32_t voltage_fan_error             : 1; /**< +- 5 % tolerance */
    uint32_t voltage_peltier_error         : 1; /**< +- 5 % tolerance */
    uint32_t voltage_15v_error             : 1; /**< +- 5 % tolerance */
    uint32_t i2c_temperature_missing_error : 1; /**< I2C Temperature sensor not found */
    uint32_t temperature_error             : 1; /**< Analog Temperature Sensor is below -60 ° C */
    uint32_t init_remaining                : 8; /**< Init time left (min) */
    uint32_t reserved02                    : 1;
} XCOMTecDataStatusBits;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                                                 /**< XCOM header */
    XCOMPluginDataHeader plugin_header;                                /**< Plugin header: ID has to set to XCOMMSG_PLUGINID_TECDATA */
    float temperatures[XCOMMSG_PLUGIN_TECDATA_NUMBER_OF_TEMPERATURES]; /**< Temperature measurements in [°C] */
    uint32_t fan_speed[XCOMMSG_PLUGIN_TECDATA_NUMBER_OF_FANSPEEDS];    /**< Fan speed measurements in [rpm] */
    float voltages[XCOMMSG_PLUGIN_TECDATA_NUMBER_OF_VOLTAGES];         /**< Voltage measurements in [V] */
    float currents[XCOMMSG_PLUGIN_TECDATA_NUMBER_OF_CURRENTS];         /**< Current measurements in [A] */
    union {
        XCOMTecDataStatusBits bits; /**< TEC controller error status */
        uint32_t value;
    } error_status;
    double uc_timer;          /**< TEC contoroller uptime in [s] */
    uint32_t fw_version;      /**< TEC controller firmware version */
    uint32_t checksum_eeprom; /**< EEPROM checksum */
    float primary_voltage;    /**< Primary voltage in V */
    XCOMFooter footer;
} XCOMmsg_TECDATA;
typedef struct XCOM_STRUCT_PACK {
    // inat_iTempstab #47
    XCOMHeader header;                  /**< XCOM header */
    XCOMPluginDataHeader plugin_header; /**< Plugin header: ID has to set to XCOMMSG_PLUGINID_TECDATA */
    float Ta;                           /**< Ambient Temperature from Ta-Model in [°C] */
    float Qa;                           /**< heat flux from imu to ambient temperature [W] */
    float pwm_extra;                    /**< Additional Perltier-PWM to compensate for Qa [%] */
    float Ip_extra;                     /**< Additional Current correspronding to pwm_extra [A] */
    float PID_info[4];                  /**< Referencing PID errors [PIDErr FRONT, IntErr FRONT, PIDErr BACK, IntErr BACK] */
    uint8_t reserved[4]; /**< Reserved for further use */ /**< Reserved for further use */
    XCOMFooter footer;
} XCOMmsg_FeedforwardCtrlData;
//**
/* iPEGASUS specific plugin parameters
*/
enum class PEGASUS_MSG_ID : uint16_t {
    CMD_ALIGN                  = 0,
    PAR_QUAT                   = 100,
    PAR_EARTHRATE              = 101,
    PAR_MIN_ALIGNTIME          = 102,
    PAR_MAX_DEV_EARTHRATE      = 103,
    PAR_SHAFT_MISALIGNMENT     = 104,
    PAR_USE_SHAFT_MISALIGNMENT = 105,
    NOT_USED                   = 0xFFFF
};
typedef struct XCOM_STRUCT_PACK {
    PEGASUS_MSG_ID msg_id;
    uint16_t reserved;
} XCOMPegausHeader;
struct XCOMPegasusStatusType {
    uint32_t alignment                 : 1; /** 1 = Alignment running */
    uint32_t in_motion                 : 1; /** 1 = Motion detected */
    uint32_t alignment_in_motion       : 1; /** 1 = Alignment invalid due to motion detection */
    uint32_t min_align_time_reached    : 1; /** 1 = alignment_time > XCOMParPEGASUS_ParMinAligntime */
    uint32_t earthrate_error           : 1; /** 1 = Deviation of determined earth rotation rate > XCOMParPEGASUS_ParMaxDevEarthRate */
    uint32_t shaft_misalignment_active : 1; /** 1 = XCOMParPEGASUS_ParShaftMisalignment active */
    uint32_t res                       : 26;
};
typedef struct XCOM_STRUCT_PACK {
    double quat[4];               /**< Relative orientation to initial orientation as quaternion */
    double phi[3];                /**< Accumulated angle increments per axis, i.e. not redundant to the quaternion [rad, rad, rad] */
    float std_azi;                /**< Estimated azimuth/yaw standard deviation TBD */
    float std_tilt;               /**< Estimated tilt standard deviation TBD*/
    float time_since_align;       /**< Duration since alignmnet finished [s] */
    float alignment_time;         /**< Duration since alignmnet started [s] */
    XCOMPegasusStatusType status; /**< PEGASUS Status Bits */
    double gps_time;              /**< GPS/System Time depending on configuration [s] */
    double absolute_rpy[3];       /**< Absolute orientation as Euler Angles [rad, rad, rad] */
    uint32_t reserved;
} XCOMmsg_PEGASUSLOG_payload;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMPluginDataHeader plugin_header;
    XCOMmsg_PEGASUSLOG_payload log;
    XCOMFooter footer;
} XCOMmsg_PEGASUSLOG;
/**
 * End of iPEGASUS specific content.
 */
#define XCOMMSG_PLUGINID_DBXDBOUT                         XCOM_PLUGINID_DBXDBOUT
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
#define XCOMMSG_PLUGINID_DBXDBOUT_SATS_USED_MAX           15
#define XCOMMSG_PLUGINID_DBXDBOUT_POSERROR_VALUE          0xffff
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
    uint8_t GNSS_SATS_USED : 4;  // satellites used
    uint8_t GNSS_POS_USED  : 1;
    uint8_t GNSS_DIFF_SOL  : 1;  // 0: single point solution, 1: differential/RTK solution
    uint8_t GNSS_TIMEREF   : 2;  // see XCOMMSG_PLUGINID_DBXDBOUT_STATUS_GNSSTIME
} XCOMDbxDbOutputStatusGps1Bits;
typedef union {
    XCOMDbxDbOutputStatusGps1Bits bits;
    uint8_t value;
} XCOMDbxDbOutputStatusGps1Type;
typedef struct {
    uint8_t GNSS_NODATA      : 1;  // no data from GNSS receiver
    uint8_t GNSS_SW_RESET    : 1;  // unused
    uint8_t GNSS_MEM_ERROR   : 1;  // unused
    uint8_t GNSS_BAT_EMPTY   : 1;  // unused
    uint8_t GNSS_NODATA_DIFF : 1;  // No DPGS data on GNSS receiver8
} XCOMDbxDbOutputStatusGps2Bits;
typedef union {
    XCOMDbxDbOutputStatusGps2Bits bits;
    uint8_t value;
} XCOMDbxDbOutputStatusGps2Type;
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;                         /**< XCOM header */
    XCOMPluginDataHeader plugin_header;        /**< plugin header */
    XCOMDbxDbOutputStatusInatType status_inat; /**< inat status flags */
    uint8_t status_bite;                       /**< sagem IMU status, not used */
    XCOMDbxDbOutputStatusGps1Type status_gps1; /**< gnss status flags (1) */
    XCOMDbxDbOutputStatusGps2Type status_gps2; /**< gnss status flags (2) */
    double gps_time;                           /**< GPS time of week of the processed epoch [s] */
    double nav_lat;                            /**< latitude of navigation solution [rad] */
    double nav_lon;                            /**< longitude of navigation solution [rad] */
    double nav_height;                         /**< height of navigation solution [m] */
    float drms;                                /**< square root from sum of lat and lon covariances of ins solution */
    double gnss_lat;                           /**< latitude of GNSS position [rad] */
    double gnss_lon;                           /**< longitude of GNSS position [rad] */
    double gnss_height;                        /**< height of GNSS position [m] */
    float gnss_ehe;                            /**< expected horizontal error of GNSS position solution [m] */
    float vel_north;                           /**< north velocity of navigation solution [m/s] */
    float vel_east;                            /**< east velocity of navigation solution [m/s] */
    float vel_down;                            /**< down velocity of navigation solution [m/s] */
    float sog;                                 /**< speed over ground of navigation solution [m/s] */
    float heading;                             /**< heading of navigation solution [rad] */
    float roll;                                /**< roll angle of navigation solultion [rad] */
    float pitch;                               /**< pitch angle of navigation solution [rad] */
    XCOMFooter footer;
} XCOMmsg_DbxDbOut;
typedef struct {
    uint8_t HPOS_VALID    : 1; /**< validity of horizontal position and drms values */
    uint8_t TIMEREF_VALID : 1; /**< validity of provided timestamp */
    uint8_t DATA_SYNC     : 1; /**< sync data with INS time */
    uint8_t HEIGHT_VALID  : 1; /**< validity of height value */
    uint8_t RESTART       : 1; /**< indicates if navigation should get stopped and re-alignment initiated */
    uint8_t IN_MOTION     : 1; /**< indicates if vehicle is in motion */
    uint8_t USE_GNSS      : 1; /**< indicates if GNSS position/velocity should be used for aiding */
    uint8_t ATT_VALID     : 1; /**< validity of rail-pitch, alpha-c, beta-c and gamma-c values */
} XCOMDbxDbInputStatusBits;
typedef union {
    XCOMDbxDbInputStatusBits bits;
    uint8_t value;
} XCOMDbxDbInputStatusType;
#define XCOMPAR_PLUGINID_DBXDBIN XCOM_PLUGINID_DBXDBIN
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMParHeader param_header;
    XCOMPluginHeader plugin_header;
    XCOMDbxDbInputStatusType status;
    double gps_time;  /**< gps timestamp used for processing data if valid [s] */
    float speed;      /**< not used (velocities used from odometer) [m/s] */
    double lat;       /**< latitude [rad] */
    double lon;       /**< longitude [rad] */
    double height;    /**< height [m] */
    float drms;       /**< distance root mean squared [m] */
    float rail_pitch; /**< pitch of the rail [rad] */
    float alpha;      /**< roll angle between IMU and bogie [rad] */
    float beta;       /**< pitch angle between IMU and bogie [rad] */
    float gamma;      /**< yaw angle between IMU and bogie [rad] */
    float delta;      /**< altitude difference between IMU and bogie [m] */
    XCOMFooter footer;
} XCOMParDBXDB_IN;
#define XCOMPAR_PLUGINID_DBXDBCONF XCOM_PLUGINID_DBXDBCONF
typedef struct XCOM_STRUCT_PACK {
    XCOMHeader header;
    XCOMParHeader param_header;
    XCOMPluginHeader plugin_header;
    uint8_t port_in;             /**< serial port input message is received */
    uint32_t baudrate_in;        /**< baudrate of serial input port */
    uint16_t divider_in;         /**< divider to determine interval of input position used for aiding */
    uint8_t port_out;            /**< serial port outmessage is sent */
    uint32_t baudrate_out;       /**< baudrate of serial output port */
    uint16_t divider_out;        /**< divider to determine output message interval in relation to IMU sampling rate */
    float undulation;            /**< undulation in [m] used to convert input height (geoidal) to WGS84 */
    uint8_t use_gnss_undulation; /**< if 0 fixed undulation value is used to convert height (see above) */
    uint16_t status_interval;    /**< time passed in [ms] from last input message until status is completely evaluated (not only changes) */
    XCOMFooter footer;
} XCOMParDBXDB_CONF;
#endif
#endif  //PLUGIN_H
#endif
// NOLINTEND
// clang-format on
