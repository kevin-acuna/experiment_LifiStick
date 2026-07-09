/******************************************************************************
 *                                                                         
 * Copyright 2012 Keithley Instruments. All rights reserved.
 *
 *****************************************************************************/

#ifndef __KE2100_HEADER
#define __KE2100_HEADER

#include "IviVisaType.h"

#if defined(__cplusplus) || defined(__cplusplus__)
extern "C" {
#endif

/**************************************************************************** 
 *---------------------------- Attribute Defines ---------------------------* 
 ****************************************************************************/
#ifndef IVI_ATTR_BASE
#define IVI_ATTR_BASE                 1000000
#endif

#ifndef IVI_INHERENT_ATTR_BASE		        
#define IVI_INHERENT_ATTR_BASE        (IVI_ATTR_BASE +  50000)   /* base for inherent capability attributes */
#endif

#ifndef IVI_CLASS_ATTR_BASE           
#define IVI_CLASS_ATTR_BASE           (IVI_ATTR_BASE + 250000)   /* base for IVI-defined class attributes */
#endif

#ifndef IVI_LXISYNC_ATTR_BASE         
#define IVI_LXISYNC_ATTR_BASE         (IVI_ATTR_BASE + 950000)   /* base for IviLxiSync attributes */
#endif

#ifndef IVI_SPECIFIC_ATTR_BASE        
#define IVI_SPECIFIC_ATTR_BASE        (IVI_ATTR_BASE + 150000)   /* base for attributes of specific drivers */
#endif


/*===== IVI Inherent Instrument Attributes ==============================*/    

/*- Driver Identification */

#define KE2100_ATTR_SPECIFIC_DRIVER_DESCRIPTION              (IVI_INHERENT_ATTR_BASE + 514L)  /* ViString, read-only */
#define KE2100_ATTR_SPECIFIC_DRIVER_PREFIX                   (IVI_INHERENT_ATTR_BASE + 302L)  /* ViString, read-only */
#define KE2100_ATTR_SPECIFIC_DRIVER_VENDOR                   (IVI_INHERENT_ATTR_BASE + 513L)  /* ViString, read-only */
#define KE2100_ATTR_SPECIFIC_DRIVER_REVISION                 (IVI_INHERENT_ATTR_BASE + 551L)  /* ViString, read-only */
#define KE2100_ATTR_SPECIFIC_DRIVER_CLASS_SPEC_MAJOR_VERSION (IVI_INHERENT_ATTR_BASE + 515L)  /* ViInt32, read-only */
#define KE2100_ATTR_SPECIFIC_DRIVER_CLASS_SPEC_MINOR_VERSION (IVI_INHERENT_ATTR_BASE + 516L)  /* ViInt32, read-only */

/*- User Options */

#define KE2100_ATTR_RANGE_CHECK                             (IVI_INHERENT_ATTR_BASE + 2L)  /* ViBoolean, read-write */
#define KE2100_ATTR_QUERY_INSTRUMENT_STATUS                 (IVI_INHERENT_ATTR_BASE + 3L)  /* ViBoolean, read-write */
#define KE2100_ATTR_CACHE                                   (IVI_INHERENT_ATTR_BASE + 4L)  /* ViBoolean, read-write */
#define KE2100_ATTR_SIMULATE                                (IVI_INHERENT_ATTR_BASE + 5L)  /* ViBoolean, read-write */
#define KE2100_ATTR_RECORD_COERCIONS                        (IVI_INHERENT_ATTR_BASE + 6L)  /* ViBoolean, read-write */
#define KE2100_ATTR_INTERCHANGE_CHECK                       (IVI_INHERENT_ATTR_BASE + 21L)  /* ViBoolean, read-write */

/*- Advanced Session Information */

#define KE2100_ATTR_LOGICAL_NAME                            (IVI_INHERENT_ATTR_BASE + 305L)  /* ViString, read-only */
#define KE2100_ATTR_IO_RESOURCE_DESCRIPTOR                  (IVI_INHERENT_ATTR_BASE + 304L)  /* ViString, read-only */
#define KE2100_ATTR_DRIVER_SETUP                            (IVI_INHERENT_ATTR_BASE + 7L)  /* ViString, read-only */

/*- Driver Capabilities */

#define KE2100_ATTR_GROUP_CAPABILITIES                      (IVI_INHERENT_ATTR_BASE + 401L)  /* ViString, read-only */
#define KE2100_ATTR_SUPPORTED_INSTRUMENT_MODELS             (IVI_INHERENT_ATTR_BASE + 327L)  /* ViString, read-only */

/*- Instrument Identification */

#define KE2100_ATTR_INSTRUMENT_FIRMWARE_REVISION            (IVI_INHERENT_ATTR_BASE + 510L)  /* ViString, read-only */
#define KE2100_ATTR_INSTRUMENT_MANUFACTURER                 (IVI_INHERENT_ATTR_BASE + 511L)  /* ViString, read-only */
#define KE2100_ATTR_INSTRUMENT_MODEL                        (IVI_INHERENT_ATTR_BASE + 512L)  /* ViString, read-only */


/*===== Instrument-Specific Attributes =====================================*/

/*- Basic Operation */

#define KE2100_ATTR_FUNCTION                                (IVI_CLASS_ATTR_BASE + 1L)  /* ViInt32, read-write */
#define KE2100_ATTR_RANGE                                   (IVI_CLASS_ATTR_BASE + 2L)  /* ViReal64, read-write */
#define KE2100_ATTR_RESOLUTION_ABSOLUTE                     (IVI_CLASS_ATTR_BASE + 8L)  /* ViReal64, read-write */

/*- AC Measurements */

#define KE2100_ATTR_AC_MAX_FREQ                             (IVI_CLASS_ATTR_BASE + 7L)  /* ViReal64, read-write */
#define KE2100_ATTR_AC_MIN_FREQ                             (IVI_CLASS_ATTR_BASE + 6L)  /* ViReal64, read-write */

/*- Configuration Information */

#define KE2100_ATTR_AUTO_RANGE_VALUE                        (IVI_CLASS_ATTR_BASE + 331L)  /* ViReal64, read-only */
#define KE2100_ATTR_APERTURE_TIME                           (IVI_CLASS_ATTR_BASE + 321L)  /* ViReal64, read-only */
#define KE2100_ATTR_APERTURE_TIME_UNITS                     (IVI_CLASS_ATTR_BASE + 322L)  /* ViInt32, read-only */

/*- Measurement Operation Options */

#define KE2100_ATTR_AUTO_ZERO                               (IVI_CLASS_ATTR_BASE + 332L)  /* ViInt32, read-write */
#define KE2100_ATTR_POWERLINE_FREQ                          (IVI_CLASS_ATTR_BASE + 333L)  /* ViReal64, read-write */

/*- Frequency Measurements */

#define KE2100_ATTR_FREQ_VOLTAGE_RANGE                      (IVI_CLASS_ATTR_BASE + 101L)  /* ViReal64, read-write */

/*- Multi-Point Acquisition */

#define KE2100_ATTR_TRIGGER_COUNT                           (IVI_CLASS_ATTR_BASE + 304L)  /* ViInt32, read-write */
#define KE2100_ATTR_MEAS_COMPLETE_DEST                      (IVI_CLASS_ATTR_BASE + 305L)  /* ViInt32, read-write */
#define KE2100_ATTR_SAMPLE_COUNT                            (IVI_CLASS_ATTR_BASE + 301L)  /* ViInt32, read-write */
#define KE2100_ATTR_SAMPLE_INTERVAL                         (IVI_CLASS_ATTR_BASE + 303L)  /* ViReal64, read-write */
#define KE2100_ATTR_SAMPLE_TRIGGER                          (IVI_CLASS_ATTR_BASE + 302L)  /* ViInt32, read-write */

/*- Temperature Measurements */

#define KE2100_ATTR_TEMP_TRANSDUCER_TYPE                    (IVI_CLASS_ATTR_BASE + 201L)  /* ViInt32, read-write */

/*- Resistance Temperature Device */

#define KE2100_ATTR_TEMP_RTD_ALPHA                          (IVI_CLASS_ATTR_BASE + 241L)  /* ViReal64, read-write */
#define KE2100_ATTR_TEMP_RTD_RES                            (IVI_CLASS_ATTR_BASE + 242L)  /* ViReal64, read-write */

/*- Thermistor */

#define KE2100_ATTR_TEMP_THERMISTOR_RES                     (IVI_CLASS_ATTR_BASE + 251L)  /* ViReal64, read-write */

/*- Thermocouple */

#define KE2100_ATTR_TEMP_TC_FIXED_REF_JUNC                  (IVI_CLASS_ATTR_BASE + 233L)  /* ViReal64, read-write */
#define KE2100_ATTR_TEMP_TC_REF_JUNC_TYPE                   (IVI_CLASS_ATTR_BASE + 232L)  /* ViInt32, read-write */
#define KE2100_ATTR_TEMP_TC_TYPE                            (IVI_CLASS_ATTR_BASE + 231L)  /* ViInt32, read-write */

/*- Trigger */

#define KE2100_ATTR_TRIGGER_DELAY                           (IVI_CLASS_ATTR_BASE + 5L)  /* ViReal64, read-write */
#define KE2100_ATTR_TRIGGER_SLOPE                           (IVI_CLASS_ATTR_BASE + 334L)  /* ViInt32, read-write */
#define KE2100_ATTR_TRIGGER_SOURCE                          (IVI_CLASS_ATTR_BASE + 4L)  /* ViInt32, read-write */

/*- Instrument Specific */

#define KE2100_ATTR_FUNCTION4                               (IVI_SPECIFIC_ATTR_BASE + 47L)  /* ViInt32, read-write */

/*- Status */

#define KE2100_ATTR_QUEST_ENABLE_REGISTER                   (IVI_SPECIFIC_ATTR_BASE + 1L)  /* ViInt32, read-write */
#define KE2100_ATTR_QUEST_EVENT_REGISTER                    (IVI_SPECIFIC_ATTR_BASE + 2L)  /* ViInt32, read-only */

/*- System */

#define KE2100_ATTR_DISPLAY_STATE                           (IVI_SPECIFIC_ATTR_BASE + 3L)  /* ViBoolean, read-write */
#define KE2100_ATTR_BEEPER_STATE                            (IVI_SPECIFIC_ATTR_BASE + 4L)  /* ViBoolean, read-write */
#define KE2100_ATTR_VERSION                                 (IVI_SPECIFIC_ATTR_BASE + 5L)  /* ViString, read-only */
#define KE2100_ATTR_CONTROL_MODE                            (IVI_SPECIFIC_ATTR_BASE + 6L)  /* ViInt32, write-only */

/*- Math */

#define KE2100_ATTR_FUNCTION2                               (IVI_SPECIFIC_ATTR_BASE + 7L)  /* ViInt32, read-write */
#define KE2100_ATTR_STATE                                   (IVI_SPECIFIC_ATTR_BASE + 8L)  /* ViBoolean, read-write */
#define KE2100_ATTR_PERCENT_TARGET                          (IVI_SPECIFIC_ATTR_BASE + 9L)  /* ViReal64, read-write */
#define KE2100_ATTR_AVERAGE_MIN                             (IVI_SPECIFIC_ATTR_BASE + 10L)  /* ViReal64, read-only */
#define KE2100_ATTR_AVERAGE_MAX                             (IVI_SPECIFIC_ATTR_BASE + 11L)  /* ViReal64, read-only */
#define KE2100_ATTR_AVERAGE_AVERAGE                         (IVI_SPECIFIC_ATTR_BASE + 12L)  /* ViReal64, read-only */
#define KE2100_ATTR_AVERAGE_COUNT                           (IVI_SPECIFIC_ATTR_BASE + 13L)  /* ViReal64, read-only */
#define KE2100_ATTR_NULL_OFFSET                             (IVI_SPECIFIC_ATTR_BASE + 14L)  /* ViReal64, read-write */
#define KE2100_ATTR_DB_RELATIVE                             (IVI_SPECIFIC_ATTR_BASE + 15L)  /* ViReal64, read-write */
#define KE2100_ATTR_DBM_REFERENCE                           (IVI_SPECIFIC_ATTR_BASE + 16L)  /* ViReal64, read-write */

/*- Channels */

#define KE2100_ATTR_INPUT_TERMINAL_TYPE                     (IVI_SPECIFIC_ATTR_BASE + 17L)  /* ViInt32, read-only */
#define KE2100_ATTR_CLOSE                                   (IVI_SPECIFIC_ATTR_BASE + 18L)  /* ViInt32, read-write */

/*- ScannerCard */

#define KE2100_ATTR_IS_INSERTED                             (IVI_SPECIFIC_ATTR_BASE + 19L)  /* ViBoolean, read-only */
#define KE2100_ATTR_SCAN_TIME_INTERVAL                      (IVI_SPECIFIC_ATTR_BASE + 20L)  /* ViReal64, read-write */
#define KE2100_ATTR_SCAN_COUNT                              (IVI_SPECIFIC_ATTR_BASE + 21L)  /* ViReal64, read-write */
#define KE2100_ATTR_CHANNEL_SCAN                            (IVI_SPECIFIC_ATTR_BASE + 22L)  /* ViInt32, read-only */

/*- Measurement */

#define KE2100_ATTR_READ_MEMORY_STATE                       (IVI_SPECIFIC_ATTR_BASE + 64L)  /* ViBoolean, read-write */

/*- Configuration */

#define KE2100_ATTR_FUNCTION3                               (IVI_SPECIFIC_ATTR_BASE + 23L)  /* ViString, read-write */
#define KE2100_ATTR_DETECTOR_BANDWIDTH                      (IVI_SPECIFIC_ATTR_BASE + 24L)  /* ViReal64, read-write */
#define KE2100_ATTR_AUTO_ZERO2                              (IVI_SPECIFIC_ATTR_BASE + 25L)  /* ViInt32, read-write */
#define KE2100_ATTR_AUTO_GAIN                               (IVI_SPECIFIC_ATTR_BASE + 26L)  /* ViInt32, read-write */
#define KE2100_ATTR_AUTO_INPUT_IMPEDANCE                    (IVI_SPECIFIC_ATTR_BASE + 27L)  /* ViBoolean, read-write */
#define KE2100_ATTR_DIGITAL_FILTER_OPERATION_MODE           (IVI_SPECIFIC_ATTR_BASE + 28L)  /* ViInt32, read-write */
#define KE2100_ATTR_DIGITAL_FILTER_COUNT                    (IVI_SPECIFIC_ATTR_BASE + 29L)  /* ViInt32, read-write */
#define KE2100_ATTR_DIGITAL_FILTER_STATE                    (IVI_SPECIFIC_ATTR_BASE + 30L)  /* ViBoolean, read-write */

/*- Temperature */

#define KE2100_ATTR_UNIT                                    (IVI_SPECIFIC_ATTR_BASE + 31L)  /* ViInt32, read-write */

/*- TCouple */

#define KE2100_ATTR_SENSOR_TYPE                             (IVI_SPECIFIC_ATTR_BASE + 32L)  /* ViInt32, read-write */
#define KE2100_ATTR_REFERENCE_JUNCTION_TYPE                 (IVI_SPECIFIC_ATTR_BASE + 33L)  /* ViInt32, read-write */
#define KE2100_ATTR_SIMULATED_REFERENCE_JUNCTION            (IVI_SPECIFIC_ATTR_BASE + 34L)  /* ViReal64, read-write */
#define KE2100_ATTR_REAL_REFERENCE_JUNCTION                 (IVI_SPECIFIC_ATTR_BASE + 35L)  /* ViReal64, read-only */

/*- RTD */

#define KE2100_ATTR_TYPE                                    (IVI_SPECIFIC_ATTR_BASE + 36L)  /* ViInt32, read-write */

/*- AC */

#define KE2100_ATTR_FREQUENCY_MAX                           (IVI_SPECIFIC_ATTR_BASE + 39L)  /* ViReal64, read-write */
#define KE2100_ATTR_FREQUENCY_MIN                           (IVI_SPECIFIC_ATTR_BASE + 40L)  /* ViReal64, read-write */

/*- Advanced */

#define KE2100_ATTR_ACTUAL_RANGE                            (IVI_SPECIFIC_ATTR_BASE + 41L)  /* ViReal64, read-only */
#define KE2100_ATTR_APERTURE_TIME2                          (IVI_SPECIFIC_ATTR_BASE + 42L)  /* ViReal64, read-only */
#define KE2100_ATTR_APERTURE_TIME_UNITS2                    (IVI_SPECIFIC_ATTR_BASE + 43L)  /* ViInt32, read-only */
#define KE2100_ATTR_AUTO_ZERO3                              (IVI_SPECIFIC_ATTR_BASE + 44L)  /* ViInt32, read-write */
#define KE2100_ATTR_POWERLINE_FREQUENCY                     (IVI_SPECIFIC_ATTR_BASE + 45L)  /* ViReal64, read-write */

/*- Frequency */

#define KE2100_ATTR_VOLTAGE_RANGE                           (IVI_SPECIFIC_ATTR_BASE + 46L)  /* ViReal64, read-write */

/*- Temperature */

#define KE2100_ATTR_TRANSDUCER_TYPE                         (IVI_SPECIFIC_ATTR_BASE + 54L)  /* ViInt32, read-write */

/*- RTD */

#define KE2100_ATTR_ALPHA                                   (IVI_SPECIFIC_ATTR_BASE + 48L)  /* ViReal64, read-write */
#define KE2100_ATTR_RESISTANCE                              (IVI_SPECIFIC_ATTR_BASE + 49L)  /* ViReal64, read-write */

/*- Thermistor */

#define KE2100_ATTR_RESISTANCE2                             (IVI_SPECIFIC_ATTR_BASE + 50L)  /* ViReal64, read-write */

/*- Thermocouple */

#define KE2100_ATTR_FIXED_REF_JUNCTION                      (IVI_SPECIFIC_ATTR_BASE + 51L)  /* ViReal64, read-write */
#define KE2100_ATTR_REF_JUNCTION_TYPE                       (IVI_SPECIFIC_ATTR_BASE + 52L)  /* ViInt32, read-write */
#define KE2100_ATTR_TYPE2                                   (IVI_SPECIFIC_ATTR_BASE + 53L)  /* ViInt32, read-write */

/*- Trigger */

#define KE2100_ATTR_DELAY                                   (IVI_SPECIFIC_ATTR_BASE + 55L)  /* ViReal64, read-write */
#define KE2100_ATTR_SLOPE                                   (IVI_SPECIFIC_ATTR_BASE + 61L)  /* ViInt32, read-write */
#define KE2100_ATTR_SOURCE                                  (IVI_SPECIFIC_ATTR_BASE + 62L)  /* ViInt32, read-write */
#define KE2100_ATTR_AUTO_DELAY                              (IVI_SPECIFIC_ATTR_BASE + 63L)  /* ViBoolean, read-write */

/*- MultiPoint */

#define KE2100_ATTR_COUNT                                   (IVI_SPECIFIC_ATTR_BASE + 56L)  /* ViInt32, read-write */
#define KE2100_ATTR_MEASUREMENT_COMPLETE                    (IVI_SPECIFIC_ATTR_BASE + 57L)  /* ViInt32, read-write */
#define KE2100_ATTR_SAMPLE_COUNT2                           (IVI_SPECIFIC_ATTR_BASE + 58L)  /* ViInt32, read-write */
#define KE2100_ATTR_SAMPLE_INTERVAL2                        (IVI_SPECIFIC_ATTR_BASE + 59L)  /* ViReal64, read-write */
#define KE2100_ATTR_SAMPLE_TRIGGER2                         (IVI_SPECIFIC_ATTR_BASE + 60L)  /* ViInt32, read-write */

/*- InstrumentIO */

#define KE2100_ATTR_IO_SESSION                              (IVI_SPECIFIC_ATTR_BASE + 65L)  /* ViInt32, read-only */


/**************************************************************************** 
 *------------------------ Attribute Value Defines -------------------------* 
 ****************************************************************************/

/*- Defined values for 
	attribute KE2100_ATTR_FUNCTION
	parameter Function in function Ke2100_ConfigureMeasurement */

#define KE2100_VAL_DC_VOLTS                                 1
#define KE2100_VAL_AC_VOLTS                                 2
#define KE2100_VAL_DC_CURRENT                               3
#define KE2100_VAL_AC_CURRENT                               4
#define KE2100_VAL_2_WIRE_RES                               5
#define KE2100_VAL_4_WIRE_RES                               101
#define KE2100_VAL_FREQ                                     104
#define KE2100_VAL_PERIOD                                   105
#define KE2100_VAL_AC_PLUS_DC_VOLTS                         106
#define KE2100_VAL_AC_PLUS_DC_CURRENT                       107
#define KE2100_VAL_TEMPERATURE                              108

/*- Defined values for 
	attribute KE2100_ATTR_APERTURE_TIME_UNITS
	parameter ApertureTimeUnits in function Ke2100_GetApertureTimeInfo */

#define KE2100_VAL_SECONDS                                  0
#define KE2100_VAL_POWER_LINE_CYCLES                        1

/*- Defined values for 
	attribute KE2100_ATTR_AUTO_ZERO
	parameter AutoZeroMode in function Ke2100_ConfigureAutoZeroMode */

#define KE2100_VAL_AUTO_ZERO_OFF                            0
#define KE2100_VAL_AUTO_ZERO_ON                             1
#define KE2100_VAL_AUTO_ZERO_ONCE                           2

/*- Defined values for 
	attribute KE2100_ATTR_TEMP_TRANSDUCER_TYPE
	parameter TransducerType in function Ke2100_ConfigureTransducerType */

#define KE2100_VAL_THERMOCOUPLE                             1
#define KE2100_VAL_THERMISTOR                               2
#define KE2100_VAL_2_WIRE_RTD                               3
#define KE2100_VAL_4_WIRE_RTD                               4

/*- Defined values for 
	attribute KE2100_ATTR_TEMP_TC_TYPE
	parameter ThermocoupleType in function Ke2100_ConfigureThermocouple */

#define KE2100_VAL_TEMP_TC_B                                1
#define KE2100_VAL_TEMP_TC_C                                2
#define KE2100_VAL_TEMP_TC_D                                3
#define KE2100_VAL_TEMP_TC_E                                4
#define KE2100_VAL_TEMP_TC_G                                5
#define KE2100_VAL_TEMP_TC_J                                6
#define KE2100_VAL_TEMP_TC_K                                7
#define KE2100_VAL_TEMP_TC_N                                8
#define KE2100_VAL_TEMP_TC_R                                9
#define KE2100_VAL_TEMP_TC_S                                10
#define KE2100_VAL_TEMP_TC_T                                11
#define KE2100_VAL_TEMP_TC_U                                12
#define KE2100_VAL_TEMP_TC_V                                13

/*- Defined values for 
	attribute KE2100_ATTR_TEMP_TC_REF_JUNC_TYPE
	parameter RefJunctionType in function Ke2100_ConfigureThermocouple */

#define KE2100_VAL_TEMP_REF_JUNC_INTERNAL                   1
#define KE2100_VAL_TEMP_REF_JUNC_FIXED                      2

/*- Defined values for 
	attribute KE2100_ATTR_TRIGGER_SOURCE
	parameter TriggerSource in function Ke2100_ConfigureTrigger */

#define KE2100_VAL_IMMEDIATE                                1
#define KE2100_VAL_EXTERNAL                                 2
#define KE2100_VAL_SOFTWARE_TRIG                            3
#define KE2100_VAL_TTL0                                     111
#define KE2100_VAL_TTL1                                     112
#define KE2100_VAL_TTL2                                     113
#define KE2100_VAL_TTL3                                     114
#define KE2100_VAL_TTL4                                     115
#define KE2100_VAL_TTL5                                     116
#define KE2100_VAL_TTL6                                     117
#define KE2100_VAL_TTL7                                     118
#define KE2100_VAL_ECL0                                     119
#define KE2100_VAL_ECL1                                     120
#define KE2100_VAL_PXI_STAR                                 131
#define KE2100_VAL_RTSI_0                                   140
#define KE2100_VAL_RTSI_1                                   141
#define KE2100_VAL_RTSI_2                                   142
#define KE2100_VAL_RTSI_3                                   143
#define KE2100_VAL_RTSI_4                                   144
#define KE2100_VAL_RTSI_5                                   145
#define KE2100_VAL_RTSI_6                                   146

/*- Defined values for 
	attribute KE2100_ATTR_TRIGGER_SLOPE
	parameter Polarity in function Ke2100_ConfigureTriggerSlope */

#define KE2100_VAL_POSITIVE                                 0
#define KE2100_VAL_NEGATIVE                                 1

/*- Defined values for 
	attribute KE2100_ATTR_SAMPLE_TRIGGER
	parameter SampleTrigger in function Ke2100_ConfigureMultiPoint */

#define KE2100_VAL_IMMEDIATE                                1
#define KE2100_VAL_EXTERNAL                                 2
#define KE2100_VAL_SOFTWARE_TRIG                            3
#define KE2100_VAL_INTERVAL                                 10
#define KE2100_VAL_TTL0                                     111
#define KE2100_VAL_TTL1                                     112
#define KE2100_VAL_TTL2                                     113
#define KE2100_VAL_TTL3                                     114
#define KE2100_VAL_TTL4                                     115
#define KE2100_VAL_TTL5                                     116
#define KE2100_VAL_TTL6                                     117
#define KE2100_VAL_TTL7                                     118
#define KE2100_VAL_ECL0                                     119
#define KE2100_VAL_ECL1                                     120
#define KE2100_VAL_PXI_STAR                                 131
#define KE2100_VAL_RTSI_0                                   140
#define KE2100_VAL_RTSI_1                                   141
#define KE2100_VAL_RTSI_2                                   142
#define KE2100_VAL_RTSI_3                                   143
#define KE2100_VAL_RTSI_4                                   144
#define KE2100_VAL_RTSI_5                                   145
#define KE2100_VAL_RTSI_6                                   146

/*- Defined values for 
	attribute KE2100_ATTR_MEAS_COMPLETE_DEST
	parameter MeasCompleteDest in function Ke2100_ConfigureMeasCompleteDest */

#define KE2100_VAL_EXTERNAL                                 2
#define KE2100_VAL_TTL0                                     111
#define KE2100_VAL_TTL1                                     112
#define KE2100_VAL_TTL2                                     113
#define KE2100_VAL_TTL3                                     114
#define KE2100_VAL_TTL4                                     115
#define KE2100_VAL_TTL5                                     116
#define KE2100_VAL_TTL6                                     117
#define KE2100_VAL_TTL7                                     118
#define KE2100_VAL_ECL0                                     119
#define KE2100_VAL_ECL1                                     120
#define KE2100_VAL_PXI_STAR                                 131
#define KE2100_VAL_RTSI_0                                   140
#define KE2100_VAL_RTSI_1                                   141
#define KE2100_VAL_RTSI_2                                   142
#define KE2100_VAL_RTSI_3                                   143
#define KE2100_VAL_RTSI_4                                   144
#define KE2100_VAL_RTSI_5                                   145
#define KE2100_VAL_RTSI_6                                   146
#define KE2100_VAL_NONE                                     -1

/*- Defined values for */

#define KE2100_VAL_AUTO_RANGE_ONCE                          -3
#define KE2100_VAL_AUTO_RANGE_OFF                           -2
#define KE2100_VAL_AUTO_RANGE_ON                            -1

/*- Defined values for */

#define KE2100_VAL_AUTO_RANGE_OFF                           -2
#define KE2100_VAL_AUTO_RANGE_ON                            -1

/*- Defined values for */

#define KE2100_VAL_MAX_TIME_IMMEDIATE                       0
#define KE2100_VAL_MAX_TIME_INFINITE                        -1

/*- Defined values for */

#define KE2100_VAL_AUTO_DELAY_OFF                           -2
#define KE2100_VAL_AUTO_DELAY_ON                            -1

/*- Defined values for 
	attribute KE2100_ATTR_CONTROL_MODE */

#define KE2100_VAL_CONTROL_MODE_LOCAL                       0
#define KE2100_VAL_CONTROL_MODE_REMOTE                      1

/*- Defined values for 
	attribute KE2100_ATTR_FUNCTION2 */

#define KE2100_VAL_MATH_FUNCTION_PERCENT                    0
#define KE2100_VAL_MATH_FUNCTION_AVERAGE                    1
#define KE2100_VAL_MATH_FUNCTION_NULL                       2
#define KE2100_VAL_MATH_FUNCTION_LIMIT                      3
#define KE2100_VAL_MATH_FUNCTIONMXB                         4
#define KE2100_VAL_MATH_FUNCTIONDB                          5
#define KE2100_VAL_MATH_FUNCTIONDBM                         6

/*- Defined values for 
	parameter Type in function Ke2100_SetMinMaxLimit
	parameter Type in function Ke2100_GetLimit
	parameter Type in function Ke2100_SetLimit */

#define KE2100_VAL_MATH_LIMIT_TYPE_LOWER                    0
#define KE2100_VAL_MATH_LIMIT_TYPE_UPPER                    1

/*- Defined values for 
	parameter Type in function Ke2100_SetMinMaxMXB
	parameter Type in function Ke2100_GetMxb
	parameter Type in function Ke2100_SetMxb */

#define KE2100_VAL_MATHMXB_TYPEMM                           0
#define KE2100_VAL_MATHMXB_TYPEMB                           1

/*- Defined values for 
	parameter MinMaxType in function Ke2100_SetMinMaxPercentTarget
	parameter MinMaxType in function Ke2100_SetMinMaxNullOffset
	parameter MinMaxType in function Ke2100_SetMinMaxLimit
	parameter MinMaxType in function Ke2100_SetMinMaxMXB
	parameter MinMaxType in function Ke2100_SetMinMaxDBRelative
	parameter MinMaxType in function Ke2100_SetMinMaxDBMReference
	parameter MinMaxType in function Ke2100_SetMinMaxRange
	parameter MinMaxType in function Ke2100_SetMinMaxResolution
	parameter MinMaxType in function Ke2100_SetMinMaxNPLCycles
	parameter MinMaxType in function Ke2100_SetMinMaxAperture
	parameter MinMaxType in function Ke2100_SetMinMaxDigitalFilterCount
	parameter MinMaxType in function Ke2100_SetMinMaxSimulatedReferenceJunction
	parameter MinMaxType in function Ke2100_SetMinMaxUserDefinedConstants
	parameter MinMaxType in function Ke2100_SetMinMaxSPRTDConstants
	parameter MinMaxType in function Ke2100_SetMinMaxSampleCount
	parameter MinMaxType in function Ke2100_SetMinMaxDelay */

#define KE2100_VAL_MIN_MAX_MIN                              0
#define KE2100_VAL_MIN_MAX_MAX                              1

/*- Defined values for 
	attribute KE2100_ATTR_INPUT_TERMINAL_TYPE */

#define KE2100_VAL_INPUT_TERMINAL_TYPE_FRONT                0
#define KE2100_VAL_INPUT_TERMINAL_TYPE_REAR                 1

/*- Defined values for 
	parameter Function in function Ke2100_GetRange2
	parameter Function in function Ke2100_SetRange2
	parameter Function in function Ke2100_SetMinMaxRange
	parameter Function in function Ke2100_GetAutoRangeState
	parameter Function in function Ke2100_SetAutoRangeState
	parameter Function in function Ke2100_GetResolution
	parameter Function in function Ke2100_SetResolution
	parameter Function in function Ke2100_SetMinMaxResolution */

#define KE2100_VAL_CONFIGURATION_FUNCTION2DC_VOLTS          0
#define KE2100_VAL_CONFIGURATION_FUNCTION2DC_RATIO          1
#define KE2100_VAL_CONFIGURATION_FUNCTION2AC_VOLTS          2
#define KE2100_VAL_CONFIGURATION_FUNCTION2DC_CURRENT        3
#define KE2100_VAL_CONFIGURATION_FUNCTION2AC_CURRENT        4
#define KE2100_VAL_CONFIGURATION_FUNCTION22_WIRE_RESISTANCE 5
#define KE2100_VAL_CONFIGURATION_FUNCTION24_WIRE_RESISTANCE 6
#define KE2100_VAL_CONFIGURATION_FUNCTION2_FREQUENCY        7
#define KE2100_VAL_CONFIGURATION_FUNCTION2_PERIOD           8

/*- Defined values for 
	parameter Function in function Ke2100_GetNplCycles
	parameter Function in function Ke2100_SetNplCycles
	parameter Function in function Ke2100_SetMinMaxNPLCycles */

#define KE2100_VAL_NPL_CYCLE_FUNCTIONDC_VOLTS               0
#define KE2100_VAL_NPL_CYCLE_FUNCTIONDC_CURRENT             1
#define KE2100_VAL_NPL_CYCLE_FUNCTION2_WIRE_RESISTANCE      2
#define KE2100_VAL_NPL_CYCLE_FUNCTION4_WIRE_RESISTANCE      3

/*- Defined values for 
	parameter Function in function Ke2100_GetAperture
	parameter Function in function Ke2100_SetAperture
	parameter Function in function Ke2100_SetMinMaxAperture */

#define KE2100_VAL_FREQ_PERIOD_FUNCTION_FREQUENCY           0
#define KE2100_VAL_FREQ_PERIOD_FUNCTION_PERIOD              1

/*- Defined values for 
	attribute KE2100_ATTR_AUTO_ZERO2
	attribute KE2100_ATTR_AUTO_ZERO3 */

#define KE2100_VAL_AUTO_ZERO_OFF2                           0
#define KE2100_VAL_AUTO_ZERO_ON2                            1
#define KE2100_VAL_AUTO_ZERO_ONCE2                          2

/*- Defined values for 
	attribute KE2100_ATTR_AUTO_GAIN */

#define KE2100_VAL_AUTO_GAIN_OFF                            0
#define KE2100_VAL_AUTO_GAIN_ON                             1
#define KE2100_VAL_AUTO_GAIN_ONCE                           2

/*- Defined values for 
	attribute KE2100_ATTR_DIGITAL_FILTER_OPERATION_MODE */

#define KE2100_VAL_DIGITAL_FILTER_OPERATION_MODE_MOVING_AVERAGE    0
#define KE2100_VAL_DIGITAL_FILTER_OPERATION_MODE_REPEATING_AVERAGE 1

/*- Defined values for 
	attribute KE2100_ATTR_UNIT */

#define KE2100_VAL_TEMPERATURE_UNIT_CELSIUS                 0
#define KE2100_VAL_TEMPERATURE_UNIT_FAHRENHEIT              1
#define KE2100_VAL_TEMPERATURE_UNIT_KELVIN                  2

/*- Defined values for 
	attribute KE2100_ATTR_SENSOR_TYPE */

#define KE2100_VAL_TERMO_COUPLE_SENSOR_TYPEE                4
#define KE2100_VAL_TERMO_COUPLE_SENSOR_TYPEJ                6
#define KE2100_VAL_TERMO_COUPLE_SENSOR_TYPEK                7
#define KE2100_VAL_TERMO_COUPLE_SENSOR_TYPEN                8
#define KE2100_VAL_TERMO_COUPLE_SENSOR_TYPER                9
#define KE2100_VAL_TERMO_COUPLE_SENSOR_TYPES                10
#define KE2100_VAL_TERMO_COUPLE_SENSOR_TYPET                11

/*- Defined values for 
	attribute KE2100_ATTR_REFERENCE_JUNCTION_TYPE */

#define KE2100_VAL_REFERENCE_JUNCTION_TYPE_REAL             0
#define KE2100_VAL_REFERENCE_JUNCTION_TYPE_SIMULATED        1

/*- Defined values for 
	attribute KE2100_ATTR_TYPE */

#define KE2100_VAL_TEMPERATURERTD_TYPEPT100                 0
#define KE2100_VAL_TEMPERATURERTD_TYPED100                  1
#define KE2100_VAL_TEMPERATURERTD_TYPEF100                  2
#define KE2100_VAL_TEMPERATURERTD_TYPEPT385                 3
#define KE2100_VAL_TEMPERATURERTD_TYPEPT3916                4
#define KE2100_VAL_TEMPERATURERTD_TYPEUSER                  5
#define KE2100_VAL_TEMPERATURERTD_TYPESPRTD                 6
#define KE2100_VAL_TEMPERATURERTD_TYPENTCT                  7

/*- Defined values for 
	parameter Constant in function Ke2100_ConfigureUserDefinedType
	parameter Constant in function Ke2100_SetMinMaxUserDefinedConstants
	parameter Constant in function Ke2100_QueryConstant */

#define KE2100_VAL_TEMPERATURERTD_CONSTANTSR_ZERO           0
#define KE2100_VAL_TEMPERATURERTD_CONSTANTS_ALPHA           1
#define KE2100_VAL_TEMPERATURERTD_CONSTANTS_BETA            2
#define KE2100_VAL_TEMPERATURERTD_CONSTANTS_DELTA           3

/*- Defined values for 
	parameter Constant in function Ke2100_ConfigureSPRTDType
	parameter Constant in function Ke2100_SetMinMaxSPRTDConstants
	parameter Constant in function Ke2100_QuerySPRTDConstant */

#define KE2100_VAL_TEMPERATURESPRTD_CONSTANTSR_ZERO         0
#define KE2100_VAL_TEMPERATURESPRTD_CONSTANTSA4             1
#define KE2100_VAL_TEMPERATURESPRTD_CONSTANTSB4             2
#define KE2100_VAL_TEMPERATURESPRTD_CONSTANTSA              3
#define KE2100_VAL_TEMPERATURESPRTD_CONSTANTSB              4
#define KE2100_VAL_TEMPERATURESPRTD_CONSTANTSC              5
#define KE2100_VAL_TEMPERATURESPRTD_CONSTANTSD              6

/*- Defined values for 
	parameter TriggerCountType in function Ke2100_SetCountEnum */

#define KE2100_VAL_TRIGGER_COUNT_TYPE_MIN                   0
#define KE2100_VAL_TRIGGER_COUNT_TYPE_MAX                   1
#define KE2100_VAL_TRIGGER_COUNT_TYPE_INFINITE              2

/*- Defined values for 
	attribute KE2100_ATTR_FUNCTION4
	parameter Function in function Ke2100_Measure
	parameter Function in function Ke2100_Configure */

#define KE2100_VAL_FUNCTIONDC_RATIO                         0
#define KE2100_VAL_FUNCTIONDC_VOLTS                         1
#define KE2100_VAL_FUNCTIONAC_VOLTS                         2
#define KE2100_VAL_FUNCTIONDC_CURRENT                       3
#define KE2100_VAL_FUNCTIONAC_CURRENT                       4
#define KE2100_VAL_FUNCTION2_WIRE_RES                       5
#define KE2100_VAL_FUNCTION4_WIRE_RES                       6
#define KE2100_VAL_FUNCTION_FREQUENCY                       7
#define KE2100_VAL_FUNCTION_PERIOD                          8
#define KE2100_VAL_FUNCTION_TEMPERATURE                     9
#define KE2100_VAL_FUNCTION_THERMO_COUPLE                   10
#define KE2100_VAL_FUNCTION_CONTINUITY                      11
#define KE2100_VAL_FUNCTION_DIODE                           12

/*- Defined values for 
	attribute KE2100_ATTR_APERTURE_TIME_UNITS2 */

#define KE2100_VAL_APERTURE_SECONDS                         0
#define KE2100_VAL_APERTURE_POWER_LINE_CYCLES               1

/*- Defined values for 
	attribute KE2100_ATTR_TYPE2
	parameter Type in function Ke2100_Configure3 */

#define KE2100_VAL_THERMOCOUPLE_TYPEE                       4
#define KE2100_VAL_THERMOCOUPLE_TYPEJ                       6
#define KE2100_VAL_THERMOCOUPLE_TYPEK                       7
#define KE2100_VAL_THERMOCOUPLE_TYPEN                       8
#define KE2100_VAL_THERMOCOUPLE_TYPER                       9
#define KE2100_VAL_THERMOCOUPLE_TYPES                       10
#define KE2100_VAL_THERMOCOUPLE_TYPET                       11

/*- Defined values for 
	attribute KE2100_ATTR_REF_JUNCTION_TYPE
	parameter RefJunctionType in function Ke2100_Configure3 */

#define KE2100_VAL_REF_JUNCTION_TYPE_INTERNAL               1
#define KE2100_VAL_REF_JUNCTION_TYPE_FIXED                  2

/*- Defined values for 
	attribute KE2100_ATTR_TRANSDUCER_TYPE */

#define KE2100_VAL_TRANSDUCER_TYPE_THERMOCOUPLE             1
#define KE2100_VAL_TRANSDUCER_TYPE_THERMISTOR               2
#define KE2100_VAL_TRANSDUCER_TYPE2_WIRE_RTD                3
#define KE2100_VAL_TRANSDUCER_TYPE4_WIRE_RTD                4

/*- Defined values for 
	attribute KE2100_ATTR_SOURCE
	parameter TriggerSource in function Ke2100_Configure4 */

#define KE2100_VAL_TRIGGER_SOURCE_IMMEDIATE                 1
#define KE2100_VAL_TRIGGER_SOURCE_EXTERNAL                  2
#define KE2100_VAL_TRIGGER_SOURCE_BUS                       0

/*- Defined values for 
	attribute KE2100_ATTR_SAMPLE_TRIGGER2
	parameter SampleTrigger in function Ke2100_Configure5 */

#define KE2100_VAL_SAMPLE_TRIGGER_IMMEDIATE                 1
#define KE2100_VAL_SAMPLE_TRIGGER_EXTERNAL                  2
#define KE2100_VAL_SAMPLE_TRIGGER_SW_TRIG_FUNC              3
#define KE2100_VAL_SAMPLE_TRIGGERTTL0                       111
#define KE2100_VAL_SAMPLE_TRIGGERTTL1                       112
#define KE2100_VAL_SAMPLE_TRIGGERTTL2                       113
#define KE2100_VAL_SAMPLE_TRIGGERTTL3                       114
#define KE2100_VAL_SAMPLE_TRIGGERTTL4                       115
#define KE2100_VAL_SAMPLE_TRIGGERTTL5                       116
#define KE2100_VAL_SAMPLE_TRIGGERTTL6                       117
#define KE2100_VAL_SAMPLE_TRIGGERTTL7                       118
#define KE2100_VAL_SAMPLE_TRIGGERECL0                       119
#define KE2100_VAL_SAMPLE_TRIGGERECL1                       120
#define KE2100_VAL_SAMPLE_TRIGGERPXI_STAR                   131
#define KE2100_VAL_SAMPLE_TRIGGERRTSI0                      140
#define KE2100_VAL_SAMPLE_TRIGGERRTSI1                      141
#define KE2100_VAL_SAMPLE_TRIGGERRTSI2                      142
#define KE2100_VAL_SAMPLE_TRIGGERRTSI3                      143
#define KE2100_VAL_SAMPLE_TRIGGERRTSI4                      144
#define KE2100_VAL_SAMPLE_TRIGGERRTSI5                      145
#define KE2100_VAL_SAMPLE_TRIGGERRTSI6                      146
#define KE2100_VAL_SAMPLE_TRIGGER_INTERVAL                  10

/*- Defined values for 
	attribute KE2100_ATTR_MEASUREMENT_COMPLETE */

#define KE2100_VAL_MEAS_COMPLETE_DEST_NONE                  -1
#define KE2100_VAL_MEAS_COMPLETE_DEST_EXTERNAL              2
#define KE2100_VAL_MEAS_COMPLETE_DESTTTL0                   111
#define KE2100_VAL_MEAS_COMPLETE_DESTTTL1                   112
#define KE2100_VAL_MEAS_COMPLETE_DESTTTL2                   113
#define KE2100_VAL_MEAS_COMPLETE_DESTTTL3                   114
#define KE2100_VAL_MEAS_COMPLETE_DESTTTL4                   115
#define KE2100_VAL_MEAS_COMPLETE_DESTTTL5                   116
#define KE2100_VAL_MEAS_COMPLETE_DESTTTL6                   117
#define KE2100_VAL_MEAS_COMPLETE_DESTTTL7                   118
#define KE2100_VAL_MEAS_COMPLETE_DESTECL0                   119
#define KE2100_VAL_MEAS_COMPLETE_DESTECL1                   120
#define KE2100_VAL_MEAS_COMPLETE_DESTPXI_STAR               131
#define KE2100_VAL_MEAS_COMPLETE_DESTRTSI0                  140
#define KE2100_VAL_MEAS_COMPLETE_DESTRTSI1                  141
#define KE2100_VAL_MEAS_COMPLETE_DESTRTSI2                  142
#define KE2100_VAL_MEAS_COMPLETE_DESTRTSI3                  143
#define KE2100_VAL_MEAS_COMPLETE_DESTRTSI4                  144
#define KE2100_VAL_MEAS_COMPLETE_DESTRTSI5                  145
#define KE2100_VAL_MEAS_COMPLETE_DESTRTSI6                  146

/*- Defined values for 
	attribute KE2100_ATTR_SLOPE */

#define KE2100_VAL_TRIGGER_SLOPE_POSITIVE                   0
#define KE2100_VAL_TRIGGER_SLOPE_NEGATIVE                   1


/**************************************************************************** 
 *---------------- Instrument Driver Function Declarations -----------------* 
 ****************************************************************************/

/*- Ke2100 */

ViStatus _VI_FUNC Ke2100_init ( ViRsrc ResourceName, ViBoolean IdQuery, ViBoolean Reset, ViSession* Vi );
ViStatus _VI_FUNC Ke2100_close ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_InitWithOptions ( ViRsrc ResourceName, ViBoolean IdQuery, ViBoolean Reset, ViConstString OptionsString, ViSession* Vi );

/*- Utility */

ViStatus _VI_FUNC Ke2100_revision_query ( ViSession Vi, ViChar DriverRev[], ViChar InstrRev[] );
ViStatus _VI_FUNC Ke2100_error_message ( ViSession Vi, ViStatus ErrorCode, ViChar ErrorMessage[] );
ViStatus _VI_FUNC Ke2100_GetError ( ViSession Vi, ViStatus* ErrorCode, ViInt32 ErrorDescriptionBufferSize, ViChar ErrorDescription[] );
ViStatus _VI_FUNC Ke2100_ClearError ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_ClearInterchangeWarnings ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_GetNextCoercionRecord ( ViSession Vi, ViInt32 CoercionRecordBufferSize, ViChar CoercionRecord[] );
ViStatus _VI_FUNC Ke2100_GetNextInterchangeWarning ( ViSession Vi, ViInt32 InterchangeWarningBufferSize, ViChar InterchangeWarning[] );
ViStatus _VI_FUNC Ke2100_InvalidateAllAttributes ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_ResetInterchangeCheck ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_Disable ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_error_query ( ViSession Vi, ViInt32* ErrorCode, ViChar ErrorMessage[] );
ViStatus _VI_FUNC Ke2100_LockSession ( ViSession Vi, ViBoolean* CallerHasLock );
ViStatus _VI_FUNC Ke2100_reset ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_ResetWithDefaults ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_self_test ( ViSession Vi, ViInt16* TestResult, ViChar TestMessage[] );
ViStatus _VI_FUNC Ke2100_UnlockSession ( ViSession Vi, ViBoolean* CallerHasLock );

/*- Attribute Accessors */

ViStatus _VI_FUNC Ke2100_GetAttributeViInt32 ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViInt32* AttributeValue );
ViStatus _VI_FUNC Ke2100_GetAttributeViReal64 ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViReal64* AttributeValue );
ViStatus _VI_FUNC Ke2100_GetAttributeViBoolean ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViBoolean* AttributeValue );
ViStatus _VI_FUNC Ke2100_GetAttributeViSession ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViSession* AttributeValue );
ViStatus _VI_FUNC Ke2100_GetAttributeViString ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViInt32 AttributeValueBufferSize, ViChar AttributeValue[] );
ViStatus _VI_FUNC Ke2100_SetAttributeViInt32 ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViInt32 AttributeValue );
ViStatus _VI_FUNC Ke2100_SetAttributeViReal64 ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViReal64 AttributeValue );
ViStatus _VI_FUNC Ke2100_SetAttributeViBoolean ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViBoolean AttributeValue );
ViStatus _VI_FUNC Ke2100_SetAttributeViSession ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViSession AttributeValue );
ViStatus _VI_FUNC Ke2100_SetAttributeViString ( ViSession Vi, ViConstString RepCapIdentifier, ViAttr AttributeID, ViConstString AttributeValue );

/*- Configuration */

ViStatus _VI_FUNC Ke2100_ConfigureMeasurement ( ViSession Vi, ViInt32 Function, ViReal64 Range, ViReal64 Resolution );

/*- Specific Measurements */

ViStatus _VI_FUNC Ke2100_ConfigureFrequencyVoltageRange ( ViSession Vi, ViReal64 FrequencyVoltageRange );
ViStatus _VI_FUNC Ke2100_ConfigureACBandwidth ( ViSession Vi, ViReal64 MinFreq, ViReal64 MaxFreq );

/*- Temperature */

ViStatus _VI_FUNC Ke2100_ConfigureTransducerType ( ViSession Vi, ViInt32 TransducerType );
ViStatus _VI_FUNC Ke2100_ConfigureFixedRefJunction ( ViSession Vi, ViReal64 FixedRefJunction );
ViStatus _VI_FUNC Ke2100_ConfigureThermistor ( ViSession Vi, ViReal64 Resistance );
ViStatus _VI_FUNC Ke2100_ConfigureRTD ( ViSession Vi, ViReal64 Alpha, ViReal64 Resistance );
ViStatus _VI_FUNC Ke2100_ConfigureThermocouple ( ViSession Vi, ViInt32 ThermocoupleType, ViInt32 RefJunctionType );

/*- Trigger */

ViStatus _VI_FUNC Ke2100_ConfigureTriggerSlope ( ViSession Vi, ViInt32 Polarity );
ViStatus _VI_FUNC Ke2100_ConfigureTrigger ( ViSession Vi, ViInt32 TriggerSource, ViReal64 TriggerDelay );

/*- Configuration Information */

ViStatus _VI_FUNC Ke2100_GetApertureTimeInfo ( ViSession Vi, ViReal64* ApertureTime, ViInt32* ApertureTimeUnits );
ViStatus _VI_FUNC Ke2100_GetAutoRangeValue ( ViSession Vi, ViReal64* AutoRangeValue );

/*- Measurement Operation Options */

ViStatus _VI_FUNC Ke2100_ConfigureAutoZeroMode ( ViSession Vi, ViInt32 AutoZeroMode );
ViStatus _VI_FUNC Ke2100_ConfigurePowerLineFrequency ( ViSession Vi, ViReal64 PowerLineFreq );

/*- MultiPoint */

ViStatus _VI_FUNC Ke2100_ConfigureMeasCompleteDest ( ViSession Vi, ViInt32 MeasCompleteDest );
ViStatus _VI_FUNC Ke2100_ConfigureMultiPoint ( ViSession Vi, ViInt32 TriggerCount, ViInt32 SampleCount, ViInt32 SampleTrigger, ViReal64 SampleInterval );

/*- Measurement */

ViStatus _VI_FUNC Ke2100_Read ( ViSession Vi, ViInt32 MaxTimeMilliseconds, ViReal64* Reading );
ViStatus _VI_FUNC Ke2100_ReadMultiPoint ( ViSession Vi, ViInt32 MaxTimeMilliseconds, ViInt32 ArraySize, ViReal64 ReadingArray[], ViInt32* ActualPts );

/*- Low Level Measurement */

ViStatus _VI_FUNC Ke2100_Abort ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_Fetch ( ViSession Vi, ViInt32 MaxTimeMilliseconds, ViReal64* Reading );
ViStatus _VI_FUNC Ke2100_FetchMultiPoint ( ViSession Vi, ViInt32 MaxTimeMilliseconds, ViInt32 ArraySize, ViReal64 ReadingArray[], ViInt32* ActualPts );
ViStatus _VI_FUNC Ke2100_Initiate ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_IsOverRange ( ViSession Vi, ViReal64 MeasurementValue, ViBoolean* IsOverRange );
ViStatus _VI_FUNC Ke2100_SendSoftwareTrigger ( ViSession Vi );

/*- Instrument Specific */

ViStatus _VI_FUNC Ke2100_QueryConfiguration ( ViSession Vi, ViInt32 ValBufferSize, ViChar Val[] );
ViStatus _VI_FUNC Ke2100_Measure ( ViSession Vi, ViInt32 Function, ViReal64 Range, ViReal64 Resolution, ViReal64* Val );
ViStatus _VI_FUNC Ke2100_Configure ( ViSession Vi, ViInt32 Function, ViReal64 Range, ViReal64 Resolution );
ViStatus _VI_FUNC Ke2100_GetRange ( ViSession Vi, ViReal64* Range );
ViStatus _VI_FUNC Ke2100_SetRange ( ViSession Vi, ViReal64 Range );
ViStatus _VI_FUNC Ke2100_GetResolution2 ( ViSession Vi, ViReal64* Resolution );
ViStatus _VI_FUNC Ke2100_SetResolution2 ( ViSession Vi, ViReal64 Resolution );

/*- Status */

ViStatus _VI_FUNC Ke2100_ResetStatusRegister ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_ClearStatusByte ( ViSession Vi );

/*- System */

ViStatus _VI_FUNC Ke2100_DisplayText ( ViSession Vi, ViConstString Text );
ViStatus _VI_FUNC Ke2100_QueryDisplayText ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_ClearDisplayText ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_Beep ( ViSession Vi );

/*- Math */

ViStatus _VI_FUNC Ke2100_SetMinMaxPercentTarget ( ViSession Vi, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_SetMinMaxNullOffset ( ViSession Vi, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_SetMinMaxLimit ( ViSession Vi, ViInt32 Type, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_SetMinMaxMXB ( ViSession Vi, ViInt32 Type, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_SetMinMaxDBRelative ( ViSession Vi, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_SetMinMaxDBMReference ( ViSession Vi, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_GetLimit ( ViSession Vi, ViInt32 Type, ViReal64* Limit );
ViStatus _VI_FUNC Ke2100_SetLimit ( ViSession Vi, ViInt32 Type, ViReal64 Limit );
ViStatus _VI_FUNC Ke2100_GetMxb ( ViSession Vi, ViInt32 Type, ViReal64* MXB );
ViStatus _VI_FUNC Ke2100_SetMxb ( ViSession Vi, ViInt32 Type, ViReal64 MXB );

/*- Channels */

ViStatus _VI_FUNC Ke2100_OpenAll ( ViSession Vi );

/*- ScannerCard */

ViStatus _VI_FUNC Ke2100_GetFunction3 ( ViSession Vi, ViInt32 Channel, ViInt32 FunctionBufferSize, ViChar Function[] );
ViStatus _VI_FUNC Ke2100_SetFunction3 ( ViSession Vi, ViInt32 Channel, ViConstString Function );
ViStatus _VI_FUNC Ke2100_Scan ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_Step ( ViSession Vi );

/*- Measurement */

ViStatus _VI_FUNC Ke2100_Abort2 ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_Fetch2 ( ViSession Vi, ViInt32 MaxTimeMilliseconds, ViReal64* Val );
ViStatus _VI_FUNC Ke2100_FetchMultiPoint2 ( ViSession Vi, ViInt32 MaxTimeMilliseconds, ViInt32 ValBufferSize, ViReal64 Val[], ViInt32* ValActualSize );
ViStatus _VI_FUNC Ke2100_Initiate2 ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_IsOverRange2 ( ViSession Vi, ViReal64 MeasurementValue, ViBoolean* Val );
ViStatus _VI_FUNC Ke2100_Read2 ( ViSession Vi, ViInt32 MaxTimeMilliseconds, ViReal64* Val );
ViStatus _VI_FUNC Ke2100_ReadMultiPoint2 ( ViSession Vi, ViInt32 MaxTimeMilliseconds, ViInt32 ValBufferSize, ViReal64 Val[], ViInt32* ValActualSize );
ViStatus _VI_FUNC Ke2100_SendSoftwareTrigger2 ( ViSession Vi );
ViStatus _VI_FUNC Ke2100_QueryDataPoints ( ViSession Vi, ViInt32* Val );

/*- Configuration */

ViStatus _VI_FUNC Ke2100_GetRange2 ( ViSession Vi, ViInt32 Function, ViReal64* Range );
ViStatus _VI_FUNC Ke2100_SetRange2 ( ViSession Vi, ViInt32 Function, ViReal64 Range );
ViStatus _VI_FUNC Ke2100_SetMinMaxRange ( ViSession Vi, ViInt32 Function, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_GetAutoRangeState ( ViSession Vi, ViInt32 Function, ViBoolean* AutoRangeState );
ViStatus _VI_FUNC Ke2100_SetAutoRangeState ( ViSession Vi, ViInt32 Function, ViBoolean AutoRangeState );
ViStatus _VI_FUNC Ke2100_GetResolution ( ViSession Vi, ViInt32 Function, ViReal64* Resolution );
ViStatus _VI_FUNC Ke2100_SetResolution ( ViSession Vi, ViInt32 Function, ViReal64 Resolution );
ViStatus _VI_FUNC Ke2100_SetMinMaxResolution ( ViSession Vi, ViInt32 Function, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_GetNplCycles ( ViSession Vi, ViInt32 Function, ViReal64* NPLCycles );
ViStatus _VI_FUNC Ke2100_SetNplCycles ( ViSession Vi, ViInt32 Function, ViReal64 NPLCycles );
ViStatus _VI_FUNC Ke2100_SetMinMaxNPLCycles ( ViSession Vi, ViInt32 Function, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_GetAperture ( ViSession Vi, ViInt32 Function, ViReal64* Aperture );
ViStatus _VI_FUNC Ke2100_SetAperture ( ViSession Vi, ViInt32 Function, ViReal64 Aperture );
ViStatus _VI_FUNC Ke2100_SetMinMaxAperture ( ViSession Vi, ViInt32 Function, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_SetMinMaxDigitalFilterCount ( ViSession Vi, ViInt32 MinMaxType );

/*- TCouple */

ViStatus _VI_FUNC Ke2100_SetMinMaxSimulatedReferenceJunction ( ViSession Vi, ViInt32 MinMaxType );

/*- RTD */

ViStatus _VI_FUNC Ke2100_ConfigureUserDefinedType ( ViSession Vi, ViInt32 Constant, ViReal64 Value );
ViStatus _VI_FUNC Ke2100_SetMinMaxUserDefinedConstants ( ViSession Vi, ViInt32 Constant, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_QueryConstant ( ViSession Vi, ViInt32 Constant, ViReal64* Value );
ViStatus _VI_FUNC Ke2100_ConfigureSPRTDType ( ViSession Vi, ViInt32 Constant, ViReal64 Value );
ViStatus _VI_FUNC Ke2100_SetMinMaxSPRTDConstants ( ViSession Vi, ViInt32 Constant, ViInt32 MinMaxType );
ViStatus _VI_FUNC Ke2100_QuerySPRTDConstant ( ViSession Vi, ViInt32 Constant, ViReal64* Value );

/*- MultiPoint */

ViStatus _VI_FUNC Ke2100_SetCountEnum ( ViSession Vi, ViInt32 TriggerCountType );
ViStatus _VI_FUNC Ke2100_SetMinMaxSampleCount ( ViSession Vi, ViInt32 MinMaxType );

/*- AC */

ViStatus _VI_FUNC Ke2100_ConfigureBandwidth ( ViSession Vi, ViReal64 MinFreq, ViReal64 MaxFreq );

/*- RTD */

ViStatus _VI_FUNC Ke2100_Configure2 ( ViSession Vi, ViReal64 Alpha, ViReal64 Resistance );

/*- Thermocouple */

ViStatus _VI_FUNC Ke2100_Configure3 ( ViSession Vi, ViInt32 Type, ViInt32 RefJunctionType );

/*- Trigger */

ViStatus _VI_FUNC Ke2100_Configure4 ( ViSession Vi, ViInt32 TriggerSource, ViReal64 TriggerDelay );
ViStatus _VI_FUNC Ke2100_SetMinMaxDelay ( ViSession Vi, ViInt32 MinMaxType );

/*- MultiPoint */

ViStatus _VI_FUNC Ke2100_Configure5 ( ViSession Vi, ViInt32 TriggerCount, ViInt32 SampleCount, ViInt32 SampleTrigger, ViReal64 SampleInterval );

/*- InstrumentIO */

ViStatus _VI_FUNC Ke2100_Query ( ViSession Vi, ViConstString Command, ViInt32 ValBufferSize, ViChar Val[] );
ViStatus _VI_FUNC Ke2100_ReadPartialString ( ViSession Vi, ViInt32 Length, ViInt32 ValBufferSize, ViChar Val[] );
ViStatus _VI_FUNC Ke2100_ReadString ( ViSession Vi, ViInt32 ValBufferSize, ViChar Val[] );
ViStatus _VI_FUNC Ke2100_WriteString ( ViSession Vi, ViConstString Command );


/**************************************************************************** 
 *----------------- Instrument Error And Completion Codes ------------------* 
 ****************************************************************************/
#ifndef _IVIC_ERROR_BASE_DEFINES_
#define _IVIC_ERROR_BASE_DEFINES_

#define IVIC_WARN_BASE                           (0x3FFA0000L)
#define IVIC_CROSS_CLASS_WARN_BASE               (IVIC_WARN_BASE + 0x1000)
#define IVIC_CLASS_WARN_BASE                     (IVIC_WARN_BASE + 0x2000)
#define IVIC_SPECIFIC_WARN_BASE                  (IVIC_WARN_BASE + 0x4000)

#define IVIC_ERROR_BASE                          (0xBFFA0000L)
#define IVIC_CROSS_CLASS_ERROR_BASE              (IVIC_ERROR_BASE + 0x1000)
#define IVIC_CLASS_ERROR_BASE                    (IVIC_ERROR_BASE + 0x2000)
#define IVIC_SPECIFIC_ERROR_BASE                 (IVIC_ERROR_BASE + 0x4000)
#define IVIC_LXISYNC_ERROR_BASE                  (IVIC_ERROR_BASE + 0x2000)

#define IVIC_ERROR_INVALID_ATTRIBUTE             (IVIC_ERROR_BASE + 0x000C)
#define IVIC_ERROR_TYPES_DO_NOT_MATCH            (IVIC_ERROR_BASE + 0x0015)
#define IVIC_ERROR_IVI_ATTR_NOT_WRITABLE         (IVIC_ERROR_BASE + 0x000D)
#define IVIC_ERROR_IVI_ATTR_NOT_READABLE         (IVIC_ERROR_BASE + 0x000E)
#define IVIC_ERROR_INVALID_SESSION_HANDLE        (IVIC_ERROR_BASE + 0x1190)

#endif


#define KE2100_ERROR_CANNOT_RECOVER                         (IVIC_ERROR_BASE + 0x0000)
#define KE2100_ERROR_INSTRUMENT_STATUS                      (IVIC_ERROR_BASE + 0x0001)
#define KE2100_ERROR_CANNOT_OPEN_FILE                       (IVIC_ERROR_BASE + 0x0002)
#define KE2100_ERROR_READING_FILE                           (IVIC_ERROR_BASE + 0x0003)
#define KE2100_ERROR_WRITING_FILE                           (IVIC_ERROR_BASE + 0x0004)
#define KE2100_ERROR_INVALID_PATHNAME                       (IVIC_ERROR_BASE + 0x000B)
#define KE2100_ERROR_INVALID_VALUE                          (IVIC_ERROR_BASE + 0x0010)
#define KE2100_ERROR_FUNCTION_NOT_SUPPORTED                 (IVIC_ERROR_BASE + 0x0011)
#define KE2100_ERROR_ATTRIBUTE_NOT_SUPPORTED                (IVIC_ERROR_BASE + 0x0012)
#define KE2100_ERROR_VALUE_NOT_SUPPORTED                    (IVIC_ERROR_BASE + 0x0013)
#define KE2100_ERROR_NOT_INITIALIZED                        (IVIC_ERROR_BASE + 0x001D)
#define KE2100_ERROR_UNKNOWN_CHANNEL_NAME                   (IVIC_ERROR_BASE + 0x0020)
#define KE2100_ERROR_TOO_MANY_OPEN_FILES                    (IVIC_ERROR_BASE + 0x0023)
#define KE2100_ERROR_CHANNEL_NAME_REQUIRED                  (IVIC_ERROR_BASE + 0x0044)
#define KE2100_ERROR_MISSING_OPTION_NAME                    (IVIC_ERROR_BASE + 0x0049)
#define KE2100_ERROR_MISSING_OPTION_VALUE                   (IVIC_ERROR_BASE + 0x004A)
#define KE2100_ERROR_BAD_OPTION_NAME                        (IVIC_ERROR_BASE + 0x004B)
#define KE2100_ERROR_BAD_OPTION_VALUE                       (IVIC_ERROR_BASE + 0x004C)
#define KE2100_ERROR_OUT_OF_MEMORY                          (IVIC_ERROR_BASE + 0x0056)
#define KE2100_ERROR_OPERATION_PENDING                      (IVIC_ERROR_BASE + 0x0057)
#define KE2100_ERROR_NULL_POINTER                           (IVIC_ERROR_BASE + 0x0058)
#define KE2100_ERROR_UNEXPECTED_RESPONSE                    (IVIC_ERROR_BASE + 0x0059)
#define KE2100_ERROR_FILE_NOT_FOUND                         (IVIC_ERROR_BASE + 0x005B)
#define KE2100_ERROR_INVALID_FILE_FORMAT                    (IVIC_ERROR_BASE + 0x005C)
#define KE2100_ERROR_STATUS_NOT_AVAILABLE                   (IVIC_ERROR_BASE + 0x005D)
#define KE2100_ERROR_ID_QUERY_FAILED                        (IVIC_ERROR_BASE + 0x005E)
#define KE2100_ERROR_RESET_FAILED                           (IVIC_ERROR_BASE + 0x005F)
#define KE2100_ERROR_RESOURCE_UNKNOWN                       (IVIC_ERROR_BASE + 0x0060)
#define KE2100_ERROR_ALREADY_INITIALIZED                    (IVIC_ERROR_BASE + 0x0061)
#define KE2100_ERROR_CANNOT_CHANGE_SIMULATION_STATE         (IVIC_ERROR_BASE + 0x0062)
#define KE2100_ERROR_INVALID_NUMBER_OF_LEVELS_IN_SELECTOR   (IVIC_ERROR_BASE + 0x0063)
#define KE2100_ERROR_INVALID_RANGE_IN_SELECTOR              (IVIC_ERROR_BASE + 0x0064)
#define KE2100_ERROR_UNKOWN_NAME_IN_SELECTOR                (IVIC_ERROR_BASE + 0x0065)
#define KE2100_ERROR_BADLY_FORMED_SELECTOR                  (IVIC_ERROR_BASE + 0x0066)
#define KE2100_ERROR_UNKNOWN_PHYSICAL_IDENTIFIER            (IVIC_ERROR_BASE + 0x0067)



#define KE2100_SUCCESS                                      0
#define KE2100_WARN_NSUP_ID_QUERY                           (IVIC_WARN_BASE + 0x0065)
#define KE2100_WARN_NSUP_RESET                              (IVIC_WARN_BASE + 0x0066)
#define KE2100_WARN_NSUP_SELF_TEST                          (IVIC_WARN_BASE + 0x0067)
#define KE2100_WARN_NSUP_ERROR_QUERY                        (IVIC_WARN_BASE + 0x0068)
#define KE2100_WARN_NSUP_REV_QUERY                          (IVIC_WARN_BASE + 0x0069)



#define KE2100_ERROR_IO_GENERAL                             (IVIC_SPECIFIC_ERROR_BASE + 0x0214)
#define KE2100_ERROR_IO_TIMEOUT                             (IVIC_SPECIFIC_ERROR_BASE + 0x0215)
#define KE2100_ERROR_MODEL_NOT_SUPPORTED                    (IVIC_SPECIFIC_ERROR_BASE + 0x0216)
#define KE2100_ERROR_PERSONALITY_NOT_ACTIVE                 (IVIC_SPECIFIC_ERROR_BASE + 0x0211)
#define KE2100_ERROR_PERSONALITY_NOT_LICENSED               (IVIC_SPECIFIC_ERROR_BASE + 0x0213)
#define KE2100_ERROR_PERSONALITY_NOT_INSTALLED              (IVIC_SPECIFIC_ERROR_BASE + 0x0212)
#define KE2100_ERROR_MAX_TIME_EXCEEDED                      (IVIC_CLASS_ERROR_BASE + 0x0003)
#define KE2100_ERROR_TRIGGER_NOT_SOFTWARE                   (IVIC_CROSS_CLASS_ERROR_BASE + 0x0001)



#define KE2100_WARN_OVER_RANGE                              (IVIC_CLASS_WARN_BASE + 0x0001)


/**************************************************************************** 
 *---------------------------- End Include File ----------------------------* 
 ****************************************************************************/
#if defined(__cplusplus) || defined(__cplusplus__)
}
#endif
#endif // __KE2100_HEADER
