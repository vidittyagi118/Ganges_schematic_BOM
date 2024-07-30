#ifndef __DEFAULT_VALUES_CONFIG_H_
#define __DEFAULT_VALUES_CONFIG_H_


#define DEVICE_FIRMWARE_VERSION                            "D2.0V"          //"D1.6V" RT old             //D1.5V    old  

#define DEFAULT_DEV_ID                                     "FFFFF"
#define DEFAULT_DEV_HW_REV                                 "0.1V"
#define DEFAULT_DEV_SW_REV                                 DEVICE_FIRMWARE_VERSION
#define DEFAULT_DEV_SERIAL_NO                              "SL000"

#define DEFAULT_DEVICE_WELCOME_MSG_HEADER                  "** SOLAR CLEANER - "
#define DEFAULT_DEVICE_WELCOME_MSG_FOOTER                  " **"
  
#define DEFAULT_DEVICE_WELCOME_MSG                         DEFAULT_DEVICE_WELCOME_MSG_HEADER DEFAULT_DEV_SW_REV DEFAULT_DEVICE_WELCOME_MSG_FOOTER

#define DEFAULT_AUTO_PWM_DUTY                              100U
#define DEFAULT_MANUAL_PWM_DUTY                            100U
#define DEFAULT_EMERGENCY_PWM_DUTY                         100U
#define DEFAULT_APPROACH_PWM_DUTY                          70U
 
#define DEFAULT_BRUSH_TIME                                 3           //07D
#define DEFAULT_TIME_PWM                                   55
#define DEFAULT_TIME                                       15

#define DEFAULT_ACEL_START_COUNT_DIFF                       (1)
#define DEFAULT_DECEL_START_COUNT_DIFF                      (1)
#define DEFAULT_PWM_ACCEL_TIME                              2000U
#define DEFAULT_PWM_DECEL_TIME                              1000U
                                                  
#define DEFAULT_ACCEL                                       235U
#define DEFAULT_DECEL                                       235U


#define DEFAULT_MIN_BATT_VOLTAGE_MOTOR                      (22.2)
#define DEFAULT_MAX_MOTOR_CURRENT_MOTOR                     6U
#define DEFAULT_ILOAD_MAX_NOISE_TIME                        500U
#define DEFAULT_ILOAD_MAX_FREQ_TIME                         15000U
#define DEFAULT_PAUSE_DELAY_TIME                            3000U

//#define DEFAULT_MAX_POS_COUNT_FOR_PV_SETUP                  25  //20000
//#define DEFAULT_MAX_NEG_COUNT_FOR_PV_SETUP                  25  //20000

//07D
#define DEFAULT_TRACK_DISTANCE                              0
#define DEFAULT_TRACK_COUNT                                 0
#define DEFAULT_RFID_DATA                                   0

#define DEFAULT_FORWARD_COUNT                               0
#define DEFAULT_REVERSE_COUNT                               0

#define DEFAULT_MAX_POS_COUNT_FOR_PV_SETUP                  21  //20000
#define DEFAULT_MAX_NEG_COUNT_FOR_PV_SETUP                  21  //20000
#define DEFAULT_POSITIVE_DISTANCE                          5000U  //20000
#define DEFAULT_NEGATIVE_DISTANCE                          5000U  //20000
#define DEFAULT_WHEEL_DIA                                  150U
#define DEFAULT_PULSE                                      2U
#define DEFAULT_INTERVAL                                   (1.0f)
#define DEFAULT_CYCLE_FREQUENCY                            5U  
#define DEFAULT_COUNT                                      5U
#define DEFAULT_COUNTINUE                                  0
#define DEFAULT_COMCOUNT                                   42
#define DEFAULT_COMDISTANCE                                10000

#define DEFAULT_MAX_TEMP_SENSOR_ONE                         100
#define DEFAULT_MAX_TEMP_SENSOR_TWO                         100

#define DEFAULT_BRUSH_MOT_DIRECTION                         0
#define DEFAULT_MOT_1_DIRECTION                             0
#define DEFAULT_MOT_2_DIRECTION                             0

#define DEFAULT_AUTO_SCHEDULED_TIME_HOUR                    (00)
#define DEFAULT_AUTO_SCHEDULED_TIME_MINUTE                  (00)

#define DEFAULT_ILOAD_MAX_REPEAT_COUNT                      1U
#define DEFAULT_ILOAD_MAX_NOISE_TIME                        500U
#define DEFAULT_ILOAD_MAX_FREQ_TIME                         15000U
#define DEFAULT_PAUSE_DELAY_TIME                            3000U

#define DEFAULT_ILOAD1_MAX_VALUE                            (2.0f)
#define DEFAULT_ILOAD2_MAX_VALUE                            (2.0f)
#define DEFAULT_ILOAD3_MAX_VALUE                            (2.0f)

#define DEFAULT_MIN_BATT_SOC                                (50)
#define DEFAULT_BRUSH_ENABLED_STATE                         true
#define DEFAULT_LINEAR_ENABLED_STATE                        true
#define DEFAULT_EDGE_SENSOR_ENABLED_STATE                   true
#define DEFAULT_ROTATE_SENSOR_ENABLED_STATE                 true

#define HEARTBEAT_INTERVAL_MS_DEFAULT                       3000U

#define NO_OF_HEARTBEAT_MESSAGES_DEFAULT                    5U
#define HEARTBEAT_ENABLE_DEFAULT                            true
#define MAX_ZIGBEE_RECONNECT_TIME_DEFAULT                   (1.5)*(60)*(1000) //1.5 minute

#endif