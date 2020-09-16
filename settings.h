#ifndef __SETTINGS__H__
#define __SETTINGS__H__

#define INFO_FIRMWARE_TYPE                   "buttonb"
#define INFO_FIRMWARE_VERSION                "1.0.0"

#define SUBSCRIPTION_TIMER_INTERVAL          120000
#define INTERNAL_TEMPERATURE_TIMER_INTERVAL  450000
#define VOLTAGE_TIMER_INTERVAL               450000

#define DEFAULT_POLL_PERIOD                  120000
#define DEFAULT_POLL_PERIOD_FAST             50
#define DEFAULT_POLL_PERIOD_FAST_TIMEOUT     500
#define DEFAULT_CHILD_TIMEOUT                240

#define ADC_SAMPLES_PER_CHANNEL              4

#define LED_SEND_NOTIFICATION                BSP_BOARD_LED_0
#define LED_RECV_NOTIFICATION                BSP_BOARD_LED_1
#define LED_ROUTER_ROLE                      BSP_BOARD_LED_2
#define LED_CHILD_ROLE                       BSP_BOARD_LED_3

#endif // __SETTINGS__H__
