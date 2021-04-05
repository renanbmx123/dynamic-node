
#ifndef __MAIN_H_
#define __MAIN_H_

#include "mbed.h"
#include "ALTIMU.h"
#include "TinyGPSPlus.h"
#include "MQ4.h"
#include "DHT.h"
#include "XBeeLib.h"

// Defines
#define TIMEOUT                50    // Set ticker time, for time interrupt.
#define NEW_PANID             0x1234 // Network Pan ID.
#define NEW_CHANNEL_MASK      0x7FFF // Network Channel.
#define SEARCH_NETWORK_RETRIES  20   // Numbers of retries search network.

//#define WRITE_LOCAL_DATA             // For write local data on the board usb storage.
//#define DEBUG_LEDS                 // USED FOR DEBUG.
#define SAMPLES                 10   // Read and send some samples to network.

// State Machine
#define SLEEP_STATE             2    // Sleep state, shutdown all modules and sensor then go to sleep.
#define SEARCH_NETWORK_STATE    1    // Search for network, stays on this state if network doenst show up.
#define READ_SEND_DATA_STATE    0    // This state send data after reading it.


// Global objects.
using namespace XBeeLib;      // Namespace for XbeeLib
DigitalOut* sleep_req = NULL; //Xbee sleep request pin.
DigitalIn* on_sleep = NULL;   // Xbee sleep read pin, for check sleep state.
AnalogIn bat_pin(p16);        // Batery level read analog pin 16.
DigitalOut mq4_heater(p17);   // MQ4 heater power pin control.
DigitalOut GpsPwrPin(p15);    // GPS power pin control`
// This is used for debug, these leds are built in leds.
#if defined(DEBUG_LEDS)
DigitalOut led1(LED1);        // Debug Led 1
DigitalOut led2(LED2);        // Debug Led 1
DigitalOut led3(LED3);        // Debug Led 1
DigitalOut led4(LED4);        // Debug Led 1
#endif
// Control variables
uint8_t xbee_power_level = 0; //Xbee radio power level, set to minumun
bool mcu_sleep = false;   // microcontroler sleep control.
Ticker tim; // Ticker, for timer interrupt .
int conn_retries = 0; // Count numbers of trying to connect
LocalFileSystem local("local");
FILE *fp;   // Logar dados no localfile da placa.
// ---------------------------------

XBeeZB xbee = XBeeZB(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 9600);
//Serial  pc(USBTX, USBRX,9600);// Serial Debug.
// Auxliar Function.
float bat_level();
bool is_radio_sleeping();
void sleep_radio();
void awake_radio();
void sleepManager();
TxStatus send_data_to_coordinator(XBeeZB& xbee, char *data);
float bat_level();
void XbeeNetworkSearch();
void file_write(char* data_file);

#endif /* __MAIN_H_ */