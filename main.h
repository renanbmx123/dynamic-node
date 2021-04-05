
#ifndef __MAIN_H_
#define __MAIN_H_

#include "mbed.h"
#include "ALTIMU.h"
#include "TinyGPSPlus.h"
#include "MQ4.h"
#include "DHT.h"
#include "XBeeLib.h"
#include <cstdint>

// Defines
#define TIMEOUT                50    // Set ticker time, for time interrupt.
#define NEW_PANID             0x1234 // Network Pan ID.
#define NEW_CHANNEL_MASK      0x7FFF // Network Channel.
#define SEARCH_NETWORK_RETRIES  20   // Numbers of retries search network.
#define DATA_SIZE               200  // Byte numbers of data size.
#define SEND_RETRIES            5

//#define WRITE_LOCAL_DATA             // For write local data on the board usb storage.
//#define DEBUG_LEDS                 // USED FOR DEBUG.
#define SAMPLES                 10   // Read and send some samples to network.

// State Machine
#define SLEEP_STATE             2    // Sleep state, shutdown all modules and sensor then go to sleep.
#define SEARCH_NETWORK_STATE    1    // Search for network, stays on this state if network doenst show up.
#define READ_SEND_DATA_STATE    0    // This state send data after reading it.
/* New state machine*/
/* Estados:  https://www.inf.pucrs.br/~emoreno/undergraduate/SI/orgarq/class_files/Aula07.pdf
0 - Estado inicial, apenas configura os modulos e realiza a calibragem de alguns sensores alem de sua inicialização, 
ales de configurar os parametros da rede, como PANID e a potência do môdulo.
1 - Estado que verifica se o modulo Xbee estabeleceu conexão na rede, caso sim vai para o proximo estágio senão apos 
N tentativas entra no modo sleep
2 - Estado que realiza a leitura de todos os sensores, armazenando os dados em um buffer separados por virgula.
3 - Estado que realiza a tentativa de envio dos dados, caso não consiga apos N tentativas, volta pro estado de verificação da rede.
4 - Estado sleep, aciona o pina de PWR_CTRL que coloca em modo low power, acina o pino de sleep da Xbee, e coloca a controladora
em modo sleep. Fica neste modo por um periodo determiando de tempo T, que pode variar conforme a politica estabelecida. 
*/
typedef enum {
    ST_INIT = 0, ST_NET, ST_RDATA, ST_SDATA, ST_SLEEP
} States;
  
struct StateMachine {
    States current_state, previous_state;
    MQ4*        mq4;                              /*Gás sensor on analog pin 18.*/
    DHT*        dht_th;                     /*Dht (Digital humidity temperature) sensor on digital pin 23.*/
    Altimu*     lib_imu;

    char        gps_data, NorthSouth = 'F', EastWest =  'F';
    uint8_t     hms[3], day_month[2], xbee_power_level = 0;
    uint8_t     data[DATA_SIZE]; //char data[200],  /*Data to send*/
    uint16_t    id, year;
    float       g[3]/*Gyroscope*/,acc[3]/*Accelerometer*/,mag[3]/*Magnetrometer*/,dht_t/*DHT22 temperature sensor*/,
                dht_h/*DHT22 humidity sensor*/, piezo/*Vibration sensor*/,vbat/*Batery percentage*/,
                lat/*Latitude*/,lon/*Longitude*/,alt/*Altitude from gps module*/,Balt/*Altitude from barometer sensor*/,
                speed/*Speed over the ground*/,Bpress;/*Barometric pressure*/
    TinyGPSPlus tgps;        
    MQ4_data_t MQ4_data;    // Store gas sensor information.

    BufferedSerial* GPSSerial;
    XBeeLib::XBeeZB*     xbee;
    bool    *mcu_sleep;   // microcontroler sleep control.
    Ticker  *tim; // Ticker, for timer interrupt . 
    int conn_retries = 0; // Count numbers of trying to connect 
    FILE *fp;   // Logar dados no localfile da placa.
    

} sm;

void state_machine();

void st_init ();
void st_net ();
void st_rdata();
void st_sdata();
void st_sleep();



/*----------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------
*/


// Global objects.
//using namespace XBeeLib;      // Namespace for XbeeLib
DigitalOut* sleep_req = NULL; //Xbee sleep request pin.
DigitalIn*  on_sleep = NULL;   // Xbee sleep read pin, for check sleep state.
AnalogIn    bat_pin(p16);        // Batery level read analog pin 16.
DigitalOut  mq4_heater(p17);   // MQ4 heater power pin control.
DigitalOut  GpsPwrPin(p15);    // GPS power pin control`
AnalogIn piezo_sensor(p20);                // Piezo sensor on analog pin 20.


float bat_level();
bool is_radio_sleeping();
void sleep_radio();
void awake_radio();
void sleepManager();
float bat_level();
void XbeeNetworkSearch();
void file_write(char* data_file);

#endif /* __MAIN_H_ */