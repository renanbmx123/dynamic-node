#include "BufferedSerial.h"
#include "DHT/DHT.h"
#include "DigitalInOut.h"
#include "DigitalOut.h"
#include "PinNames.h"
#include "TinyGPSPlus/TinyGPSPlus.h"
#include "UnbufferedSerial.h"
#include "XBeeLib/XBeeLib.h"
#include "MQ4/MQ4.h"
#include "ALTIMU/ALTIMU.h"
#include "mbed.h"
#include <cstdint>
#include <cstdio>

// Defines
#define TIMEOUT 120               // Set ticker time, for time interrupt.
#define NEW_PANID 0x1234         // Network Pan ID.
#define NEW_CHANNEL_MASK 0x7FFF  // Network Channel.
#define SEARCH_NETWORK_RETRIES 5 // Numbers of retries search network.
#define DATA_SIZE 200            // B3000yte numbers of data size.
#define SEND_RETRIES 10
#define SAMPLES 5     // Read and send some samples to network.
#define HEATER_TIME 20 // Time for MQ4 heat up.
//#define WRITE_LOCAL_DATA             // For write local data on the board usb
//storage.

/* Maquina de Estados:
https://www.inf.pucrs.br/~emoreno/undergraduate/SI/orgarq/class_files/Aula07.pdf
0 - Estado inicial, apenas configura os modulos e realiza a calibragem de alguns
sensores alem de sua inicialização, ales de configurar os parametros da rede,
como PANID e a potência do môdulo. 1 - Estado que verifica se o modulo Xbee
estabeleceu conexão na rede, caso sim vai para o proximo estágio senão apos N
tentativas entra no modo sleep 2 - Estado que realiza a leitura de todos os
sensores, armazenando os dados em um buffer separados por virgula. 3 - Estado
que realiza a tentativa de envio dos dados, caso não consiga apos N tentativas,
volta pro estado de verificação da rede. 4 - Estado sleep, aciona o pina de
PWR_CTRL que coloca em modo low power, acina o pino de sleep da Xbee, e coloca a
controladora em modo sleep. Fica neste modo por um periodo determiando de tempo
T, que pode variar conforme a politica estabelecida.
*/
typedef enum { ST_INIT = 0, ST_CON, ST_RDATA, ST_SDATA, ST_SLEEP } States;

struct StateMachine {
  States current_state, previous_state;

  bool *mcu_sleep; // microcontroler sleep control.

  int conn_retries = 0; // Count numbers of trying to connect
  FILE *fp;             // Logar dados no localfile da placa.

} sm;

void state_machine();

void st_init();
void st_con();
void st_rdata();
void st_sdata();
void st_sleep();

/*----------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------
 */

// Global objects.
DigitalOut led1 (LED1);
DigitalOut led2 (LED2);
DigitalOut led3 (LED3);
DigitalOut led4 (LED4);
// using namespace XBeeLib;      // Namespace for XbeeLib
DigitalIn on_sleep(p21); //   Xbee on sleep pin.;   // Xbee sleep read pin, for
                         //   check sleep state.
DigitalOut _sleep_req(p22); //   Xbee on sleep pin.; //Xbee sleep request pin.
DigitalOut mq4_heater(p17); // MQ4 heater power pin control.
DigitalOut GpsPwrPin(p15);  // GPS power pin control`
AnalogIn bat_pin(p16);      // Batery level read analog pin 16.
AnalogIn piezo_sensor(p20); // Piezo sensor on analog pin 20.

MQ4 mq4(p18); /*Gás sensor on analog pin 18.*/
DHT dht_th(
    p23,
    DHT22); // Dht (Digital humidity temperature) sensor on digital pin 23.;
            // /*Dht (Digital humidity temperature) sensor on digital pin 23.*/
Altimu lib_imu(p28, p27);
UnbufferedSerial GPSSerial(p9, p10, 9600);

LocalFileSystem local("local");
Ticker tim; // Ticker, for timer interrupt .
char gps_data, NorthSouth = 'F', EastWest = 'F';
uint8_t hms[3] = {0, 0, 0}, day_month[2] = {0, 0}, xbee_power_level = 0;
uint8_t data[DATA_SIZE],maxPowerLevel = 0; // char data[200],  /*Data to send*/
uint8_t sensor_change = 0;
uint8_t sendDataSize = 0;
uint16_t id = 0, year = 0;
uint16_t sleep_time = TIMEOUT;
float g[3] = {0, 0, 0} /*Gyroscope*/, acc[3] = {0, 0, 0} /*Accelerometer*/,
      mag[3] = {0, 0, 0} /*Magnetrometer*/,
      dht_t = 0 /*DHT22 temperature sensor*/,
      dht_h = 0 /*DHT22 humidity sensor*/, piezo = 0 /*Vibration sensor*/,
      vbat = 5 /*Batery percentage*/, lat = 0 /*Latitude*/,
      lon = 0 /*Longitude*/, alt = 0 /*Altitude from gps module*/,
      Balt = 0 /*Altitude from barometer sensor*/,
      speed = 0 /*Speed over the ground*/, Bpress = 0; /*Barometric pressure*/
TinyGPSPlus tgps;
MQ4_data_t MQ4_data; // Store gas sensor information.



XBeeLib::XBeeZB xbee(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 9600);
float bat_level();
bool is_radio_sleeping();
void sleep_radio();
void awake_radio();
void sleepManager();
float bat_level();
void XbeeNetworkSearch();
void file_write(char *data_file);

int main() {
  sm.previous_state = sm.current_state = ST_INIT;
  //printf("Iniciando...\n");
  state_machine();
}

void state_machine() {
  Timer t;

  while (1) {
    switch (sm.current_state) {
    case ST_INIT:
      led1 = 1;
      t.start();
      st_init();
      t.stop();
      led1 = 0;
      // printf("INIT,
      // t:%llu\n",chrono::duration_cast<chrono::milliseconds>(t.elapsed_time()).count());
      t.reset();
      break;
    case ST_CON:
      led2 = 1;
      t.start();
      st_con();
      t.stop();
      led2 = 0;
      // printf("CON,
      // t:%llu\n",chrono::duration_cast<chrono::milliseconds>(t.elapsed_time()).count());
      t.reset();
      break;
    case ST_RDATA:
      led3 = 1;
      t.start();
      st_rdata();
      t.stop();
      led3 = 0;
      // printf("RDATA,
      // t:%llu\n",chrono::duration_cast<chrono::milliseconds>(t.elapsed_time()).count());
      t.reset();
      break;
    case ST_SDATA:
      led4 = 1;
      t.start();
      st_sdata();
      t.stop();
      led4 = 0;
    //   printf("SDATA, t:%llu\n",chrono::duration_cast<chrono::milliseconds>(t.elapsed_time())
    //              .count());
      t.reset();
      break;
    case ST_SLEEP:
      t.start();
      st_sleep();
      t.stop();
    //   printf("SLEEP, t:%llu\n",
    //          chrono::duration_cast<chrono::milliseconds>(t.elapsed_time())
    //              .count());
      t.reset();
      break;
    }
  }
}
void st_init() {
  // Initial configuration.
  xbee_power_level = 0;
  XBeeLib::RadioStatus radioStatus = xbee.init();
  xbee.set_power_level(xbee_power_level);
  sm.mcu_sleep = new bool(false);
  MQ4_data_t MQ4_data; // Store gas sensor information.

  // Configure radio and get id number.
  uint32_t serialn_low;
  uint64_t current_panid;
  uint16_t current_channel_mask;
  uint8_t count=0;
  States nextState=ST_CON;
  // Set PAN_ID if is not configured.
  xbee.get_operating_panid(&current_panid);
  while (current_panid != NEW_PANID) {
    if (count > 100){
        count = 0;
        sm.previous_state = ST_INIT;
        nextState = ST_SLEEP;
        break;
    }
    count++;
    xbee.set_panid(NEW_PANID);
  }
  // Set CN (channel mask) if its is not set.
  xbee.get_channel_mask(&current_channel_mask);
  while (current_channel_mask != NEW_CHANNEL_MASK) {
    if (count > 100){
        count = 0;
        sm.previous_state = ST_INIT;
        nextState = ST_SLEEP;
        break;
    }
    count++;
    xbee.set_channel_mask(NEW_CHANNEL_MASK);
    xbee.write_config();
  }
  int panid;
  uint16_t channel;
  xbee.get_channel_mask(&channel);
  
  xbee.get_param("SL", &serialn_low);
  id = (uint16_t)serialn_low;

  
 
  _sleep_req = 0;
  mq4_heater = 1; // Set heater on, for Mq4 sensor set up.
  GpsPwrPin = 1; // Set gps on.
  ThisThread::sleep_for(std::chrono::seconds(HEATER_TIME));
  mq4.begin();               // Begin te R0 calculation and sensor calibration.
  sm.current_state = nextState; // Go to next state.
}

void st_con() {
  // whaiting for xbee join to network.
  //   sm.previous_state = sm.current_state;
 
  sm.conn_retries = 0; // Reset the count variable.
  xbee.set_power_level(xbee_power_level);

  sm.current_state = ST_RDATA;
  while (!xbee.is_joined()) {
    xbee.set_power_level(xbee_power_level);
    ThisThread::sleep_for(std::chrono::seconds(1));
    sm.conn_retries++;
    // printf("retries: %d\n",sm.conn_retries);
    if (sm.conn_retries == SEARCH_NETWORK_RETRIES) {
      if (xbee_power_level < maxPowerLevel) {
        xbee_power_level++;
        sm.conn_retries = 0;
        // printf("Power Level, %d\n",xbee_power_level);
      } else {
        // sm.conn_retries = 0; // Reset the count variable.
        xbee_power_level = 0;
        sm.previous_state = sm.current_state;
        sm.current_state = ST_SLEEP;
        // printf("Sleep\n");
        break;
      }
    } else {
      sm.current_state = ST_RDATA;
    }
  }
}

void st_rdata() {

  int gpsRetries = 0, alternate_senor = 0;
  // Read data
  // Read 3 axis gyroscope sensor.
  lib_imu.read_L3GD20(&g[0], &g[1], &g[2]);
  // Read linear accleration and magnecti field.
  lib_imu.read_LSM303D(&acc[0], &acc[1], &acc[2], &mag[0], &mag[1], &mag[2]);
  // Read imu pressure in hPa and Altitude in metters.
  lib_imu.read_LPS25H(&Bpress, &Balt);
  dht_th.readData();
  dht_t = dht_th.ReadTemperature(CELCIUS);
  dht_h = dht_th.ReadHumidity();
  vbat = bat_level();
  if (vbat > 4.9){
      maxPowerLevel = 4;
      sensor_change = 0;
  }
      
  else if (vbat > 4.8){
      maxPowerLevel = 3;
      sensor_change = 0;
  }  
  else if(vbat > 4.75){
      maxPowerLevel = 2;
      sensor_change = 1;
  }
  else if(vbat > 4.7){
      sensor_change = 0;
      maxPowerLevel = 1;
  }
        
  else{
      sensor_change = 1;
      maxPowerLevel = 0;
  }
  // Reading GPS data...
  if(sensor_change == 1 and alternate_senor == 0){
    
    for (int i = 0; i < 100; i++) {
        if (GPSSerial.readable()) {
            GPSSerial.read(&gps_data, 1);
            while (!tgps.encode(gps_data)) {
                GPSSerial.read(&gps_data, 1);
                gpsRetries ++;
                if (gpsRetries++ >= 1000) {
                    gpsRetries= 0;
                    break;
                }
            }
        }
   }    
   alternate_senor = 1;
  } else if(sensor_change == 1 and alternate_senor == 1){
      mq4.read(&MQ4_data);
      alternate_senor = 0;
  } else{
        mq4.read(&MQ4_data);
        for (int i = 0; i < 100; i++) {
            if (GPSSerial.readable()) {
                GPSSerial.read(&gps_data, 1);
                
                while (!tgps.encode(gps_data)) {
                    GPSSerial.read(&gps_data, 1);
                    gpsRetries ++;
                if (gpsRetries++ >= 1000) {
                     gpsRetries= 0;
                break;
                    }
                }
            }
   
        }
    }
 
  // Store data into a vector.

  sendDataSize = sprintf(
      (char *)data,
      "%04x,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%."
      "2f,%02d%02d%02d,%02d%02d%4d,%.3f,%.3f,%.2f,%c,%c,%.2f,%.2f,%.2f,%d",
      id, g[0], g[1], g[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2],
      Bpress, dht_h, dht_t, MQ4_data.ch4, piezo_sensor.read(), tgps.time.hour(),
      tgps.time.minute(), tgps.time.second(), tgps.date.year(),
      tgps.date.month(), tgps.date.day(), tgps.location.lng(),
      tgps.location.lat(), tgps.speed.mps(), tgps.NorthSouth, tgps.EastWest,
      Balt, tgps.altitude.meters(), bat_level(), xbee_power_level);

#if defined(WRITE_LOCAL_DATA)
  file_write(data);
#endif
  sm.current_state = ST_SDATA;
}
void st_sdata() {
  sm.previous_state = sm.current_state;
  sm.current_state = ST_SLEEP;
  const XBeeLib::TxStatus txStatus =
      xbee.send_data_to_coordinator(data, sendDataSize);
  uint8_t retries = SEND_RETRIES;
  while (txStatus != XBeeLib::TxStatusSuccess) {
    ThisThread::sleep_for(chrono::milliseconds(500));
    if ((retries--) == 0) {
      sm.previous_state = sm.current_state;
      sm.current_state = ST_SLEEP;
      break;
    }
  }
  
}

void st_sleep() {
  int alternate = 0;
  mq4_heater = 0; // Set heater on, for Mq4 sensor set up.
  GpsPwrPin = 0; // Set gps on.
  sleep_radio(); // sleep radio
  tim.attach(
      &sleepManager,
      std::chrono::seconds(
          sleep_time)); // Call this function every time when reach TIMEOUT value.
  if (sensor_change == 1 and alternate == 0 ){
      GpsPwrPin = 1; // Set gps on.  
      alternate = 1;
  }else if(sensor_change == 1 and alternate == 1 ){
      mq4_heater = 1; // Set heater on, for Mq4 sensor set up.
      alternate = 0;
  }else{
    mq4_heater = 1; // Set heater on, for Mq4 sensor set up.
    GpsPwrPin = 1; // Set gps on.
  }

  _sleep_req = 0;
  
  
  sleep();           // Sleep for a while when network is not reach.
  awake_radio();     // wake up radio
  sm.current_state = sm.previous_state;
  sm.current_state = ST_CON;
}

// Auxliar Function.
bool is_radio_sleeping() // Set radio to sleep, return 0 if radio sleep.
{
  // assert(on_sleep != NULL);
  return on_sleep.read() == 0;
}

void sleep_radio() // Sleep xbee radio function.
{
  // assert(_sleep_req != NULL);
  //_sleep_req.write(1);
  _sleep_req = 1;
}

void awake_radio() // Wake up xbee radio funcion.
{
  assert(_sleep_req != NULL);
  //_sleep_req.write(0);
  _sleep_req = 0;
  /* Wait until radio awakes. Typically 14 mS */
  ////printf("Sleep\n");
  while (is_radio_sleeping())
    ;
  ////printf("Awake\n");
}

void sleepManager() // Sleep Manager function.
{
  tim.detach(); // Deatach timer interrupt.
}

void file_write(char *data_file) {

  sm.fp = fopen("/local/out.csv", "a");
  if (sm.fp == NULL) {

    error("Arquivo nao encontrado\n\r");
  }
  fprintf(sm.fp, "%s\n", data_file);
  fclose(sm.fp);
}

float bat_level() { 
    return bat_pin.read() * 5; 
}