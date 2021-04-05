#include "main.h"
#include "FileBase.h"
#include "FileHandle.h"
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <rt_sys.h>

int main(){

    sm.current_state = sm.current_state = ST_INIT;
    while(1){

    }
}

void st_init (){ 
  // Initial configuration.
  on_sleep     = new DigitalIn(RADIO_ON_SLEEP);  //   Xbee on sleep pin.
  sleep_req    = new DigitalOut(RADIO_SLEEP_REQ); // Xbee Sleep request pin
  sm.dht_th    = new DHT(p23,DHT22);                // Dht (Digital humidity temperature) sensor on digital pin 23.
  sm.mq4       = new MQ4(p18);                              // Gás sensor on analog pin 18.
  sm.GPSSerial = new BufferedSerial(p9, p10,9600);
  sm.lib_imu   = new Altimu(p28,p27);
  LocalFileSystem local("local");
  sm.xbee_power_level = 0;
  sm.xbee =new  XBeeLib::XBeeZB(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 9600);
  XBeeLib::RadioStatus radioStatus = sm.xbee->init();
  sm.xbee->set_power_level(sm.xbee_power_level);
  sm.mcu_sleep = new bool(false);

  // Configure radio and get id number.
  uint32_t serialn_low;
  uint64_t current_panid;
  uint16_t current_channel_mask;
  // Get 2 bytes of the address module, set Node id with it.
  sm.xbee->get_param("SL", &serialn_low);
  sm.id = (uint16_t)serialn_low;
  // Set PAN_ID if is not configured.
  sm.xbee->get_operating_panid(&current_panid);
  if (current_panid != NEW_PANID) {
    sm.xbee->set_panid(NEW_PANID);
  }
  // Set CN (channel mask) if its is not set.
  sm.xbee->get_channel_mask(&current_channel_mask);
  if (current_channel_mask != NEW_CHANNEL_MASK) {
    sm.xbee->set_channel_mask(NEW_CHANNEL_MASK);
    sm.xbee->write_config();
  }
  mq4_heater = 1;         // Set heater on, for Mq4 sensor set up.
  GpsPwrPin = 1;          // Set gps on.
  ThisThread::sleep_for(chrono::milliseconds(20*1000));
  sm.mq4->begin();        // Begin te R0 calculation and sensor calibration.
  sm.current_state = ST_NET; // Go to next state.
}

void st_net (){
  // whaiting for xbee join to network.
  while (!sm.xbee->is_joined()) {
    sm.xbee->set_power_level(sm.xbee_power_level);
    ThisThread::sleep_for(std::chrono::seconds(1));
    sm.conn_retries ++;
    if(sm.conn_retries == SEARCH_NETWORK_RETRIES){
      if(sm.xbee_power_level < 4){
        sm.xbee_power_level ++;
      }
        else{
            // sleep_radio();                      // sleep radio
            // tim.attach(&sleepManager,std::chrono::seconds(TIMEOUT));  // Call this function every time when reach TIMEOUT value.
            // sleep();                            // Sleep for a while when network is not reach.
            // awake_radio();                      // wake up radio
            sm.conn_retries = 0;                   // Reset the count variable.
            sm.current_state = ST_SLEEP;

        }
    }else {
        sm.current_state = ST_SDATA;
    }
  }
}

void st_rdata(){
    //Read data
    // Read 3 axis gyroscope sensor.
    sm.lib_imu->read_L3GD20(&sm.g[0],&sm.g[1],&sm.g[2]);
    // Read linear accleration and magnecti field.
    sm.lib_imu->read_LSM303D(&sm.acc[0],&sm.acc[1],&sm.acc[2],&sm.mag[0],&sm.mag[1],&sm.mag[2]);
    // Read imu pressure in hPa and Altitude in metters.
    sm.lib_imu->read_LPS25H(&sm.Bpress, &sm.Balt);
    sm.dht_th->readData();
    sm.mq4->read(&sm.MQ4_data);
    // Reading GPS data...
    for(int i =0;i<100;i++){
      if(sm.GPSSerial->readable()){
      sm.GPSSerial->read(&sm.gps_data ,1);
        while (!sm.tgps.encode(sm.gps_data));
        i = 100;
            }
        }
        int id;
        char data;
        sprintf(data, "asd %d",id);
        //sprintf(sm.data, "%d", sm.id);
        //Store data into a vector.
        // sprintf(sm.data,"%4x,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%02d%02d%02d,%02d%02d%4d,%.3f,%.3f,%.2f,%c,%c,%.2f,%.2f,%.2f,%d",sm.id,
        // sm.g[0], sm.g[1], sm.g[2],sm.acc[0],sm.acc[1], sm.acc[2], sm.mag[0], sm.mag[1], sm.mag[2],sm.Bpress, sm.dht_h,sm.dht_t,sm.MQ4_data.ch4,piezo_sensor.read(), sm.tgps.time.hour(),
        // sm.tgps.time.minute(),sm.tgps.time.second(),sm.tgps.date.year(),sm.tgps.date.month(),sm.tgps.date.day(), sm.tgps.location.lng(), sm.tgps.location.lat(), sm.tgps.speed.mps(), 
        // sm.tgps.NorthSouth, sm.tgps.EastWest, sm.Balt, sm.tgps.altitude.meters(),bat_level(),sm.xbee_power_level);
        
        #if defined(WRITE_LOCAL_DATA)
        file_write(data);
        #endif        
        sm.current_state = ST_SDATA;

}
void st_sdata(){
    const XBeeLib::TxStatus txStatus = sm.xbee->send_data_to_coordinator(sm.data, sizeof(sm.data));
    uint8_t retries = SEND_RETRIES;
    while(txStatus != XBeeLib::TxStatusSuccess) {
        ThisThread::sleep_for(chrono::milliseconds(300));
        if((--retries) == 0){
           sm.current_state = ST_NET;
           return;
            }
    }
        sm.current_state = ST_SLEEP;
}


void st_sleep(){
     sleep_radio();                      // sleep radio
     sm.tim->attach(&sleepManager,std::chrono::seconds(TIMEOUT));  // Call this function every time when reach TIMEOUT value.
     sleep();                            // Sleep for a while when network is not reach.
     awake_radio();                      // wake up radio
}

// Auxliar Function.
bool is_radio_sleeping()            // Set radio to sleep, return 0 if radio sleep.
{
  assert(on_sleep != NULL);
  return on_sleep->read() == 0;
}

void sleep_radio()                 // Sleep xbee radio function.
{
  assert(sleep_req != NULL);
  sleep_req->write(1);
}

void awake_radio()                // Wake up xbee radio funcion.
{
  assert(sleep_req != NULL);
  sleep_req->write(0);
  /* Wait until radio awakes. Typically 14 mS */
  while(is_radio_sleeping());
}

void sleepManager()              // Sleep Manager function.
{
  sm.tim->detach();   // Deatach timer interrupt.
  sm.current_state = ST_NET;
}

void file_write(char* data_file){
    
    sm.fp = fopen("/local/out.csv", "a");
    if(sm.fp == NULL){
        
        error("Arquivo nao encontrado\n\r");
    }
    fprintf(sm.fp,"%s\n",data_file);
    fclose(sm.fp);
}

