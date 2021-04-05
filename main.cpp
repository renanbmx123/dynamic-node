/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

// #include "mbed.h"

// #define WAIT_TIME_MS 500 
// DigitalOut led1(LED1);

// int main()
// {
//     printf("This is the bare metal blinky example running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

//     while (true)
//     {
//         led1 = !led1;
//         thread_sleep_for(WAIT_TIME_MS);
//     }
// }


#include "main.h"
#include <chrono>

// Main function.
int main(){
  // Serial Configuration.
  sleep_req = new DigitalOut(RADIO_SLEEP_REQ); // Xbee Sleep request pin
  on_sleep = new DigitalIn(RADIO_ON_SLEEP);  //   Xbee on sleep pin.
  uint8_t state;
  char        gps_data;
  char        data[200],  // Data to send
  NorthSouth = 'F',       // North South direction
  EastWest = 'F';         // East West direction
  uint16_t    id;         // Node identification
  uint8_t     hour;       //**********************
  uint8_t     minute;     //**********************
  uint8_t     second;     //**** Data and time UTC
  uint8_t     day;        //**** ,from gps module.
  uint8_t     month;      //**********************
  uint16_t    year;       //**********************
  float       g[3],       // Gyroscope
  acc[3]                  // Accelerometer
  ,mag[3],                // Magnetrometer
  dht_t,                  // DHT22 temperature sensor
  dht_h,                  // DHT22 humidity sensor
  piezo,                  // Vibration sensor
  vbat,                   // Batery percentage.
  lat,                    // Latitude .
  lon,                    // Longitude.
  alt,                    // Altitude from gps module.
  Balt,                   // Altitude from barometer sensor.
  speed;                  // Speed over the ground
  float      Bpress;      // Barometric pressure
  // Instantiate objects.
  DHT dht_th(p23,DHT22);                     // Dht (Digital humidity temperature) sensor on digital pin 23.
  AnalogIn piezo_sensor(p20);                // Piezo sensor on analog pin 20.
  MQ4 mq4(p18);                              // Gás sensor on analog pin 18.
  BufferedSerial GPSSerial(p9, p10,9600);
  Altimu lib_imu(p28,p27);
  TinyGPSPlus tgps;
  // Xbee object configuration.
  RadioStatus radioStatus = xbee.init();
  xbee.set_power_level(xbee_power_level);

  // Configure radio and get id number.
  uint32_t serialn_low;
  uint64_t current_panid;
  uint16_t current_channel_mask;
  // Get 2 bytes of the address module, set Node id with it.
  xbee.get_param("SL", &serialn_low);
  id = (uint16_t)serialn_low;
  // Set PAN_ID if is not configured.
  xbee.get_operating_panid(&current_panid);
  if (current_panid != NEW_PANID) {
    xbee.set_panid(NEW_PANID);
  }
  // Set CN (channel mask) if its is not set.
  xbee.get_channel_mask(&current_channel_mask);
  if (current_channel_mask != NEW_CHANNEL_MASK) {
    xbee.set_channel_mask(NEW_CHANNEL_MASK);
    xbee.write_config();
  }
  mq4_heater = 1;         // Set heater on, for Mq4 sensor set up.
  GpsPwrPin = 1;          // Set gps on.
  #if defined(DEBUG_LEDS)
  led1 = 1;
  #endif
  //wait(30);              // Wait sensor warm up on cold start, first start.
  ThisThread::sleep_for(chrono::milliseconds(20*1000));
  #if defined(DEBUG_LEDS)
  led1 = 0;
  #endif
  MQ4_data_t MQ4_data;    // Store gas sensor information.
  mq4.begin();            // Begin te R0 calculation and sensor calibration.
  // Verify network connection at first start.
  XbeeNetworkSearch();
  state = READ_SEND_DATA_STATE;

  while(1){

    switch (state) {
      case READ_SEND_DATA_STATE:
      //Read data
        for (int j = 0; j < 10; j++){
        // Read 3 axis gyroscope sensor.
        lib_imu.read_L3GD20(&g[0],&g[1],&g[2]);
        // Read linear accleration and magnecti field.
        lib_imu.read_LSM303D(&acc[0],&acc[1],&acc[2],&mag[0],&mag[1],&mag[2]);
        // Read imu pressure in hPa and Altitude in metters.
        lib_imu.read_LPS25H(&Bpress, &Balt);
        // Read piezoeletric sensor.
        piezo = piezo_sensor.read();
        // Read 9V baterry charge.
        vbat = bat_level();
        dht_th.readData();
        // Read DHT22 temperature in celcius.
        dht_t = dht_th.ReadTemperature(CELCIUS);
        // Read DHT22 relative humidity
        dht_h = dht_th.ReadHumidity();
        // Read MQ4 gas sensor
        mq4.read(&MQ4_data);
        // Reading GPS data...
        for(int i =0;i<100;i++){
          if(GPSSerial.readable()){
            //while (!tgps.encode(GPSSerial.getc()));
            GPSSerial.read(&gps_data ,1);
            while (!tgps.encode(gps_data));
            i = 100;
          }
        }
        if(tgps.location.isValid()){
          lat = tgps.location.lat();
          lon = tgps.location.lng();
        }else{
          lat = 0;
          lon = 0;
        }
        if(tgps.altitude.isValid()){
          alt = tgps.altitude.meters();
        }else{
          alt = 0;
        }
        if(tgps.speed.isValid()){
          speed = tgps.speed.mps();
        }else{
          speed = 0;
        }
        if((tgps.NorthSouth == 'S') |
           (tgps.NorthSouth =='N')){
          NorthSouth = tgps.NorthSouth;
        }
        else{
          NorthSouth = 'F';
        }
        if ((tgps.EastWest == 'W') |
            (tgps.EastWest == 'E')){
          EastWest = tgps.EastWest;
        }else{
          EastWest = 'F';
        }
        if (tgps.time.isValid())
        {
          //second[1] = (tgps.time.second()%10)+48;
          //second[0] = (tgps.time.second()/10) +48;
          //minute[1] = (tgps.time.minute() %10)+48;
          second = tgps.time.second();//minute[0] = (tgps.time.minute()/10) +48;
          minute = tgps.time.minute();//hour[1] = (tgps.time.hour()%10)+48;
          hour   = tgps.time.hour();//hour[0] = (tgps.time.hour()/10) +48;
          day    = tgps.date.day();
          year   = tgps.date.year();
          month  = tgps.date.month();
          
        }
        //Store data into a vector.
        sprintf(data,"%4x,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%02d%02d%02d,%02d%02d%4d,%.3f,%.3f,%.2f,%c,%c,%.2f,%.2f,%.2f,%d",id,
        g[0], g[1], g[2],acc[0],acc[1], acc[2], mag[0], mag[1], mag[2],Bpress, dht_h,dht_t,MQ4_data.ch4,piezo, hour, minute,second,year,month,day, lon, lat, speed, NorthSouth, EastWest, Balt, alt,vbat,xbee_power_level);
        #if defined(WRITE_LOCAL_DATA)
        file_write(data);
        #endif        
        while(send_data_to_coordinator(xbee, data) != TxStatusSuccess) // try to send data, every 100 ms
        {
          #if defined(DEBUG_LEDS)
          led1 = !led1;
          #endif
          //wait_ms(500);
          ThisThread::sleep_for(chrono::milliseconds(300));
        }
        //wait_ms(800);
        ThisThread::sleep_for(chrono::milliseconds(500));
      }
        state = SLEEP_STATE;
      break;
      case SLEEP_STATE:
        //Debug Led
        #if defined(DEBUG_LEDS)
        led2 = !led2;
        #endif

        sleep_radio();          // Set radio to sleep.
        mq4_heater = 0;         // Disable Mq4 sensor heater resistence.
        GpsPwrPin = 0;
        tim.attach(&sleepManager,std::chrono::seconds(TIMEOUT));  // attach timeout function
        sleep();
        //Debug Led
        #if defined(DEBUG_LEDS)
        led2 = !led2;
        #endif

        mq4_heater = 1;           // Disable Mq4 sensor heater resistence.
        GpsPwrPin = 1;            // Enable gps module.
        awake_radio();
        //wait(30);
        state = SEARCH_NETWORK_STATE;
        // whaiting for xbee join to network.
      break;
      case SEARCH_NETWORK_STATE:    // Looking for network before
        XbeeNetworkSearch();
        state = READ_SEND_DATA_STATE;
      break;
    }

  }
}

// Auxliar Function.
void file_write(char* data_file){
    
    fp = fopen("/local/out.csv", "a");
    if(fp == NULL){
        
        error("Arquivo nao encontrado\n\r");
    }
    fprintf(fp,"%s\n",data_file);
    fclose(fp);
}

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
  tim.detach();   // Deatach timer interrupt.
  #if defined(DEBUG_LEDS)
  led4 = !led4;
  #endif
}
TxStatus send_data_to_coordinator(XBeeZB& xbee, char *data)
{
  const TxStatus txStatus = xbee.send_data_to_coordinator((const uint8_t *)data, strlen(data));
  return txStatus;
}

float bat_level(){
  return bat_pin.read()*5;
}

void XbeeNetworkSearch(){
  // whaiting for xbee join to network.
  while (!xbee.is_joined()) {
    xbee.set_power_level(xbee_power_level);
    //wait_ms(1000);    
    ThisThread::sleep_for(std::chrono::seconds(1));
    conn_retries ++;
    if(conn_retries == SEARCH_NETWORK_RETRIES){
      if(xbee_power_level < 4)
      {
        xbee_power_level ++;
      }
        else{
            
            sleep_radio();                      // sleep radio
            tim.attach(&sleepManager,std::chrono::seconds(TIMEOUT));  // Call this function every time when reach TIMEOUT value.
            sleep();                            // Sleep for a while when network is not reach.
            awake_radio();                      // wake up radio
            conn_retries = 0;                   // Reset the count variable.
        }
    }
    #if defined(DEBUG_LEDS)
    led3 = !led3;
    #endif
  }
}