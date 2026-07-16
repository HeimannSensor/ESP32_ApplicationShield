/*** PROGRAMM INFO***************************************************************************************
  source code for ESP32 and HTPAd Application Shield
  name:           ESP32_HTPAd_60x40.ino
  version/date:   2.2 / 20 Dec 2022
  programmer:     Heimann Sensor GmbH / written by Dennis Pauer (pauer@heimannsensor.com)
*********************************************************************************************************/



/*** MODES **********************************************************************************************
  The source code includes three ways to interact with the sensor:
  - via WIFI you can stream thermal images in our GUI (#define WIFIMODE)
  - via the SERIAL monitor you can observe the sensor data as text output (#define SERIALMODE)
  - via ACCESSPOINT the ESP32 creates the wifi network. You have to connect your computer to this network
    to stream thermal images in the GUI (#define ACCESSPOINT)
  Both modes are contain in the same code and you can activate one or both by activate the matching define.
*********************************************************************************************************/
#define WIFIMODE
#define SERIALMODE
//#define ACCESSPOINT // automatically diablead if WIFIMODE is active

/*** NETWORK INFORMATION ********************************************************************************
  If you want to use the WIFI function, you have to change ssid and pass.
*********************************************************************************************************/
char ssid[] = "Balogh";
char pass[] = "Balogh1234";


#include <SPI.h>
#include "def.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <pins_arduino.h>
#include "uTimerLib.h"


//-----------------------------------------
// WIFI
int status = WL_IDLE_STATUS;
int keyIndex = 0;
unsigned int localPort = 30444;
char packetBuffer[256];
char  ReplyBuffer[] = "acknowledged";
uint8_t mac[6];
uint8_t ip_partner[4];
uint8_t device_bind;
signed short WIFIstrength;
WiFiUDP udp;
unsigned char wifi_on = 0;
//-----------------------------------------

// STRUCT WITH ALL SENSOR CHARACTERISTICS
struct characteristics {
  unsigned short NumberOfPixel;
  unsigned char NumberOfBlocks;
  unsigned char RowPerBlock;
  unsigned short PixelPerBlock;
  unsigned short PixelPerColumn;
  unsigned short PixelPerRow;
  unsigned char AllowedDeadPix;
  unsigned short TableNumber;
  unsigned short TableOffset;
  unsigned char PTATPos;
  unsigned char VDDPos;
  unsigned char PTATVDDSwitch;
  unsigned char CyclopsActive;
  unsigned char CyclopsPos;
  unsigned char DataPos;
};

characteristics DevConst = {
  NUMBER_OF_PIXEL,
  NUMBER_OF_BLOCKS,
  ROW_PER_BLOCK,
  PIXEL_PER_BLOCK,
  PIXEL_PER_COLUMN,
  PIXEL_PER_ROW,
  ALLOWED_DEADPIX,
  TABLENUMBER,
  TABLEOFFSET,
  PTAT_POS,
  VDD_POS,
  PTAT_VDD_SWITCH,
  ATC_ACTIVE,
  ATC_POS,
  DATA_POS,
};

//-----------------------------------------
// EEPROM DATA
unsigned char mbit_user, bias_user, clk_user, bpa_user;
unsigned long id;
unsigned char arraytype;
unsigned char bitstoshift;
signed char globaloff;
unsigned short globalgain;
//-----------------------------------------
// SENSOR DATA
unsigned short data_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
unsigned char RAMoutput[2 * NUMBER_OF_BLOCKS + 2][BLOCK_LENGTH];

/*
  RAMoutput is the place where the raw values are saved

  example, order for 80x64:
  RAMoutput[0][]... data from block 0 top
  RAMoutput[1][]... data from block 1 top
  RAMoutput[2][]... data from block 2 top
  RAMoutput[3][]... data from block 3 top
  RAMutput[4][]... electrical offset top
  RAMoutput[5][]... electrical offset bottom
  RAMoutput[6][]... data from block 3 bottom
  RAMoutput[7][]... data from block 2 bottom
  RAMoutput[8][]... data from block 1 bottom
  RAMoutput[9][]... data from block 0 bottom

*/
unsigned short eloffset[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
unsigned char statusreg;
unsigned short Ta, ptat_av_uint16, vdd_av_uint16, ATC0, ATC1;


// BUFFER for PTAT,VDD and elOffsets
// PTAT:
uint16_t ptat_buffer[PTAT_BUFFER_SIZE];
uint16_t ptat_buffer_average;
uint8_t use_ptat_buffer = 0;
uint8_t ptat_i = 0;
uint8_t PTATok = 0;
// VDD:
uint16_t vdd_buffer[VDD_BUFFER_SIZE];
uint16_t vdd_buffer_average;
uint8_t use_vdd_buffer = 0;
uint8_t vdd_i = 0;
// electrical offsets:
uint8_t use_eloffsets_buffer = 0;
uint8_t eloffsets_i = 0;
uint8_t new_offsets = 1;

// PROGRAMM CONTROL
bool switch_ptat_vdd = 0;
unsigned char adr_offset = 0x00;
unsigned char send_data = 0;
unsigned short picnum = 0;
unsigned char state = 0;
unsigned char read_block_num = START_WITH_BLOCK; // start with electrical offset
unsigned char read_eloffset_next_pic = 0;
unsigned char gui_mode = 0;
unsigned char wait_pic = 0;
bool ReadingRoutineEnable = 1;

// OTHER
unsigned char print_state = 0;

//LEDs
unsigned char pinLEDred = 33;
unsigned char pinLEDgreen = 25;
unsigned char pinLEDblue = 32;



unsigned short timert;
char serial_input = 'm';

#define SPI_CLK 14
#define SPI_MISO  13
#define SPI_MOSI  12
#define SPI_CS 2




/********************************************************************
 ********************************************************************
    - - - PART 1: ARDUINO FUNCTIONS - - -
    ISR()...    interrupt service routine, called by timer
    setup()...  this function is only called once before jumping into
                the main function loop()
    loop()...   main function of an arduino code
    set_LED(unsigned char red, unsigned char green, unsigned char blue)
 ********************************************************************
 *********************************************************************/



/********************************************************************
   Function:        ISR()
   Description:     interrupt service routine; called by timer
 *******************************************************************/
void IRAM_ATTR ISR(void)
{


  // conrtol LED state
  if ((read_eloffset_next_pic == 1) && (read_block_num == 1)) {
    static bool toggle2 = false;
    toggle2 = !toggle2;

    if (send_data) {
      //toggle2 != toggle2;

      WIFIstrength = WiFi.RSSI(); // get wifi signal strength
      if (WIFIstrength < -74)
        set_LED(toggle2, 0, 0); // really poor wifi
      else
        set_LED(0, toggle2, 0); // wifi okay
    }
    else {
      if (device_bind)
        set_LED(0, toggle2, toggle2); // blink light blue if bound
      else {
#ifdef WIFIMODE
        set_LED(0, 0, toggle2); // blink blue if not bound
#else
        set_LED(0, toggle2, 0); // blink blue if not bound
#ifdef ACCESSPOINT
        set_LED(0, toggle2, !toggle2); // blink red&blue if not bound
#endif
#endif
      }
    }
  }

  // read new sensor data
  if (ReadingRoutineEnable) {
    readblockinterrupt();
  }


}

















/********************************************************************
   Function:        setup()
   Description:
 *******************************************************************/
void setup() {

  Serial.begin(115200);
  while (!Serial);

  //*******************************************************************
  // set LED pins as output
  //*******************************************************************
  pinMode(pinLEDred, OUTPUT);
  pinMode(pinLEDgreen, OUTPUT);
  pinMode(pinLEDblue, OUTPUT);
  digitalWrite(pinLEDred, HIGH);
  digitalWrite(pinLEDgreen, HIGH);
  digitalWrite(pinLEDblue, HIGH);


  //*******************************************************************
  // WIFI initialization
  //*******************************************************************
#ifdef WIFIMODE
  wifi_on = 1;
#endif

#ifdef ACCESSPOINT
  wifi_on = 2;
#endif

  if (wifi_on) {
    set_LED(0, 0, 1);
    initWIFI();
    udp.begin(localPort);
    set_LED(0, 0, 0);
  }
  //*******************************************************************
  // SPI connection
  //*******************************************************************
  SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI);
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  pinMode(SPI_CS, OUTPUT);

  //*******************************************************************
  // searching for sensor; if connected: read the whole EEPROM
  // (LEDs are blinking as long as no sensor is detected)
  //*******************************************************************
  // search sensor via ID
  Serial.print("search device... ");
  while (id == 0 || id == 0xFFFFFFFF) {
    id = read_EEPROM_byte(E_ID4) << 24 | read_EEPROM_byte(E_ID3) << 16 | read_EEPROM_byte(E_ID2) << 8 | read_EEPROM_byte(E_ID1);
    if (id > 0x00 && id < 0xFFFFFFFF) {
      Serial.println("HTPAd detected");
    }
    else {
      Serial.println("no HTPAd detected; next try in a seconds");
      delay(1000);
    }
  }
  // read the whole eeprom only once
  read_eeprom();


  //*******************************************************************
  // wake up and start the sensor
  //*******************************************************************
  // to wake up sensor set configuration register to 0x01
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x01);

  // write the calibration settings into the trim registers
  write_user_settings_to_sensor();

  // to start sensor set configuration register to 0x09
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  if (read_block_num < NUMBER_OF_BLOCKS)
    write_sensor_byte(CONFIGURATION_REGISTER, 0x09 | 0x10 * read_block_num);
  else
    write_sensor_byte(CONFIGURATION_REGISTER, 0x0B);
  Serial.println("HTPAd is ready");


  //*******************************************************************
  // timer initialization
  //*******************************************************************
  timert = calc_timert(clk_user, mbit_user);
  TimerLib.setInterval_us(ISR, timert );
  //*******************************************************************
  // print the menu for the first time
  //*******************************************************************
#ifdef SERIALMODE
  print_menu();
#endif



}



/********************************************************************
  Function:        loop()
  Description:
*******************************************************************/
void loop() {

#ifdef SERIALMODE
  checkSerial();
#endif

  if (wifi_on)
    checkUDP();



  if (state) { // state is 1 when all raw sensor voltages are read for this picture

    sort_data(); // sort the data if the reading routing is done
    state = 0; // reset the state to sample new raw pixel data

    //*******************************************************************
    // WIFI OUTPUT
    //*******************************************************************
    if (send_data) {

      // calculate the pixel data only if the GUI wants to receive thermal images in deci Kelvin
      // send_data = 1 for T-MODE / send_data = 2 for V-MODE
      /*
        if (send_data == 1) {
          calculate_pixel_temp();
        }
      */

      /*
            if (tablenumber != DevConst.TableNumber) {
              printWrongLUT();
            }
      */

      // send the thermal image as UDP packets to GUI
      sortUDPpacket();

    }
    //*******************************************************************
    // SERIAL OUTPUT
    //*******************************************************************
    else {
      // here the print functions are called for serial monitor output
      // (there was activate in checkSerial() function before)

      // print final pixel temperatures in dK
      if (print_state == 1) {
        //calculate_pixel_temp();
        //print_final_array();
        print_state = 0;
      }

      // print raw sensor RAM output
      if (print_state == 2) {
        print_RAM_array();
        print_state = 0;
      }

    }


  }

}









/********************************************************************
   Function:      set_LED()
   Description:
   Dependencies:
 *******************************************************************/
void set_LED(unsigned char red, unsigned char green, unsigned char blue) {
  digitalWrite(pinLEDred, !red);
  digitalWrite(pinLEDgreen, !green);
  digitalWrite(pinLEDblue, !blue);
}





/********************************************************************
  Function:        HSFlashStandby()
  Description:
*******************************************************************/
void HSFlashStandby(void) {

  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x10);
  delayMicroseconds(1);
  digitalWrite(SPI_CS, HIGH);
  delayMicroseconds(100);
}




/********************************************************************
  Function:        HSFlashWake()
  Description:
*******************************************************************/
void HSFlashWake(void) {

  HSFlashStandby();   //must be called upfront wake!
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x20);
  delayMicroseconds(1);
  digitalWrite(SPI_CS, HIGH);
  delayMicroseconds(20);
}


/********************************************************************
  Function:        HSF_HighDensByteRead()
  Description:
*******************************************************************/
bool HSF_HighDensByteRead(unsigned char *data1, unsigned short byteaddr, unsigned int NumBytes) {
  unsigned int ret, i = 0, k, kk;
  unsigned char *pTemp;
  signed int NumB = NumBytes;

  if (NumBytes > (HSF_NR_OF_PAGES * HSF_BYTE_PAGE_SIZE))
    return false;

  while (NumB > 0) {
    ret = HSF_Read32(byteaddr >> 2);
    pTemp = (unsigned char*)&ret;
    //erial.println(sizeof(unsigned int));
    //Serial.println(ret,HEX);
    if (!i) {
      kk = 0;
      for (k = 0; k < (byteaddr % 4); k++) {
        pTemp++;
        kk++;
      }
      for (k = kk; k < 4; k++) {
        *data1++ = *pTemp++;
        NumB--;
        if (!NumB)
          break;
      }
    }
    else if (NumB >= 4) {
      for (k = 0; k < 4; k++)
        *data1++ = *pTemp++;
      NumB -= 4;
    }
    else { //the last bytes
      kk = NumB;
      for (k = 0; k < kk; k++) {
        *data1++ = *pTemp++;
        NumB--;
        if (!NumB)
          break;
      }
    }
    byteaddr += sizeof(unsigned int);
    i++;
  }

  return true;
}



/********************************************************************
  Function:        HSF_Read32()
  Description:
*******************************************************************/
unsigned int HSF_Read32(unsigned short addr) {
  char __attribute__((unused))ErrorCode;
  unsigned int retval = 0;

  HSF_SetAddr(addr);

  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0xB0);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  delayMicroseconds(1);
  digitalWrite(SPI_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0xB1);
  retval |= SPI.transfer(0x00) << 24;
  retval |= SPI.transfer(0x00) << 16;
  retval |= SPI.transfer(0x00) << 8;
  retval |= SPI.transfer(0x00);
  delayMicroseconds(1);
  digitalWrite(SPI_CS, HIGH);
  delayMicroseconds(1);

  return retval;
}




/********************************************************************
  Function:        HSF_SetAddr()
  Description:
*******************************************************************/
void HSF_SetAddr(unsigned short addr) {

  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x90);
  SPI.transfer((addr & 0xFF00) >> 8);
  SPI.transfer((addr & 0x00FF));
  delayMicroseconds(1);
  digitalWrite(SPI_CS, HIGH);
  delayMicroseconds(1);

}




/********************************************************************
   Function:      calc_timert(uint8_t clk, uint8_t mbit)
   Description:   calculate the duration of the timer which reads the sensor blocks
 *******************************************************************/
word calc_timert(uint8_t clk, uint8_t mbit) {

  float a;
  uint16_t calculated_timer_duration;


  float Fclk_float = 5000000.0 / 63.0 * (float)clk + 500000.0;    // calc clk in Hz

  a = 0.8 * 4.0 * (float)(pow(2, (unsigned char)(mbit & 0b00001111)) + 100.0) / Fclk_float;

  calculated_timer_duration = (unsigned short)(a * 1000000); // a in s | timer_duration in µs

  return calculated_timer_duration;


}




/********************************************************************
   Function:        void readblockinterrupt()
   Description:     read one sensor block and change configuration register to next block
                    (also read electrical offset when read_eloffset_next_pic is set)
 *******************************************************************/
void readblockinterrupt() {

  unsigned char currentblock = read_block_num;

  TimerLib.clearTimer();
  unsigned char bottomblock;



  // wait for end of conversion bit (~27ms)
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }



  read_block_num++;

  // get data of top half:
  digitalWrite(SPI_CS, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(RAMoutput[currentblock], BLOCK_LENGTH); // receive data
  digitalWrite(SPI_CS, LOW);


  // get data of bottom half:
  bottomblock = (unsigned char)((unsigned char)(NUMBER_OF_BLOCKS + 1) * 2 - currentblock - 1);
  digitalWrite(SPI_CS, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  for(int i = 0; i<100000; i++);
  SPI.transfer(RAMoutput[bottomblock], BLOCK_LENGTH); // receive data
  digitalWrite(SPI_CS, LOW);


  if (read_block_num < NUMBER_OF_BLOCKS) {

    // to start sensor set configuration register to 0x09
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  x  |  x  |   1   |    0     |   0   |    1   |
    write_sensor_byte(CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x10 * read_block_num)));
  }
  else {
    //*******************************************************************
    // all blocks for the current image are sampled, now check if its time
    // to get new electrical offsets and/or for switching PTAT and VDD
    //*******************************************************************

    if (read_eloffset_next_pic) {
      read_eloffset_next_pic = 0;

      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
      write_sensor_byte(CONFIGURATION_REGISTER, (unsigned char)(0x0B));
      new_offsets = 1;
    }
    else {
      if (picnum > 1)
        state = 1; // state = 1 means that all required blocks are sampled
      picnum++; // increase the picture counter

      // check if the next sample routine should include electrical offsets
      if ((unsigned char)(picnum % READ_ELOFFSET_EVERYX) == 0)
        read_eloffset_next_pic = 1;


      if (DevConst.PTATVDDSwitch)
        switch_ptat_vdd ^= 1;


      read_block_num = 0;

      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
      write_sensor_byte(CONFIGURATION_REGISTER, (unsigned char)(0x09));
    }
  }


  TimerLib.setInterval_us(ISR, timert );


  /*
    // get data of top half:
    digitalWrite(SPI_CS, HIGH);  // set HIGH to communicate with sensor
    SPI.transfer(TOP_HALF);  // read command
    SPI.transfer(RAMoutput[currentblock], BLOCK_LENGTH); // receive data
    digitalWrite(SPI_CS, LOW);


    // get data of bottom half:
    bottomblock = (unsigned char)((unsigned char)(NUMBER_OF_BLOCKS + 1) * 2 - currentblock - 1);
    digitalWrite(SPI_CS, HIGH);  // set HIGH to communicate with sensor
    SPI.transfer(BOTTOM_HALF);  // read command
    SPI.transfer(RAMoutput[bottomblock], BLOCK_LENGTH); // receive data
    digitalWrite(SPI_CS, LOW);
  */
}


/********************************************************************
   Function:        void read_eeprom()
   Description:     read all values from eeprom
 *******************************************************************/
void read_eeprom() {
  int m = 0;
  int n = 0;
  byte b[4];
  id = read_EEPROM_byte(E_ID4) << 24 | read_EEPROM_byte(E_ID3) << 16 | read_EEPROM_byte(E_ID2) << 8 | read_EEPROM_byte(E_ID1);
  mbit_user = read_EEPROM_byte(E_MBIT_USER);
  bias_user = read_EEPROM_byte(E_BIAS_USER);
  clk_user = read_EEPROM_byte(E_CLK_USER);
  bpa_user = read_EEPROM_byte(E_BPA_USER);
  arraytype = read_EEPROM_byte(E_ARRAYTYPE);

}


/********************************************************************
   Function:        void read_EEPROM_byte(unsigned int eeaddress )
   Description:     read eeprom register as 8
   Dependencies:    register address (address)
 *******************************************************************/
byte read_EEPROM_byte(unsigned int address ) {
  byte rdata = 0xFF;

  HSF_HighDensByteRead((unsigned char*)&rdata, address, 1);

  return rdata;
}



/********************************************************************
   Function:        void read_sensor_register( uint16_t addr)
   Description:     read sensor register
   Dependencies:    register address (addr),
 *******************************************************************/
byte read_sensor_register( uint8_t addr) {
  byte rdata;

  digitalWrite(SPI_CS, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(addr);  // command
  rdata = SPI.transfer(0xFF);// end of message
  digitalWrite(SPI_CS, LOW); // set Low, back to eeprom

  return rdata;

}


/********************************************************************
   Function:        void sort_data()
   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd
 *******************************************************************/
void sort_data() {

  unsigned long sum = 0, sum2 = 0;
  unsigned short pos = 0;

  for (int m = 0; m < DevConst.RowPerBlock; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      /*
         for example: a normal line of RAMoutput for HTPAd80x64 looks like:
         RAMoutput[0][] = [ PTAT(MSB), PTAT(LSB), DATA0[MSB], DATA0[LSB], DATA1[MSB], DATA1[LSB], ... , DATA640[MSB], DATA640LSB];
                                                      |
                                                      |-- DATA_Pos = 2 (first data byte)
      */
      pos = (unsigned short)(2 * n + DevConst.DataPos + m * 2 * DevConst.PixelPerRow);



      /******************************************************************************************************************
        new PIXEL values
      ******************************************************************************************************************/
      for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
        // top half
        data_pixel[m + i * DevConst.RowPerBlock][n] =
          (unsigned short)((RAMoutput[i][pos] << 8 | RAMoutput[i][pos + 1]) << bitstoshift);
        // bottom half
        data_pixel[DevConst.PixelPerColumn + m - i * DevConst.RowPerBlock - DevConst.RowPerBlock][n] =
          (unsigned short)((RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos + 1]) << bitstoshift);
      }


      /******************************************************************************************************************
        new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
      ******************************************************************************************************************/
      if ((picnum % ELOFFSETS_BUFFER_SIZE == 1) || !eloffset[m][n]) {
        if (picnum < ELOFFSETS_FILTER_START_DELAY) {
          // top half
          eloffset[m][n] = (unsigned short)((RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]) << bitstoshift);
          // bottom half
          eloffset[m+DevConst.RowPerBlock][n] = (unsigned short)((RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]) << bitstoshift);
          use_eloffsets_buffer = 1;

        }
        else {
          // use a moving average filter
          // top half
          sum = (unsigned long)eloffset[m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)((RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]) << bitstoshift);
          eloffset[m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
          // bottom half
          sum = (unsigned long)eloffset[m+DevConst.RowPerBlock][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)((RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]) << bitstoshift);
          eloffset[m+DevConst.RowPerBlock][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
        }
      }


    }

  }



  /******************************************************************************************************************
    new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  sum = 0;
  // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
  for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
    // block top half
    sum += (unsigned short)((RAMoutput[i][DevConst.PTATPos] << 8 | RAMoutput[i][DevConst.PTATPos + 1]) << bitstoshift);
    // block bottom half
    sum += (unsigned short)((RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos + 1]) << bitstoshift);
  }
  ptat_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));


  ptat_buffer[ptat_i] = ptat_av_uint16;
  ptat_i++;
  if (ptat_i == PTAT_BUFFER_SIZE) {
    if (use_ptat_buffer == 0) {
      //Serial.print(" | PTAT buffer complete");
      use_ptat_buffer = 1;
    }
    ptat_i = 0;
  }

  if (use_ptat_buffer) {
    // now overwrite the old ptat average
    sum = 0;
    for (int i = 0; i < PTAT_BUFFER_SIZE; i++) {
      sum += ptat_buffer[i];
    }
    ptat_av_uint16 = (uint16_t)((float)sum / PTAT_BUFFER_SIZE);
  }





  /******************************************************************************************************************
    new VDD values (store them in VDD buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  sum = 0;
  // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
  for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
    // block top half
    sum += (unsigned short)((RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][DevConst.VDDPos + 1]) << bitstoshift);
    // block bottom half
    sum += (unsigned short)((RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos + 1]) << bitstoshift);
  }
  vdd_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));


  // write into vdd buffer
  vdd_buffer[vdd_i] = vdd_av_uint16;
  vdd_i++;
  if (vdd_i == VDD_BUFFER_SIZE) {
    if (use_vdd_buffer == 0) {
      //Serial.print(" | VDD buffer complete");
      use_vdd_buffer = 1;
    }
    vdd_i = 0;
  }
  if (use_vdd_buffer) {
    sum = 0;
    for (int i = 0; i < VDD_BUFFER_SIZE; i++) {
      sum += vdd_buffer[i];
    }
    // now overwrite the old vdd average
    vdd_av_uint16 = (uint16_t)((float)sum / VDD_BUFFER_SIZE);
  }


  /******************************************************************************************************************
    new ATC values (store them in VDD buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  sum = 0;
  sum2 = 0;
  for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
    // block top half
    sum += (unsigned short)((RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][DevConst.VDDPos + 1]) << bitstoshift);
    // block bottom half
    sum2 += (unsigned short)((RAMoutput[2 * DevConst.NumberOfBlocks - i][DevConst.VDDPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i][DevConst.VDDPos + 1]) << bitstoshift);
  }
  ATC0 = (unsigned short)((float)sum / (float)(DevConst.NumberOfBlocks));
  ATC1 = (unsigned short)((float)sum2 / (float)(DevConst.NumberOfBlocks));

}



/********************************************************************
   Function:        void write_sensor_byte( unsigned short addr)
   Description:     write to sensor register
   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
void write_sensor_byte( unsigned char addr, unsigned char input) {

  digitalWrite(SPI_CS, HIGH);  // set HIGH to communicate with Sensor
  SPI.transfer(addr);  // register address eeprom
  SPI.transfer(input);
  //SPI.transfer(0xFF);// end of message
  digitalWrite(SPI_CS, LOW); // set LOW, back to eeprom
}

/********************************************************************
   Function:        void write_user_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_user_settings_to_sensor() {

  write_sensor_byte(TRIM_REGISTER1, mbit_user);
  delay(5);
  write_sensor_byte(TRIM_REGISTER2, bias_user);
  delay(5);
  write_sensor_byte(TRIM_REGISTER3, bias_user);
  delay(5);
  write_sensor_byte(TRIM_REGISTER4, clk_user);
  delay(5);
  write_sensor_byte(TRIM_REGISTER5, bpa_user);
  delay(5);
  write_sensor_byte(TRIM_REGISTER6, bpa_user);

}








/********************************************************************
 ********************************************************************
    - - - PART 3: WIFI FUNCTIONS - - -
    initWIFI()
    checkUDP()
    sortUDPpacket()
 ********************************************************************
 ********************************************************************/



/********************************************************************
   Function:        initWIFI()
   Description:
 *******************************************************************/
void initWIFI(void) {


#ifdef WIFIMODE

  unsigned char ConnCounter = 0;
  while (status != WL_CONNECTED) {
    Serial.print("\nstart connection to network:");
    Serial.print(ssid);
    WiFi.mode(WIFI_STA);
    status = WiFi.begin(ssid, pass);
    delay(5000);
    //if (ConnCounter == 3)
    //  break;
    //ConnCounter++;
  }
  Serial.println("\n\nWIFI setup:");
  Serial.print("network:\t\t");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address:\t\t");
  Serial.println(WiFi.localIP());
  Serial.print("signal strength :\t");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  return;
#endif

#ifdef ACCESSPOINT

  const char* ssid1     = ACCESSPOINTNAME;
  const char* password1 = ACCESSPOINTKEY;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid1, password1);;

  Serial.println("\n\naccess point setup:");
  Serial.print("name:\t\t");
  Serial.print(ACCESSPOINTNAME);
  Serial.print("\nIP Address:\t");
  Serial.println(WiFi.softAPIP());
#endif

}







/********************************************************************
   Function:        checkUDP()
   Description:
 *******************************************************************/
void checkUDP() {
  int packetSize = udp.parsePacket();

  char packetChangeEPSILON[] = {"Set Emission to "};
  uint8_t change_ip, change_epsilon, change_id;


  if (packetSize) {

    if ( (ip_partner[0] == udp.remoteIP()[0] &&
          ip_partner[1] == udp.remoteIP()[1] &&
          ip_partner[2] == udp.remoteIP()[2] &&
          ip_partner[3] == udp.remoteIP()[3]) || device_bind == 0) {

      int len = udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
    }
    else {
      udp.read(packetBuffer, 255);
      return;
    }

    // ----------------------------------------------------
    // HTPA RESPONSED
    if (strcmp(packetBuffer, "Calling HTPA series devices") == 0) {
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("HTPA series responsed! I am Arraytype "); // 00 for 8x8
      udp.print(arraytype);
      udp.print(" MODTYPE 011\r\nADC: ");                                         //005 = 8x8; 011 = 80x64
      udp.print( (mbit_user & 15) + 4);    // calc ADC resolution
      udp.print("\r\n");
      //udp.print("HTPA32x32d v.0.01 Heimann Sensor GmbH; written by D. Pauer 2019-11-13\r\n");
      udp.print("HTPAxxxxxd v.2.00 Heimann Sensor GmbH; written by D. Pauer 2021-01-13\r\n");
      udp.print("I am running on ");
      float clk_float = 12000000 / 63 * clk_user + 1000000;    // calc clk in MHz
      udp.print(clk_float / 1000, 1); // print clk in kHz
      udp.print(" kHz\r\n");
      udp.print("MAC-ID: ");
      for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) {
          udp.print("0");
        }
        udp.print(mac[i], HEX);
        if (i < 5) {
          udp.print(".");
        }
      }
      udp.print(" IP: ");
      for (int i = 0; i < 4; i++) {

        if (WiFi.localIP()[i] < 10) {
          udp.print("00");
        }
        else if (WiFi.localIP()[i] < 100) {
          udp.print("0");
        }
        udp.print(WiFi.localIP()[i]);
        if (i < 3) {
          udp.print(".");
        }
      }
      udp.print(" DevID: ");
      for (int i = 1; i < 9; i++) {
        if (id < pow(10, i)) {
          udp.print("0");
        }
      }
      udp.print(id);
      udp.endPacket();

    }

    // ----------------------------------------------------
    // SEND IP AND MAC (HW FILTER)
    if (strcmp(packetBuffer, "Bind HTPA series device") == 0) {
      if (device_bind == 0) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print("HW Filter is ");
        for (int i = 0; i < 4; i++) {

          if (WiFi.localIP()[i] < 10) {
            udp.print("00");
          }
          else if (WiFi.localIP()[i] < 100) {
            udp.print("0");
          }
          udp.print(WiFi.localIP()[i]);
          if (i < 3) {
            udp.print(".");
          }
        }
        udp.print(" MAC ");
        for (int i = 0; i < 6; i++) {
          if (mac[i] < 0x10) {
            udp.print("0");
          }
          udp.print(mac[i], HEX);
          if (i < 5) {
            udp.print(".");
          }
        }
        udp.print("\n\r");
        udp.endPacket();

        device_bind = 1;
        for (int i = 0; i < 4; i++) {
          ip_partner[i] = udp.remoteIP()[i];
        }

      }
      else {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print("Device already bound\n\r");
        udp.endPacket();
      }
    }


    if (device_bind) {
      // ----------------------------------------------------
      //USER SETTING
      if (strcmp(packetBuffer, "G") == 0) {
        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("HTPAxxxxxd 2021/01/13 v.2.00 Heimann Sensor GmbH; written by D. Pauer\n\r");
        udp.print("BIAS: ");
        if (bias_user < 0x10)
          udp.print("0");
        udp.print(bias_user, HEX);
        udp.print("Clock: ");
        if (clk_user < 0x10)
          udp.print("0");
        udp.print(clk_user, HEX);
        udp.print("MBIT: ");
        if (mbit_user < 0x10)
          udp.print("0");
        udp.print(mbit_user, HEX);
        udp.print("BPA: ");
        if (bpa_user < 0x10)
          udp.print("0");
        udp.print(bpa_user, HEX);
        udp.print("PU: ");
        unsigned char pu_user = 0;
        if (pu_user < 0x10)
          udp.print("0");
        udp.print(pu_user, HEX);
        udp.print("GlobalOffset: ");
        udp.print(globaloff, HEX);
        udp.print("GlobalGain: ");
        udp.print(globalgain, HEX);

        udp.endPacket();

      }

      // SEND DATA (TEMPS)
      if (strcmp(packetBuffer, "K") == 0) {
        TimerLib.clearTimer();
        write_user_settings_to_sensor();
        timert = calc_timert(clk_user, mbit_user);
        TimerLib.setInterval_us(ISR, timert );
        send_data = 1;
      }

      // SEND DATA (TEMPS)
      if (strcmp(packetBuffer, "t") == 0) {
        TimerLib.clearTimer();
        write_user_settings_to_sensor();
        timert = calc_timert(clk_user, mbit_user);
        TimerLib.setInterval_us(ISR, timert );
        send_data = 2;
      }

      // ----------------------------------------------------
      // STOP SENDING
      if (strcmp(packetBuffer, "x") == 0) {
        send_data = 0;
      }


      // ----------------------------------------------------
      //HW RELEASED
      if (strcmp(packetBuffer, "x Release HTPA series device") == 0) {
        send_data = 0;
        device_bind = 0;
        for (int i = 0; i < 4; i++) {
          ip_partner[i] = 0;
        }
        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("HW-Filter released\r\n");
        udp.endPacket();
      }


      // ----------------------------------------------------
      // DECREAS/INCREASE CLK
      if (strcmp(packetBuffer, "a") == 0 || strcmp(packetBuffer, "A") == 0 ) {

        TimerLib.clearTimer();
        send_data = 0;

        if (strcmp(packetBuffer, "a") == 0  && clk_user > 0)
          clk_user--;

        if (strcmp(packetBuffer, "A") == 0 && clk_user < 63)
          clk_user++;

        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("MHZClk is ");
        float clk_float = 12000000 / 63 * clk_user + 1000000;    // calc clk in MHz

        udp.print(clk_float / 1000, 1);
        udp.print(" kHz\r\n");
        udp.endPacket();

        TimerLib.setInterval_us(ISR, timert );
      }
      // ----------------------------------------------------
      // DECREAS/INCREASE BIAS
      if (strcmp(packetBuffer, "i") == 0 || strcmp(packetBuffer, "I") == 0 ) {

        TimerLib.clearTimer();
        send_data = 0;

        if (strcmp(packetBuffer, "i") == 0 && bias_user > 0)
          bias_user--;
        if (strcmp(packetBuffer, "I") == 0 && bias_user < 31)
          bias_user++;

        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("BIAS-Trim: ");
        udp.print(bias_user, HEX);
        udp.print("\r\n");
        udp.endPacket();

        TimerLib.setInterval_us(ISR, timert );
      }

      // ----------------------------------------------------
      // DECREAS/INCREASE BPA
      if (strcmp(packetBuffer, "j") == 0 || strcmp(packetBuffer, "J") == 0 ) {

        TimerLib.clearTimer();
        send_data = 0;

        if (strcmp(packetBuffer, "j") == 0 && bpa_user > 0)
          bpa_user--;
        if (strcmp(packetBuffer, "J") == 0 && bpa_user < 31)
          bpa_user++;

        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("BPA-Trim: ");
        udp.print(bpa_user, HEX);
        udp.print("\r\n");
        udp.endPacket();


        TimerLib.setInterval_us(ISR, timert );
      }


      // ----------------------------------------------------
      // DECREAS/INCREASE MBIT
      if (strcmp(packetBuffer, "r") == 0 ||
          strcmp(packetBuffer, "R") == 0 ||
          strcmp(packetBuffer, "o") == 0 ||
          strcmp(packetBuffer, "O") == 0 ) {

        TimerLib.clearTimer();
        send_data = 0;

        uint8_t adc_res, adc_ref;
        adc_res = mbit_user & 15;
        adc_ref = (mbit_user & 48) >> 4;

        if (strcmp(packetBuffer, "r") == 0 && adc_res > 4)
          adc_res--;
        if (strcmp(packetBuffer, "R") == 0 && adc_res < 12)
          adc_res++;
        if (strcmp(packetBuffer, "o") == 0 && adc_ref > 0)
          adc_ref--;
        if (strcmp(packetBuffer, "O") == 0 && adc_ref < 3)
          adc_ref++;

        mbit_user = adc_ref << 4 | adc_res;

        if (strcmp(packetBuffer, "r") == 0 ||
            strcmp(packetBuffer, "R") == 0 ) {
          udp.beginPacket(ip_partner, udp.remotePort());
          udp.print("Resolution: ");
          if (adc_res + 4 < 10) {
            udp.print("0");
          }
          udp.print(adc_res + 4);
          udp.print(" bit\r\n");
          udp.endPacket();
        }

        if (strcmp(packetBuffer, "o") == 0 ||
            strcmp(packetBuffer, "O") == 0 ) {
          udp.beginPacket(ip_partner, udp.remotePort());
          udp.print("ADC-Ref: ");
          if (adc_ref < 10) {
            udp.print("0");
          }
          udp.print(adc_ref);
          udp.print(" bit\r\n");
          udp.endPacket();
        }

        TimerLib.setInterval_us(ISR, timert );
      }


      // ----------------------------------------------------
      // GET BCC
      if (strcmp(packetBuffer, "h") == 0) {
        send_data = 0;
        TimerLib.clearTimer();
        uint8_t packet1[1000];
        unsigned short bytei = 0;
        for (int i = 0; i < (unsigned short)EEPROM_SIZE; i++) {
          packet1[bytei++] = read_EEPROM_byte(i);
          if (bytei == 1000) {
            udp.beginPacket(ip_partner, udp.remotePort());
            udp.write(packet1, 1000);
            udp.endPacket();
            bytei = 0;
          }
        }
        udp.beginPacket(ip_partner, udp.remotePort());
        udp.write(packet1, bytei);
        udp.endPacket();

        TimerLib.setInterval_us(ISR, timert );
      }

      // ----------------------------------------------------

    }

  }

}


/********************************************************************
   Function:        sortUDPpacket()
   Description:
 *******************************************************************/
void sortUDPpacket() {

  uint8_t packet1[UDP_PACKET_LENGTH];
  int bytei = 1, packetnumber = 0;

  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      packet1[bytei++] = (data_pixel[m][n] & 0x00ff);
      packet1[bytei++] = (data_pixel[m][n] & 0xff00) >> 8;

      if (bytei == UDP_PACKET_LENGTH) {
        packet1[0] = packetnumber + 1;
        udp.beginPacket(ip_partner, udp.remotePort());
        udp.write(packet1, UDP_PACKET_LENGTH);
        udp.endPacket();
        bytei = 1;
        packetnumber++;
      }

    }
  }


  for (int m = 0; m < ROW_PER_BLOCK * 2; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      packet1[bytei++] = (eloffset[m][n] & 0x00ff);
      packet1[bytei++] = (eloffset[m][n] & 0xff00) >> 8;

      if (bytei == UDP_PACKET_LENGTH) {
        packet1[0] = packetnumber + 1;
        udp.beginPacket(ip_partner, udp.remotePort());
        udp.write(packet1, UDP_PACKET_LENGTH);
        udp.endPacket();
        bytei = 1;
        packetnumber++;
      }

    }
  }

  packet1[bytei++] = (vdd_av_uint16 & 0x00ff);
  packet1[bytei++] = (vdd_av_uint16 & 0xff00) >> 8;

  packet1[bytei++] = (Ta & 0x00ff);
  packet1[bytei++] = (Ta & 0xff00) >> 8;

  for (int i = 0; i < NUMBER_OF_BLOCKS * 2; i++) {
    packet1[bytei++] = (ptat_av_uint16 & 0x00ff);
    packet1[bytei++] = (ptat_av_uint16 & 0xff00) >> 8;
  }

  packet1[bytei++] = (ATC0 & 0x00ff);
  packet1[bytei++] = (ATC0 & 0xff00) >> 8;

  packet1[bytei++] = (ATC1 & 0x00ff);
  packet1[bytei++] = (ATC1 & 0xff00) >> 8;

  packet1[0] = packetnumber + 1;
  udp.beginPacket(ip_partner, udp.remotePort());
  udp.write(packet1, LAST_UDP_PACKET_LENGTH);
  udp.endPacket();
}











/********************************************************************
   Function:        print_final_array()
   Description:
 *******************************************************************/
void print_final_array(void) {
  Serial.println("\n\n---pixel data ---");
  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.println("");
  }
}

/********************************************************************
   Function:        print_RAM_array()
   Description:
 *******************************************************************/
void print_RAM_array(void) {
  Serial.print("\n\n\n---pixel data ---\n");
  for (int m = 0; m < (2 * NUMBER_OF_BLOCKS + 2); m++) {
    for (int n = 0; n < BLOCK_LENGTH; n++) {
      Serial.print(RAMoutput[m][n], HEX);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
  Serial.print("\n\n\n");
}

/********************************************************************
   Function:        checkSerial()
   Description:
 *******************************************************************/
void checkSerial() {

  serial_input = Serial.read();

  switch (serial_input) {
    case 0xFF:
      //nothing
      break;

    case 'a':
      if (send_data)
        Serial.println("stop data stream in GUI before");
      else
        print_state = 1;
      break;

    case 'b':
      if (send_data)
        Serial.println("stop data stream in GUI before");
      else
        print_state = 2;
      break;

    case 'm':
      while (state);
      ReadingRoutineEnable = 0;
      print_menu();
      ReadingRoutineEnable = 1;
      break;


  }


}


/********************************************************************
   Function:      print_menu()
   Description:
 *******************************************************************/
void print_menu() {
  Serial.println("\n\n\n***************************************************");
  Serial.println("Application Shield v2.2                 /_/eimann");
  Serial.println("for ESP32-DevkitC-32D                  / /   Sensor");

  Serial.println("\nYou can choose one of these options by sending the \ncharacter\n ");
  Serial.println("read SENSOR values:");
  Serial.println("  a... pixel data in array format (in digit)");
  Serial.println("  b... show all raw values (in digits)");
  Serial.println("***************************************************\n\n\n");
}
