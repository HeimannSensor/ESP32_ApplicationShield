/*** PROGRAMM INFO***************************************************************************************
  source code for ESP32 and HTPAd Application Shield
  name:           ESP32_HTPAd_16x16.ino
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
//#define WIFIMODE
#define SERIALMODE
#define ACCESSPOINT // automatically diablead if WIFIMODE is active


/*** NETWORK INFORMATION ********************************************************************************
  If you want to use the WIFI function, you have to change ssid and pass.
*********************************************************************************************************/
char ssid[] = "WifiAppsetNetwork";
char pass[] = "heimannsensor";

#include "Wire.h"
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
unsigned char mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, mbit_user, bias_user, clk_user, bpa_user, pu_user;
unsigned char gradscale, epsilon, lastepsilon, arraytype;
signed char globaloff;
signed short thgrad[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
unsigned short tablenumber, ptatth1, ptatth2, ptatgr, globalgain;
signed short thoffset[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
unsigned long id, ptatoff;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;
uint16_t pij[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
uint32_t pixcij_uint32[PIXEL_PER_COLUMN][PIXEL_PER_ROW];


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
unsigned short Ta, ptat_av_uint16;


// BUFFER for PTAT,VDD and elOffsets
// PTAT:
uint16_t ptat_buffer[PTAT_BUFFER_SIZE];
uint16_t ptat_buffer_average;
uint8_t use_ptat_buffer = 0;
uint8_t ptat_i = 0;
uint8_t PTATok = 0;
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
uint32_t gradscale_div;
unsigned long t1;
unsigned char print_state = 0;

//LEDs
unsigned char pinLEDred = 33;
unsigned char pinLEDgreen = 25;
unsigned char pinLEDblue = 32;

unsigned NewDataAvailable = 1;

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
  if (read_eloffset_next_pic == 1) {
    static bool toggle2 = false;
    toggle2 = !toggle2;

    if (send_data) {
      toggle2 != toggle2;

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
    /*
       HINT:
       this interrupt service routine set a flag called NedDataAvailable.
       This flag will be checked in the main loop. If this flag is set, the main loop will call
       the function to read the new sensor data and reset this flag and the timer. I go that way
       because the ESP32 cannot read I2C data directly in the ISR. If your µC can handle I2C in
       an interrupt,please read the new sensor volatges direclty in the ISR.
    */
    NewDataAvailable = 1;
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
  // searching for sensor; if connected: read the whole EEPROM
  //*******************************************************************
  uint8_t error;
  while (error != 0) {
    delay(2000);
    Wire.begin();
    Wire.beginTransmission(SENSOR_ADDRESS);
    error = Wire.endTransmission();
  }

  Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
  read_complete_eeprom();
  Wire.setClock(CLOCK_SENSOR);  // I2C clock frequency (for sensor communication)

  //*******************************************************************
  // wake up and start the sensor
  //*******************************************************************
  // to wake up sensor set configuration register to 0x01
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);

  // write the calibration settings into the trim registers
  write_calibration_settings_to_sensor();

  // to start sensor set configuration register to 0x09
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  //write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);
  Serial.println("HTPAd is ready");

  //*******************************************************************
  // do bigger calculation here before you jump into the loop() function
  //*******************************************************************
  gradscale_div = pow(2, gradscale);
  calcPixC(); // calculate the pixel constants

  //*******************************************************************
  // timer initialization
  //*******************************************************************
  timert = calc_timert(clk_calib, mbit_calib);
  TimerLib.setInterval_us(ISR, timert);

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


  /* check if the timer interrupt set the NewDataAvailable flag. If so
     read the new raw pixel data via I2C */
  if (NewDataAvailable) {
    readblockinterrupt();
    NewDataAvailable = 0;
  }



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
      if (send_data == 1) {
        calculate_pixel_temp();
      }

      if (tablenumber != DevConst.TableNumber) {
        printWrongLUT();
      }

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
        calculate_pixel_temp();
        print_final_array();
        print_state = 0;
      }

      // print raw sensor RAM output
      if (print_state == 2) {
        print_RAM_array();
        print_state = 0;
      }

      // print all calculation steps
      if (print_state == 3) {
        print_calc_steps2();
        print_state = 0;
      }
    }


  }


}


/********************************************************************
  Function:        printWrongLUT()
  Description:
*******************************************************************/
void printWrongLUT() {

  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < 8; n++) {
      if (LUTshape[m][n] == 1)
        data_pixel[m][n] = 2732;
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
 ********************************************************************
    - - - PART 2: HTPAd FUNCTIONS - - -
    calcPixC()
    calculate_pixel_temp()
    pixel_masking()
    readblockinterrupt()
    read_eeprom()
    read_EEPROM_byte( uint8_t addr)
    read_sensor_register()
    sort_data()
    write_calibration_settings_to_sensor()
    write_sensor_byte( unsigned char addr, unsigned char input)
 ********************************************************************
 ********************************************************************/



/********************************************************************
   Function:      calcPixC
   Description:   calculates the pixel constants with the unscaled
                  values from EEPROM
 *******************************************************************/
void calcPixC() {

  /* uses the formula from datasheet:

                     PixC_uns[m][n]*(PixCmax-PixCmin)               epsilon   GlobalGain
      PixC[m][n] = ( -------------------------------- + PixCmin ) * ------- * ----------
                                  65535                               100        1000
  */

  double pixcij;

  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      pixcij = (double)pixcmax;
      pixcij -= (double)pixcmin;
      pixcij /= (double)65535.0;
      pixcij *= (double)pij[m][n];
      pixcij += (double)pixcmin;
      pixcij /= (double)100.0;
      pixcij *= (double)epsilon;
      pixcij /= (double)10000.0;
      pixcij *= (double)globalgain;
      pixcij += 0.5;

      pixcij_uint32[m][n] = (unsigned long)pixcij;

    }
  }


}


/********************************************************************
   Function:        calculate_pixel_temp()
   Description:     compensate thermal, electrical offset and vdd and multiply sensitivity coeff
                    look for the correct temp in lookup table
 *******************************************************************/
void calculate_pixel_temp() {

  int64_t vij_pixc_and_pcscaleval;
  int64_t pixcij;
  int64_t vdd_calc_steps;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;
  signed long pixel;


  /******************************************************************************************************************
    step 0: find column of lookup table
  ******************************************************************************************************************/
  for (int i = 0; i < NROFTAELEMENTS; i++) {
    if (Ta > XTATemps[i]) {
      table_col = i;
    }
  }
  dta = Ta - XTATemps[table_col];
  ydist = (int32_t)ADEQUIDISTANCE;


  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      /******************************************************************************************************************
         step 1: use a variable with bigger data format for the compensation steps
       ******************************************************************************************************************/
      pixel = (signed long) data_pixel[m][n];

      /******************************************************************************************************************
         step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
       ******************************************************************************************************************/
      pixel -= (int32_t)(((int32_t)thgrad[m][n] * (int32_t)Ta) / (int32_t)gradscale_div);
      pixel -= (int32_t)thoffset[m][n];

      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      pixel -= eloffset[m][n];

      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel =  (int32_t)(vij_pixc_and_pcscaleval / pixcij_uint32[m][n]);
      /******************************************************************************************************************
         step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
       ******************************************************************************************************************/
      table_row = pixel + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      pixel = (uint32_t)((vy - vx) * ((int32_t)(pixel + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      data_pixel[m][n] = (unsigned short)pixel;

    }
  }


}


/********************************************************************
   Function:      calc_timert(uint8_t clk, uint8_t mbit)
   Description:   calculate the duration of the timer which reads the sensor blocks
 *******************************************************************/
word calc_timert(uint8_t clk, uint8_t mbit) {

  float a;
  uint16_t calculated_timer_duration;

  float Fclk_float = 12000000.0 / 63.0 * (float)clk + 1000000.0;    // calc clk in Hz
  a = 32.0 * ((float)pow(2, (unsigned char)(mbit & 0b00001111)) + 4.0) / Fclk_float;

  calculated_timer_duration = (unsigned short)(0.98 * a * 1000000); // c in s | timer_duration in µs
  return calculated_timer_duration;
}






/********************************************************************
   Function:        void readblockinterrupt()
   Description:     read one sensor block and change configuration register to next block
                    (also read electrical offset when read_eloffset_next_pic is set)
 *******************************************************************/
void readblockinterrupt() {

  ReadingRoutineEnable = 0;
  TimerLib.clearTimer();

  // wait for end of conversion bit (~27ms)
  // check EOC bit
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&RAMoutput[read_block_num], BLOCK_LENGTH);


  read_block_num++;

  if (read_block_num < NUMBER_OF_BLOCKS) {

    // to start sensor set configuration register to 0x09
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  x  |  x  |   1   |    0     |   0   |    1   |
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x10 * read_block_num) + (0x04 * switch_ptat_vdd)));
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
      write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (unsigned char)(0x0B ));
      new_offsets = 1;
    }
    else {
      if (picnum > 1)
        state = 1; // state = 1 means that all required blocks are sampled
      picnum++; // increase the picture counter
      // check if the next sample routine should include electrical offsets
      if ((unsigned char)(picnum % READ_ELOFFSET_EVERYX) == 0)
        read_eloffset_next_pic = 1;


      read_block_num = 0;

      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
      write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (unsigned char)(0x09));
    }
  }

  TimerLib.setInterval_us(ISR, timert);
  ReadingRoutineEnable = 1;

}


/********************************************************************
   Function:        read_complete_eeprom();
   Description:     read complete eeprom and save in global variables
 *******************************************************************/
void read_complete_eeprom() {

  uint16_t b[2];
  id = eeprom_read_routine(E_ID2) << 16 | eeprom_read_routine(E_ID1);
  mbit_calib = (uint8_t)eeprom_read_routine(E_MBIT_CALIB);
  bias_calib = (uint8_t)eeprom_read_routine(E_BIAS_CALIB);
  clk_calib = (uint8_t)eeprom_read_routine(E_CLK_CALIB);
  bpa_calib = (uint8_t)eeprom_read_routine(E_BPA_CALIB);
  pu_calib = (uint8_t)eeprom_read_routine(E_PU_CALIB);
  mbit_user = (uint8_t)eeprom_read_routine(E_MBIT_USER);
  bias_user = (uint8_t)eeprom_read_routine(E_BIAS_USER);
  clk_user = (uint8_t)eeprom_read_routine(E_CLK_USER);
  bpa_user = (uint8_t)eeprom_read_routine(E_BPA_USER);
  pu_user = (uint8_t)eeprom_read_routine(E_PU_USER);
  tablenumber = (uint8_t)eeprom_read_routine(E_TABLENUMBER);
  gradscale = (uint8_t)eeprom_read_routine(E_GRADSCALE);
  epsilon = (uint8_t)eeprom_read_routine(E_EPSILON);
  ptatth1 = eeprom_read_routine(E_PTATTH1);
  ptatth2 = eeprom_read_routine(E_PTATTH2);
  globalgain = eeprom_read_routine(E_GLOBALGAIN);
  b[0] =  eeprom_read_routine(E_PTATGR_1);
  b[1] =  eeprom_read_routine(E_PTATGR_2);
  ptatgr_float = *(float*)b;
  b[0] =  eeprom_read_routine(E_PTATOFF_1);
  b[1] =  eeprom_read_routine(E_PTATOFF_2);
  ptatoff_float = *(float*)b;
  b[0] =  eeprom_read_routine(E_PIXCMIN_1);
  b[1] =  eeprom_read_routine(E_PIXCMIN_2);
  pixcmin = *(float*)b;
  b[0] =  eeprom_read_routine(E_PIXCMAX_1);
  b[1] =  eeprom_read_routine(E_PIXCMAX_2);
  pixcmax = *(float*)b;


  // read the bigger arrays
  for (int m = 0; m < (PIXEL_PER_COLUMN); m++) {
    for (int n = 0; n < PIXEL_PER_ROW; n++) {
      thgrad[m][n] = eeprom_read_routine(E_THGRAD + n + m * 8);
      thoffset[m][n] = eeprom_read_routine(E_THOFFSET + n + m * 8);
      pij[m][n] = eeprom_read_routine(E_PIJ + n + m * 8);
    }
  }

}



/********************************************************************
   Function:        eeprom_read_routine(uint16_t addr)

   Description:     read eeprom register (see datasheet, chp.: 10.10 I2C Example Sequence – EEPROM Sequential Read )

   Dependencies:    addr... eeprom register adress
 *******************************************************************/
word eeprom_read_routine(uint16_t addr) {

  // EEPROM_ACTIVE
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write(ACTIVE);
  Wire.endTransmission();

  // SET_ADDR
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(SET_ADDRESS));
  Wire.write((int)(addr));
  Wire.endTransmission();

  // NORMAL_READ
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(NORMAL_READ));
  Wire.endTransmission();

  // GET_DATA
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(GET_DATA));
  Wire.endTransmission(false);

  unsigned char c[2];
  Wire.requestFrom(EEPROM_ADDRESS, 2, 1);
  while (Wire.available()) { // peripheral may send less than requested
    c[0] = Wire.read(); // receive a byte as character
    c[1] = Wire.read(); // receive a byte as character
  }
  return (unsigned short)(c[1] << 8 | c[0]);

}


/********************************************************************
   Function:        void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n)
   Description:     read sensor register
 *******************************************************************/
void read_sensor_register(uint16_t addr, uint8_t *dest, uint16_t n)
{
  uint8_t sizeadr = 1;
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write((uint8_t)(addr));
  Wire.endTransmission(false);

  Wire.requestFrom((uint16_t)SENSOR_ADDRESS, n, true);
  while (Wire.available()) *dest++ = Wire.read();
}


/********************************************************************
   Function:        void sort_data()
   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd
 *******************************************************************/
void sort_data() {

  unsigned long sum = 0;
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
        data_pixel[m + i * DevConst.RowPerBlock][n] =
          (unsigned short)(RAMoutput[i][pos] << 8 | RAMoutput[i][pos + 1]);
      }


      /******************************************************************************************************************
        new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
      ******************************************************************************************************************/
      if (picnum % ELOFFSETS_BUFFER_SIZE == 1) {
        if ((!eloffset[m][n]) || (picnum < ELOFFSETS_FILTER_START_DELAY)) {
          eloffset[m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
        }
        else {
          // use a moving average filter
          sum = (unsigned long)eloffset[m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          eloffset[m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
        }
      }

    }

  }



  /******************************************************************************************************************
    new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  ptat_av_uint16 = (unsigned short)(RAMoutput[0][DevConst.PTATPos] << 8 | RAMoutput[0][DevConst.PTATPos + 1]);
  Ta = (unsigned short)((unsigned short)ptat_av_uint16 * (float)ptatgr_float + (float)ptatoff_float);

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

}


/********************************************************************
   Function:        void write_calibration_settings_to_sensor()
   Description:     write calibration data (from eeprom) to trim registers (sensor)
 *******************************************************************/
void write_calibration_settings_to_sensor() {

  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_calib);
  delay(5);
}




/********************************************************************
   Function:        write_eeprom_routine(uint16_t addr, uint16_t value)
   Description:
 *******************************************************************/
word write_eeprom_routine(uint16_t addr, uint16_t value) {

  // EEPROM_ACTIVE
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write(ACTIVE);
  Wire.endTransmission();
  delay(6);


  // SET_ADDR
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(SET_ADDRESS));
  Wire.write((int)(addr));
  Wire.endTransmission();

  // NORMAL_ERASE
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(NORMAL_ERASE));
  Wire.endTransmission();
  delay(6);

  // SET_DATA
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(SET_DATA));
  Wire.write((int)(value & 0xFF)); // LSB
  Wire.write((int)(value >> 8));  // MSB
  Wire.endTransmission();

  // NORMAL_WRITE
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(NORMAL_WRITE));
  Wire.endTransmission();
  delay(6);


}




/********************************************************************
   Function:        void write_sensor_byte( unsigned short addr)
   Description:     write to sensor register
   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
byte write_sensor_byte(uint8_t deviceaddress, uint8_t registeraddress, uint8_t input) {

  Wire.beginTransmission(deviceaddress);
  Wire.write(registeraddress);
  Wire.write(input);
  Wire.endTransmission();

}



/********************************************************************
   Function:        void write_user_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_user_settings_to_sensor() {

  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, pu_user);

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
    arraytype = 1;
    if (strcmp(packetBuffer, "Calling HTPA series devices") == 0) {
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("HTPA series responsed! I am Arraytype 00"); // 00 for 8x8
      //udp.print(arraytype);
      udp.print(" MODTYPE 005\r\nADC: ");                                         //005 = 8x8; 011 = 80x64
      udp.print( (mbit_calib & 15) + 4);    // calc ADC resolution
      udp.print("\r\n");
      udp.print("HTPA8x8d v.2.00 Heimann Sensor GmbH; written by D. Pauer 2021-01-13\r\n");
      udp.print("I am running on ");
      float clk_float = 12000000 / 63 * clk_calib + 1000000;    // calc clk in MHz
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
        udp.print("HTPA8x8d 2021/01/13 v.2.00 Heimann Sensor GmbH; written by D. Pauer\n\r");
        udp.print("BIAS: ");
        if (bias_user < 0x10)
          udp.print("0");
        udp.print(bias_user, HEX);
        udp.print("Clock: ");
        if (clk_calib < 0x10)
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
        write_calibration_settings_to_sensor();
        timert = calc_timert(clk_calib, mbit_calib);
        TimerLib.setInterval_us(ISR, timert);
        send_data = 1;
      }

      // SEND DATA (TEMPS)
      if (strcmp(packetBuffer, "t") == 0) {
        TimerLib.clearTimer();
        write_user_settings_to_sensor();
        timert = calc_timert(clk_user, mbit_user);
        TimerLib.setInterval_us(ISR, timert);
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


        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_eeprom_routine(E_CLK_USER, clk_user);
        Wire.setClock(CLOCK_SENSOR);
        delay(5);

        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("MHZClk is ");
        float clk_float = 12000000 / 63 * clk_user + 1000000;    // calc clk in MHz

        udp.print(clk_float / 1000, 1);
        udp.print(" kHz\r\n");
        udp.endPacket();

        TimerLib.setInterval_us(ISR, timert);
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


        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_eeprom_routine(E_BIAS_USER, bias_user);
        Wire.setClock(CLOCK_SENSOR);
        delay(5);

        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("BIAS-Trim: ");
        udp.print(bias_user, HEX);
        udp.print("\r\n");
        udp.endPacket();

        TimerLib.setInterval_us(ISR, timert);
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


        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_eeprom_routine(E_BPA_USER, bpa_user);
        Wire.setClock(CLOCK_SENSOR);
        delay(5);

        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("BPA-Trim: ");
        udp.print(bpa_user, HEX);
        udp.print("\r\n");
        udp.endPacket();


        TimerLib.setInterval_us(ISR, timert);
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


        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_eeprom_routine(E_MBIT_USER, mbit_user);
        Wire.setClock(CLOCK_SENSOR);
        delay(5);

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

        TimerLib.setInterval_us(ISR, timert);
      }

      // ----------------------------------------------------
      // DECREAS/INCREASE PU
      if (strcmp(packetBuffer, "p") == 0) {

        TimerLib.clearTimer();
        send_data = 0;

        if (pu_user == 17)
          pu_user = 34;
        else if (pu_user == 34)
          pu_user = 68;
        else if (pu_user == 68)
          pu_user = 136;
        else if (pu_user == 136)
          pu_user = 17;
        udp.beginPacket(ip_partner, udp.remotePort());
        udp.print("PU-Trim: ");
        udp.print(pu_user, HEX);
        udp.print("\r\n");
        udp.endPacket();


        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_eeprom_routine(E_PU_USER, pu_user);
        Wire.setClock(CLOCK_SENSOR);

        TimerLib.setInterval_us(ISR, timert);
      }

      // ----------------------------------------------------
      // GET BCC
      if (strcmp(packetBuffer, "h") == 0) {
        send_data = 0;
        TimerLib.clearTimer();
        uint8_t packet1[1000];
        unsigned short bytei = 0;
        unsigned short eecontent;
        for (int i = 0; i < (unsigned short)(EEPROM_SIZE); i++) {
          eecontent = (unsigned short)eeprom_read_routine(i);
          packet1[bytei++] = (unsigned char)(eecontent & 0x00FF);
          packet1[bytei++] = (unsigned char)((eecontent & 0xFF00) >> 8);
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
      // CHANGE EPSILON
      if (packetSize > 16) {
        change_epsilon = 1;
        // compare the first position of string
        for (int i = 0; i < 16; i++) {
          if (packetBuffer[i] != packetChangeEPSILON[i]) {
            change_epsilon = 0;
          }
        }
        if (change_epsilon) {
          send_data = 0;
          TimerLib.clearTimer();
          epsilon = (int)(packetBuffer[16] - '0') * 100 + (int)(packetBuffer[17] - '0') * 10 + (int)(packetBuffer[18] - '0');

          // write new epsilon to eeprom
          Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
          write_eeprom_routine(E_EPSILON, epsilon);
          Wire.setClock(CLOCK_SENSOR);

          // calculate pixcij with new epsilon
          calcPixC();

          udp.beginPacket(ip_partner, udp.remotePort());
          udp.print("Emission changed to ");
          udp.print(epsilon);
          udp.print("%\r\n\r\n");
          udp.endPacket();
          delay(1000);
          TimerLib.setInterval_us(ISR, timert);
        }
      }




    }

  }

}


/********************************************************************
   Function:        sortUDPpacket()
   Description:
 *******************************************************************/
void sortUDPpacket() {

  uint8_t packet1[UDP_PACKET_LENGTH];
  int bytei = 0, packetnumber = 0;

  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {
      packet1[bytei++] = (data_pixel[m][n] & 0x00ff);
      packet1[bytei++] = (data_pixel[m][n] & 0xff00) >> 8;
    }
  }

  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {
      packet1[bytei++] = (eloffset[m][n] & 0x00ff);
      packet1[bytei++] = (eloffset[m][n] & 0xff00) >> 8;
    }
  }
  packet1[bytei++] = (ptat_av_uint16 & 0x00ff);
  packet1[bytei++] = (ptat_av_uint16 & 0xff00) >> 8;

  packet1[bytei++] = (Ta & 0x00ff);
  packet1[bytei++] = (Ta & 0xff00) >> 8;

  packet1[bytei++] = (ptat_av_uint16 & 0x00ff);
  packet1[bytei++] = (ptat_av_uint16 & 0xff00) >> 8;

  udp.beginPacket(ip_partner, udp.remotePort());
  udp.write(packet1, UDP_PACKET_LENGTH);
  udp.endPacket();
}


















/********************************************************************
 ********************************************************************
    - - - PART 4: SERIAL FUNCTIONS - - -
  checkSerial()
  print_eeprom_header()
  print_eeprom_hex()
  print_menu()
 ********************************************************************
 ********************************************************************/



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

    case 'c':
      if (send_data)
        Serial.println("stop data stream in GUI before");
      else
        print_state = 3;
      break;


    case 'm':
      while (state);
      ReadingRoutineEnable = 0;
      print_menu();
      ReadingRoutineEnable = 1;
      break;

    case 'd':
      while (state);
      ReadingRoutineEnable = 0;
      print_eeprom_hex();
      ReadingRoutineEnable = 1;
      break;

    case 'e':
      while (state);
      ReadingRoutineEnable = 0;
      print_eeprom_header();
      ReadingRoutineEnable = 1;
      break;

    case 'h':
      while (state);
      ReadingRoutineEnable = 0;
      Serial.print("\n\n\n---ThGrad---\n");
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(thgrad[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;



    case 'i':
      while (state);
      ReadingRoutineEnable = 0;
      // print ThOffset in serial monitor
      Serial.print("\n\n\n---ThOffset---\n");
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(thoffset[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;

    case 'j':
      while (state);
      ReadingRoutineEnable = 0;
      // print PixC in serial monitor
      Serial.print("\n\n\n---PixC---\n");
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(pixcij_uint32[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;

    case 'k':
      ReadingRoutineEnable = 0;
      TimerLib.clearTimer();
      Serial.println("\n\n\n---Increase emissivity---");
      Serial.print("old emissivity: \t");
      Serial.println(epsilon);
      Serial.print("new emissivity: \t");
      if (epsilon < 100) {
        epsilon++;
        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_eeprom_routine(E_EPSILON, epsilon);
        Wire.setClock(CLOCK_SENSOR);

        // calculate pixcij with new epsilon
        calcPixC();
        Serial.print(epsilon);
        Serial.println(" (new emissivity is stored in the EEPROM now)");
      }
      else {
        Serial.print(epsilon);
        Serial.println(" (you cannot set the emissivity higher than 100%)");
      }
      delay(1000);
      TimerLib.setInterval_us(ISR, timert);
      ReadingRoutineEnable = 1;
      break;

    case 'l':
      ReadingRoutineEnable = 0;
      TimerLib.clearTimer();
      Serial.print("\n\n\n---Decrease emissivity---");
      Serial.print("\nold emissivity: \t");
      Serial.print(epsilon);
      Serial.print("\nnew emissivity: \t");
      if (epsilon > 0) {
        epsilon--;
        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_eeprom_routine(E_EPSILON, epsilon);
        Wire.setClock(CLOCK_SENSOR);
        // calculate pixcij with new epsilon
        calcPixC();
        Serial.print(epsilon);
        Serial.print(" (new emissivity is stored in the EEPROM now)");
      }
      else {
        Serial.print(epsilon);
        Serial.print(" (you cannot set the emissivity lower as 0%)");
      }
      delay(1000);
      TimerLib.setInterval_us(ISR, timert);
      ReadingRoutineEnable = 1;
      break;


  }


}


/********************************************************************
   Function:        print_calc_steps()
   Description:     print every needed step for temperature calculation + pixel masking
 *******************************************************************/
void print_calc_steps2() {
  int64_t vij_pixc_and_pcscaleval;
  int64_t pixcij;
  int64_t vdd_calc_steps;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;
  signed long pixel;

  Serial.println("\n\ncalculate the average of VDD and PTAT buffer");

  Serial.print("PTATbuf[");
  Serial.print(PTAT_BUFFER_SIZE);
  Serial.print("] = { ");
  for (int i = 0; i < PTAT_BUFFER_SIZE; i++) {
    Serial.print(ptat_buffer[i]);
    if (i < (PTAT_BUFFER_SIZE - 1))
      Serial.print(" , ");
    else
      Serial.print(" }");
  }
  Serial.print("\nPTAT_average = ");
  Serial.print(ptat_av_uint16);

  Serial.println("\n\ncalculate ambient temperatur (Ta)");
  Serial.print("Ta = ");
  Serial.print(ptat_av_uint16);
  Serial.print(" * ");
  Serial.print(ptatgr_float, 5);
  Serial.print(" + ");
  Serial.print(ptatoff_float, 5);
  Serial.print(" = ");
  Serial.print(Ta);
  Serial.print(" (Value is given in dK)");


  /******************************************************************************************************************
    step 0: find column of lookup table
  ******************************************************************************************************************/
  for (int i = 0; i < NROFTAELEMENTS; i++) {
    if (Ta > XTATemps[i]) {
      table_col = i;
    }
  }
  dta = Ta - XTATemps[table_col];
  ydist = (int32_t)ADEQUIDISTANCE;

  Serial.println("\n\nprint all calculation steps for each pixel");
  Serial.println("table columns:");
  Serial.println("No\tpixel number");
  Serial.println("i\trepresents the row of the pixel");
  Serial.println("j\trepresents the column of the pixel");
  Serial.println("Vij\tis row pixel voltages (digital); readout from the RAM");
  Serial.println("I\tis the thermal offset compensated voltage");
  Serial.println("II\tis the thermal and electrical offset compensated voltage");
  Serial.println("III\tis the sensivity compensated IR voltage");
  Serial.println("T[dK]\tis final pixel temperature in dK (deci Kelvin)");
  Serial.println("T[°C]\tis final pixel temperature in °C");

  Serial.println("\n\nNo\ti\tj\tVij\tI\tII\tIII\tT[dK]\tT[°C]");
  Serial.println("-----------------------------------------------------------------------------");


  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      Serial.print(m * DevConst.PixelPerRow + n);
      Serial.print("\t");
      Serial.print(m);
      Serial.print("\t");
      Serial.print(n);
      /******************************************************************************************************************
         step 1: use a variable with bigger data format for the compensation steps
       ******************************************************************************************************************/
      pixel = (signed long) data_pixel[m][n];
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
       ******************************************************************************************************************/
      pixel -= (int32_t)(((int32_t)thgrad[m][n] * (int32_t)Ta) / (int32_t)gradscale_div);
      pixel -= (int32_t)thoffset[m][n];
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      pixel -= eloffset[m][n];
      /*
        if (m < DevConst.PixelPerColumn / 2) { // top half
        pixel -= eloffset[m % DevConst.RowPerBlock][n];
        }
        else { // bottom half
        pixel -= eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
        }
      */
      Serial.print("\t"); Serial.print(pixel);

      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel =  (int32_t)(vij_pixc_and_pcscaleval / pixcij_uint32[m][n]);
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
       ******************************************************************************************************************/
      table_row = pixel + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      pixel = (uint32_t)((vy - vx) * ((int32_t)(pixel + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);
      Serial.print("\t"); Serial.print(pixel);
      Serial.print("\t"); Serial.print((pixel - 2732) / 10);
      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      data_pixel[m][n] = (unsigned short)pixel;
      Serial.print("\n");

    }
  }

}



/********************************************************************
   Function:        print_eeprom_header()
   Description:
 *******************************************************************/
void print_eeprom_header() {
  Serial.print("data\t\tregister\ttype\t\tvalue\n");
  Serial.println("------------------------------------------------------------");
  Serial.print("PixCmin\t\t0x00-0x03\tfloat\t\t");
  Serial.println(pixcmin, 0);
  Serial.print("PixCmax\t\t0x04-0x07\tfloat\t\t");
  Serial.println(pixcmax, 0);
  Serial.print("gradScale\t0x08\t\tunsigned char\t");
  Serial.println(gradscale);
  Serial.print("TN\t\t0x0B-0x0C\tunsigned short\t");
  Serial.println(tablenumber);
  Serial.print("epsilon\t\t0x0D\t\tunsigned char\t");
  Serial.println(epsilon);
  Serial.print("MBIT(calib)\t0x1A\t\tunsigned char\t");
  Serial.println(mbit_calib);
  Serial.print("BIAS(calib)\t0x1B\t\tunsigned char\t");
  Serial.println(bias_calib);
  Serial.print("CLK(calib)\t0x1C\t\tunsigned char\t");
  Serial.println(clk_calib);
  Serial.print("BPA(calib)\t0x1D\t\tunsigned char\t");
  Serial.println(bpa_calib);
  Serial.print("PU(calib)\t0x1E\t\tunsigned char\t");
  Serial.println(pu_calib);
  Serial.print("Arraytype\t0x22\t\tunsigned char\t");
  Serial.println(arraytype);
  Serial.print("PTAT-gradient\t0x34-0x37\tfloat\t\t");
  Serial.println(ptatgr_float, 4);
  Serial.print("PTAT-offset\t0x38-0x3B\tfloat\t\t");
  Serial.println(ptatoff_float, 4);
  Serial.print("PTAT(Th1)\t0x3C-0x3D\tunsigned short\t");
  Serial.println(ptatth1);
  Serial.print("PTAT(Th2)\t0x3E-0x3F\tunsigned short\t");
  Serial.println(ptatth2);
  Serial.print("GlobalGain\t0x55-0x56\tunsigned short\t");
  Serial.println(globalgain);
  Serial.print("SensorID\t0x74-0x77\tunsigned long\t");
  Serial.println(id);
}


/********************************************************************
   Function:        print_eeprom_hex()
   Description:     print eeprom contint as hex values
 *******************************************************************/
void print_eeprom_hex() {

  Serial.print("\n\n\n---PRINT EEPROM (HEX)---\n");
  Serial.print("\n\nEEPROM 16x16\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    Serial.print("- ");
  }

  for (int i = 0; i < EEPROM_SIZE; i++) {


    if (i % 16 == 0) {
      Serial.print("\n");

      if (i < E_THGRAD) {
        Serial.print("HEADER\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_THOFFSET) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_PIJ) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < EEPROM_SIZE) {
        Serial.print("pixc\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }
    else {
      Serial.print("\t");
    }

    Serial.print("0x");
    if (eeprom_read_routine(i) < 0x10) {
      Serial.print("0");
    }
    Serial.print(eeprom_read_routine(i), HEX);

  }

  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");
}


/********************************************************************
   Function:      print_menu()
   Description:
 *******************************************************************/
void print_menu() {
  Serial.println("\n\n\n***************************************************");
  Serial.println("Application Shield v2.2                 /_/eimann");
  Serial.println("for ESP32-DevkitC-32D                  / /   Sensor");

  /*
    if (TABLENUMBER == tablenumber) {
    }
    else {
    Serial.println("WARNING: The def.h file is not the correct one for ");
    Serial.println("         this sensor. Please replace the def.h file.");
    }
  */
  Serial.println("\nYou can choose one of these options by sending the \ncharacter\n ");
  Serial.println("read SENSOR values:");
  Serial.println("  a... final array temperatures (in deci Kelvin)");
  Serial.println("  b... show all raw values (in digits)");
  Serial.println("  c... show all calculation steps");
  Serial.println("read EEPROM values:");
  Serial.println("  d... whole eeprom content (in hexadecimal)");
  Serial.println("  e... Header values");
  Serial.println("  h... ThGrad");
  Serial.println("  i... ThOff");
  Serial.println("  j... PixC (scaled)");
  Serial.println("write/change EEPROM values:");
  Serial.println("  k... increase emissivity by 1");
  Serial.println("  l... decrease emissivity by 1");
  Serial.println("\t\t\t\t\tver2.2 (dp)");
  Serial.println("***************************************************\n\n\n");
}
