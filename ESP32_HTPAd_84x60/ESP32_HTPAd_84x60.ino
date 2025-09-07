/*** PROGRAMM INFO***************************************************************************************
  source code for ESP32 and HTPAd Application Shield
  name:           ESP32_HTPAd_32x32.ino
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
unsigned char mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, mbit_user, bias_user, clk_user, bpa_user, pu_user;
unsigned char nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, lastepsilon, arraytype;
unsigned char deadpixmask[ALLOWED_DEADPIX];
signed char globaloff;
signed char thgrad[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
unsigned short tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
unsigned short deadpixadr[ALLOWED_DEADPIX * 2];
signed short thoffset[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
signed short vddcompgrad[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
signed short vddcompoff[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
unsigned long id, ptatoff;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;
// use a heap allocated memory to store the pixc instead of a nxm array
unsigned long *pixc2_0; // start address of the allocated heap memory
unsigned long *pixc2; // increasing address pointer

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
unsigned short Ta, ptat_av_uint16, vdd_av_uint16;


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
uint32_t gradscale_div;
uint32_t vddscgrad_div;
uint32_t vddscoff_div;
int vddcompgrad_n;
int vddcompoff_n;
unsigned long t1;
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
  // use a heap allocated memory to store the pixc instead of a global nxm array
  //*******************************************************************
  //this is special for the ESP32, because it isn't enough space in the stack
  // you don't have to do that in this way if you have enough space to store pixc as global variable
  pixc2_0 = (unsigned long*)heap_caps_malloc(NUMBER_OF_PIXEL * 4, MALLOC_CAP_32BIT);
  if (pixc2_0 == NULL)
  {
    Serial.println("heap_caps_malloc failed");
  }
  else
  {
    Serial.println("heap_caps_malloc succeeded");
    pixc2 = pixc2_0; // set pointer to start address of the allocated heap
  }


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
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  pinMode(SPI_CS, OUTPUT);

  //*******************************************************************
  // searching for sensor; if connected: read the whole EEPROM
  // (LEDs are blinking as long as no sensor is detected)
  //*******************************************************************
  // search sensor via ID
  Serial.print("\n\nsearch device... ");
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
  write_calibration_settings_to_sensor();

  // to start sensor set configuration register to 0x09
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  if (read_block_num < NUMBER_OF_BLOCKS)
    write_sensor_byte(CONFIGURATION_REGISTER, 0x09 | 0x10 * read_block_num);
  else
    write_sensor_byte(CONFIGURATION_REGISTER, 0x0B);
  Serial.println("HTPAd is ready");


  //*******************************************************************
  // do bigger calculation here before you jump into the loop() function
  //*******************************************************************
  gradscale_div = pow(2, gradscale);
  vddscgrad_div = pow(2, vddscgrad);
  vddscoff_div = pow(2, vddscoff);
  calcPixC(); // calculate the pixel constants

  //*******************************************************************
  // timer initialization
  //*******************************************************************
  timert = calc_timert(clk_calib, mbit_calib);
  TimerLib.setInterval_us(ISR, timert );
  //*******************************************************************
  // print the menu for the first time
  //*******************************************************************
#ifdef SERIALMODE
  print_menu();
#endif



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
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap

  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      pixcij = (double)pixcmax;
      pixcij -= (double)pixcmin;
      pixcij /= (double)65535.0;
      pixcij *= (double) * pixc2;
      pixcij += (double)pixcmin;
      pixcij /= (double)100.0;
      pixcij *= (double)epsilon;
      pixcij /= (double)10000.0;
      pixcij *= (double)globalgain;
      pixcij += 0.5;

      *pixc2 = (unsigned long)pixcij;
      pixc2++;

    }
  }

  lastepsilon = epsilon;

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
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap


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
      pixel -= (int32_t)(((int32_t)thgrad[m][n] * (int32_t)ptat_av_uint16) / (int32_t)gradscale_div);
      pixel -= (int32_t)thoffset[m][n];

      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      if (m < DevConst.PixelPerColumn / 2) { // top half
        pixel -= eloffset[m % DevConst.RowPerBlock][n];
      }
      else { // bottom half
        pixel -= eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }

      /******************************************************************************************************************
         step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
       ******************************************************************************************************************/
      // first select VddCompGrad and VddCompOff for pixel m,n:
      if (m < DevConst.PixelPerColumn / 2) {      // top half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock][n];
      }
      else {       // bottom half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      // now do the vdd calculation
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16  - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      pixel -= vdd_calc_steps;

      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel =  (int32_t)(vij_pixc_and_pcscaleval / *pixc2);
      pixc2++;
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
         step 7: add GlobalOffset (stored as signed char)
       ******************************************************************************************************************/
      pixel += globaloff;

      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      data_pixel[m][n] = (unsigned short)pixel;

    }
  }

  /******************************************************************************************************************
    step 8: overwrite the uncompensate pixel with the new calculated compensated value
  ******************************************************************************************************************/
  pixel_masking();

}


/********************************************************************
   Function:      calc_timert(uint8_t clk, uint8_t mbit)
   Description:   calculate the duration of the timer which reads the sensor blocks
 *******************************************************************/
word calc_timert(uint8_t clk, uint8_t mbit) {

  float a;
  uint16_t calculated_timer_duration;


  float Fclk_float = 10000000.0 / 63.0 * (float)clk + 6000000.0;    // calc clk in Hz
  a = 32.0 * (float)(pow(2, (unsigned char)(mbit & 0b00001111)) + 100.0) / Fclk_float;

  calculated_timer_duration = (unsigned short)(a * 1000000); // c in s | timer_duration in µs
  return calculated_timer_duration;
}




/********************************************************************
   Function:        void pixel_masking()
   Description:     repair dead pixel by using the average of the neighbors
 *******************************************************************/
void pixel_masking() {


  uint8_t number_neighbours[ALLOWED_DEADPIX];
  uint32_t temp_defpix[ALLOWED_DEADPIX];

  for (int i = 0; i < nrofdefpix; i++) {
    number_neighbours[i] = 0;
    temp_defpix[i] = 0;

    // top half
    if (deadpixadr[i] < (unsigned short)(NUMBER_OF_PIXEL / 2)) {

      if ( (deadpixmask[i] & 1 )  == 1) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

    }

    // bottom half
    else {

      if ( (deadpixmask[i] & 1 )  == 1 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }
    }

    temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
    data_pixel[deadpixadr[i] / PIXEL_PER_ROW][deadpixadr[i] % PIXEL_PER_ROW] = temp_defpix[i];

  }

}


/********************************************************************
   Function:        void readblockinterrupt()
   Description:     read one sensor block and change configuration register to next block
                    (also read electrical offset when read_eloffset_next_pic is set)
 *******************************************************************/
void readblockinterrupt() {

  TimerLib.clearTimer();
  unsigned char bottomblock;

  // wait for end of conversion bit (~27ms)
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(SPI_CS, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(RAMoutput[read_block_num], BLOCK_LENGTH); // receive data
  digitalWrite(SPI_CS, LOW);


  // get data of bottom half:
  bottomblock = (unsigned char)((unsigned char)(NUMBER_OF_BLOCKS + 1) * 2 - read_block_num - 1);
  digitalWrite(SPI_CS, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(RAMoutput[bottomblock], BLOCK_LENGTH); // receive data
  digitalWrite(SPI_CS, LOW);


  read_block_num++;

  if (read_block_num < NUMBER_OF_BLOCKS) {

    // to start sensor set configuration register to 0x09
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  x  |  x  |   1   |    0     |   0   |    1   |
    write_sensor_byte(CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x10 * read_block_num) + (0x04 * switch_ptat_vdd)));
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
      write_sensor_byte(CONFIGURATION_REGISTER, (unsigned char)(0x0B + (0x04 * switch_ptat_vdd)));
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
      write_sensor_byte(CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x04 * switch_ptat_vdd)));
    }
  }

  TimerLib.setInterval_us(ISR, timert );
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
  mbit_calib = read_EEPROM_byte(E_MBIT_CALIB);
  bias_calib = read_EEPROM_byte(E_BIAS_CALIB);
  clk_calib = read_EEPROM_byte(E_CLK_CALIB);
  bpa_calib = read_EEPROM_byte(E_BPA_CALIB);
  pu_calib = read_EEPROM_byte(E_PU_CALIB);
  mbit_user = read_EEPROM_byte(E_MBIT_USER);
  bias_user = read_EEPROM_byte(E_BIAS_USER);
  clk_user = read_EEPROM_byte(E_CLK_USER);
  bpa_user = read_EEPROM_byte(E_BPA_USER);
  pu_user = read_EEPROM_byte(E_PU_USER);
  vddth1 = read_EEPROM_byte(E_VDDTH1_2) << 8 | read_EEPROM_byte(E_VDDTH1_1);
  vddth2 = read_EEPROM_byte(E_VDDTH2_2) << 8 | read_EEPROM_byte(E_VDDTH2_1);
  vddscgrad = read_EEPROM_byte(E_VDDSCGRAD);
  vddscoff = read_EEPROM_byte(E_VDDSCOFF);
  ptatth1 = read_EEPROM_byte(E_PTATTH1_2) << 8 | read_EEPROM_byte(E_PTATTH1_1);
  ptatth2 = read_EEPROM_byte(E_PTATTH2_2) << 8 | read_EEPROM_byte(E_PTATTH2_1);
  nrofdefpix = read_EEPROM_byte(E_NROFDEFPIX);
  gradscale = read_EEPROM_byte(E_GRADSCALE);
  tablenumber = read_EEPROM_byte(E_TABLENUMBER2) << 8 | read_EEPROM_byte(E_TABLENUMBER1);
  arraytype = 13;//read_EEPROM_byte(E_ARRAYTYPE);
  b[0] = read_EEPROM_byte(E_PTATGR_1);
  b[1] = read_EEPROM_byte(E_PTATGR_2);
  b[2] = read_EEPROM_byte(E_PTATGR_3);
  b[3] = read_EEPROM_byte(E_PTATGR_4);
  ptatgr_float = *(float*)b;
  b[0] = read_EEPROM_byte(E_PTATOFF_1);
  b[1] = read_EEPROM_byte(E_PTATOFF_2);
  b[2] = read_EEPROM_byte(E_PTATOFF_3);
  b[3] = read_EEPROM_byte(E_PTATOFF_4);
  ptatoff_float = *(float*)b;
  b[0] = read_EEPROM_byte(E_PIXCMIN_1);
  b[1] = read_EEPROM_byte(E_PIXCMIN_2);
  b[2] = read_EEPROM_byte(E_PIXCMIN_3);
  b[3] = read_EEPROM_byte(E_PIXCMIN_4);
  pixcmin = *(float*)b;
  b[0] = read_EEPROM_byte(E_PIXCMAX_1);
  b[1] = read_EEPROM_byte(E_PIXCMAX_2);
  b[2] = read_EEPROM_byte(E_PIXCMAX_3);
  b[3] = read_EEPROM_byte(E_PIXCMAX_4);
  pixcmax = *(float*)b;
  epsilon = read_EEPROM_byte(E_EPSILON);
  globaloff = read_EEPROM_byte(E_GLOBALOFF);
  globalgain = read_EEPROM_byte(E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(E_GLOBALGAIN_1);


  // for (int m = 0; m < DevConst.PixelPerColumn; m++) {
  //   for (int n = 0; n < DevConst.PixelPerRow; n++) {

  // --- DeadPixAdr ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixadr[i] = read_EEPROM_byte(E_DEADPIXADR + 2 * i + 1 ) << 8 | read_EEPROM_byte(E_DEADPIXADR + 2 * i);
    if (deadpixadr[i] > (unsigned short)(DevConst.NumberOfPixel / 2)) {  // adaptedAdr:
      deadpixadr[i] = (unsigned short)(DevConst.NumberOfPixel) + (unsigned short)(DevConst.NumberOfPixel / 2) - deadpixadr[i] + 2 * (unsigned short)(deadpixadr[i] % DevConst.PixelPerRow ) - DevConst.PixelPerRow;
    }
  }


  // --- DeadPixMask ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixmask[i] = read_EEPROM_byte(E_DEADPIXMASK + i);
  }


  // --- Thgrad_ij, ThOffset_ij and P_ij ---
  m = 0;
  n = 0;
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap // reset pointer to initial address
  // top half
  for (int i = 0; i < (unsigned short)(DevConst.NumberOfPixel / 2); i++) {
    thgrad[m][n] = (signed char)(read_EEPROM_byte(E_THGRAD + i));
    thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
    *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m++;  // !!!! forwards !!!!
    }
  }
  // bottom half
  m = (unsigned char)(DevConst.PixelPerColumn - 1);
  n = 0;
  for (int i = (unsigned short)(DevConst.NumberOfPixel / 2); i < (unsigned short)(DevConst.NumberOfPixel); i++) {
    thgrad[m][n] = read_EEPROM_byte(E_THGRAD + i);
    thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
    *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
    n++;

    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m--;      // !!!! backwards !!!!
    }
  }

  //---VddCompGrad and VddCompOff---
  // top half
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < (unsigned short)(DevConst.PixelPerBlock); i++) {
    vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
    vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m++;  // !!!! forwards !!!!
    }
  }
  // bottom half
  m = (unsigned char)(DevConst.RowPerBlock * 2 - 1);
  n = 0;
  for (int i = (unsigned short)(DevConst.PixelPerBlock); i < (unsigned short)(DevConst.PixelPerBlock * 2); i++) {
    vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
    vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m--;      // !!!! backwards !!!!
    }
  }

}


/********************************************************************
   Function:        void read_EEPROM_byte(unsigned int eeaddress )
   Description:     read eeprom register as 8
   Dependencies:    register address (address)
 *******************************************************************/
byte read_EEPROM_byte(unsigned int address ) {
  byte rdata = 0xFF;
  digitalWrite(SPI_CS, LOW);  // set LOW to communicate with eeprom
  SPI.transfer(0b00000011); // READ command eeprom
  SPI.transfer16(address);  // register address eeprom
  rdata = SPI.transfer(0xFF); // end of message
  digitalWrite(SPI_CS, HIGH); // set HIGH, back to sensor

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
        // top half
        data_pixel[m + i * DevConst.RowPerBlock][n] =
          (unsigned short)(RAMoutput[i][pos] << 8 | RAMoutput[i][pos + 1]);
        // bottom half
        data_pixel[DevConst.PixelPerColumn - 1 - m - i * DevConst.RowPerBlock][n] =
          (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos + 1]);
      }


      /******************************************************************************************************************
        new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
      ******************************************************************************************************************/
      if (picnum % ELOFFSETS_BUFFER_SIZE == 1) {
        if (!eloffset[m][n]) {
          // top half
          eloffset[m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          // bottom half
          eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
          use_eloffsets_buffer = 1;
        }
        else {
          // use a moving average filter
          // top half
          sum = (unsigned long)eloffset[m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          eloffset[m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
          // bottom half
          sum = (unsigned long)eloffset[2 * DevConst.RowPerBlock - 1 - m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
          eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
        }
      }


    }

  }



  /******************************************************************************************************************
    new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  if (switch_ptat_vdd == 1) {
    sum = 0;
    // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
    for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
      // block top half
      sum += (unsigned short)(RAMoutput[i][DevConst.PTATPos] << 8 | RAMoutput[i][DevConst.PTATPos + 1]);
      // block bottom half
      sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos + 1]);
    }
    ptat_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));
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


  /******************************************************************************************************************
    new VDD values (store them in VDD buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  if (switch_ptat_vdd == 0) {
    sum = 0;
    // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
    for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
      // block top half
      sum += (unsigned short)(RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][DevConst.VDDPos + 1]);
      // block bottom half
      sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos + 1]);
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

  }

}


/********************************************************************
   Function:        void write_calibration_settings_to_sensor()
   Description:     write calibration data (from eeprom) to trim registers (sensor)
 *******************************************************************/
void write_calibration_settings_to_sensor() {

  write_sensor_byte(TRIM_REGISTER1, mbit_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER2, bias_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER3, bias_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER4, clk_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER5, bpa_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER6, bpa_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER7, pu_calib);
}


/********************************************************************
   Function:        void read_EEPROM_byte(unsigned int eeaddress )
   Description:     read eeprom register as 8
   Dependencies:    register address (address)
 *******************************************************************/
void write_EEPROM_byte(unsigned short address, unsigned char content ) {
  digitalWrite(SPI_CS, HIGH); // set HIGH, back to sensor
  delay(1);

  //WRITE ENABLE SEQUENCE (WREN)
  digitalWrite(SPI_CS, LOW);  // set LOW to communicate with eeprom
  SPI.transfer(0b00000110); // WRITE command eeprom
  digitalWrite(SPI_CS, HIGH); // set HIGH, back to sensor
  delay(1);

  //BYTE WRITE SEQUENCE
  digitalWrite(SPI_CS, LOW);  // set LOW to communicate with eeprom
  SPI.transfer(0b00000010); // WRITE command eeprom
  SPI.transfer16(address);  // register address eeprom
  SPI.transfer(content);
  digitalWrite(SPI_CS, HIGH); // set HIGH, back to sensor
  delay(1);

  //WRITE DISABLE SEQUENCE (WRDI)
  digitalWrite(SPI_CS, LOW);  // set LOW to communicate with eeprom
  SPI.transfer(0b00000100); // WRITE command eeprom
  digitalWrite(SPI_CS, HIGH); // set HIGH, back to sensor
  delay(1);

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
  delay(5);
  write_sensor_byte(TRIM_REGISTER7, pu_user);

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
      udp.print( (mbit_calib & 15) + 4);    // calc ADC resolution
      udp.print("\r\n");
      //udp.print("HTPA32x32d v.0.01 Heimann Sensor GmbH; written by D. Pauer 2019-11-13\r\n");
      udp.print("HTPAxxxxxd v.2.00 Heimann Sensor GmbH; written by D. Pauer 2021-01-13\r\n");
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
        udp.print("HTPAxxxxxd 2021/01/13 v.2.00 Heimann Sensor GmbH; written by D. Pauer\n\r");
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

        write_EEPROM_byte(E_CLK_USER, clk_user);
        delay(5);

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

        write_EEPROM_byte(E_BIAS_USER, bias_user);
        delay(5);

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

        write_EEPROM_byte(E_BPA_USER, bpa_user);
        delay(5);

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

        write_EEPROM_byte(E_MBIT_USER, mbit_user);
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

        TimerLib.setInterval_us(ISR, timert );
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

        write_EEPROM_byte(E_PU_USER, pu_user);
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
          write_EEPROM_byte(E_EPSILON, epsilon);

          // calculate pixcij with new epsilon
          double d = (double)epsilon / (double)lastepsilon;
          pixc2 = pixc2_0; // set pointer to start address of the allocated heap
          for (int m = 0; m < DevConst.PixelPerColumn; m++) {
            for (int n = 0; n < DevConst.PixelPerRow; n++) {
              *pixc2 = (unsigned long)((double)(*pixc2) * (double)d);
              pixc2++;
            }
          }
          lastepsilon = epsilon;

          udp.beginPacket(ip_partner, udp.remotePort());
          udp.print("Emission changed to ");
          udp.print(epsilon);
          udp.print("%\r\n\r\n");
          udp.endPacket();
          delay(1000);
          TimerLib.setInterval_us(ISR, timert );
        }
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

  packet1[0] = packetnumber + 1;
  udp.beginPacket(ip_partner, udp.remotePort());
  udp.write(packet1, LAST_UDP_PACKET_LENGTH);
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

    case 'f':
      while (state);
      ReadingRoutineEnable = 0;
      Serial.print("\n\n\n---VddCompGrad---\n");
      for (int m = 0; m < (DevConst.RowPerBlock * 2); m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(vddcompgrad[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;

    case 'g':
      while (state);
      ReadingRoutineEnable = 0;
      Serial.print("\n\n\n---VddCompOff---\n");
      for (int m = 0; m < (DevConst.RowPerBlock * 2); m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(vddcompoff[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
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
      pixc2 = pixc2_0; // set pointer to start address of the allocated heap
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(*(pixc2 + m * DevConst.PixelPerRow + n));
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
        write_EEPROM_byte(E_EPSILON, epsilon);
        // calculate pixcij with new epsilon
        pixc2 = pixc2_0; // set pointer to start address of the allocated heap
        double d = (double)epsilon / (double)lastepsilon;
        for (int m = 0; m < DevConst.PixelPerColumn; m++) {
          for (int n = 0; n < DevConst.PixelPerRow; n++) {
            *(pixc2 + m * DevConst.PixelPerRow + n) = (unsigned long)((double) * (pixc2 + m * DevConst.PixelPerRow + n) * (double)d);
          }
        }
        lastepsilon = epsilon;
        Serial.print(epsilon);
        Serial.println(" (new emissivity is stored in the EEPROM now)");
      }
      else {
        Serial.print(epsilon);
        Serial.println(" (you cannot set the emissivity higher than 100%)");
      }
      delay(1000);
      TimerLib.setInterval_us(ISR, timert );
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
        write_EEPROM_byte(E_EPSILON, epsilon);
        // calculate pixcij with new epsilon
        pixc2 = pixc2_0; // set pointer to start address of the allocated heap
        double d = (double)epsilon / (double)lastepsilon;
        for (int m = 0; m < DevConst.PixelPerColumn; m++) {
          for (int n = 0; n < DevConst.PixelPerRow; n++) {
            *(pixc2 + m * DevConst.PixelPerRow + n) = (unsigned long)((double) * (pixc2 + m * DevConst.PixelPerRow + n) * (double)d);
          }
        }
        lastepsilon = epsilon;
        Serial.print(epsilon);
        Serial.print(" (new emissivity is stored in the EEPROM now)");
      }
      else {
        Serial.print(epsilon);
        Serial.print(" (you cannot set the emissivity lower as 0%)");
      }
      delay(1000);
      TimerLib.setInterval_us(ISR, timert );
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
  pixc2 = pixc2_0;

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

  Serial.print("\nVDDbuf[");
  Serial.print(VDD_BUFFER_SIZE);
  Serial.print("] = { ");
  for (int i = 0; i < VDD_BUFFER_SIZE; i++) {
    Serial.print(vdd_buffer[i]);
    if (i < (VDD_BUFFER_SIZE - 1))
      Serial.print(" , ");
    else
      Serial.print(" }");
  }
  Serial.print("\nVDD_average = ");
  Serial.print(vdd_av_uint16);

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
  Serial.println("III\tis the Vdd compensated voltage");
  Serial.println("IV\tis the sensivity compensated IR voltage");
  Serial.println("T[dK]\tis final pixel temperature in dK (deci Kelvin)");
  Serial.println("T[°C]\tis final pixel temperature in °C");

  Serial.println("\n\nNo\ti\tj\tVij\tI\tII\tIII\tIV\tT[dK]\tT[°C]");
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
      pixel -= (int32_t)(((int32_t)thgrad[m][n] * (int32_t)ptat_av_uint16) / (int32_t)gradscale_div);
      pixel -= (int32_t)thoffset[m][n];
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      if (m < DevConst.PixelPerColumn / 2) { // top half
        pixel -= eloffset[m % DevConst.RowPerBlock][n];
      }
      else { // bottom half
        pixel -= eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
       ******************************************************************************************************************/
      // first select VddCompGrad and VddCompOff for pixel m,n:
      if (m < DevConst.PixelPerColumn / 2) {      // top half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock][n];
      }
      else {       // bottom half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      // now do the vdd calculation
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16  - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      pixel -= vdd_calc_steps;
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel =  (int32_t)(vij_pixc_and_pcscaleval / *pixc2);
      pixc2++;
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

      /******************************************************************************************************************
         step 7: add GlobalOffset (stored as signed char)
       ******************************************************************************************************************/
      pixel += globaloff;
      Serial.print("\t"); Serial.print(pixel);
      Serial.print("\t"); Serial.print((float)((pixel - 2732) / 10.0));
      Serial.print("\n");
      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      data_pixel[m][n] = (unsigned short)pixel;

    }
  }

  /******************************************************************************************************************
    step 8: overwrite the uncompensate pixel with the new calculated compensated value
  ******************************************************************************************************************/
  pixel_masking();


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
  Serial.print("VDDTH1\t\t0x26-0x27\tunsigned short\t");
  Serial.println(vddth1);
  Serial.print("VDDTH2\t\t0x28-0x29\tunsigned short\t");
  Serial.println(vddth2);
  Serial.print("PTAT-gradient\t0x34-0x37\tfloat\t\t");
  Serial.println(ptatgr_float, 4);
  Serial.print("PTAT-offset\t0x38-0x3B\tfloat\t\t");
  Serial.println(ptatoff_float, 4);
  Serial.print("PTAT(Th1)\t0x3C-0x3D\tunsigned short\t");
  Serial.println(ptatth1);
  Serial.print("PTAT(Th2)\t0x3E-0x3F\tunsigned short\t");
  Serial.println(ptatth2);
  Serial.print("VddScGrad\t0x4E\t\tunsigned char\t");
  Serial.println(vddscgrad);
  Serial.print("VddScOff\t0x4F\t\tunsigned char\t");
  Serial.println(vddscoff);
  Serial.print("GlobalOff\t0x54\t\tsigned char\t");
  Serial.println(globaloff);
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
  Serial.print("\n\nEEPROM 32x32\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    Serial.print("- ");
  }

  for (int i = 0; i < EEPROM_SIZE; i++) {


    if (i % 16 == 0) {
      Serial.print("\n");

      if (i < E_DEADPIXADR) {
        Serial.print("HEADER\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x00D0) {
        Serial.print("DEADPIX\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_VDDCOMPGRAD) {
        Serial.print("FREE\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_VDDCOMPOFF) {
        Serial.print("VDDGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_THGRAD) {
        Serial.print("VDDOFF\t0x");
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
    if (read_EEPROM_byte(i) < 0x10) {
      Serial.print("0");
    }
    Serial.print(read_EEPROM_byte(i), HEX);

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
  Serial.println("  f... VddCompGrad");
  Serial.println("  g... VddCompOff");
  Serial.println("  h... ThGrad");
  Serial.println("  i... ThOff");
  Serial.println("  j... PixC (scaled)");
  Serial.println("write/change EEPROM values:");
  Serial.println("  k... increase emissivity by 1");
  Serial.println("  l... decrease emissivity by 1");
  Serial.println("\t\t\t\t\tver2.0 (dp)");
  Serial.println("***************************************************\n\n\n");
}
