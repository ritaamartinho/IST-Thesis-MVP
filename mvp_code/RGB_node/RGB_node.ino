//CURRENT CODE HAS APN FOR 1NCE

//power - 6uA sem RFM, 7uA com
// - 78uA
//NEW SCREEN DEFINES
#include <Arduino.h>

//#include <reset.h>

#include <Wire.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#include <Adafruit_SHT31.h>
#include "Adafruit_APDS9960.h"

#include <CayenneLPP.h>
CayenneLPP lpp(28);

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_APDS9960 apds;

#define TRUE 1
#define FALSE 0

// END FLASH
//

#include <Sodaq_wdt.h>
#include <RTCZero.h> //timer thing
#include <FlashStorage.h> //to store the sequence number
#include <avr/dtostrf.h>


#define LUX_ANALOG A1
#define LED_R A0
#define LED_B A3

#define LUX_ENABLE 7

#define MEDIR_CAP 3 //PA09
#define CAP_V A2

#define RFM_CS 9 //PA07
#define RFM_DIO0 13 //PA17
#define RFM_RST 5//PA15

//const byte voltage_interrupt = A3;
//
#define Serial Serial


//END DEFINITION

//END SCREEN DEFINE

//Flash Storage

FlashStorage(seq_0, byte);
FlashStorage(seq_1, byte);
FlashStorage(DST, int); //0 not written, 1 none, 2 DST

#define CONSOLE_STREAM   Serial
#define MODEM_STREAM     Serial1

#define HASH_SIZE 32
#define BLOCK_SIZE 64

int count = 0;
int last_color = -1;

//CONNECTION 

#define CUSTOM
int DEBUG = 1;



//RTCZero rtc;

#define ADC_AREF 2.33f
#define BATVOLT_R1 100.0f
#define BATVOLT_R2 150.0f
#define BATVOLT_PIN CAP_V


/* Create an rtc object */
RTCZero rtc;

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 34;
const byte hours = 16;

const byte min_alarm = 30; 
/* Change these values to set the current initial date */
const byte day = 21;
const byte month = 5;
const byte year = 21;

bool initmodem = false;
volatile bool isClock = false; //variables shared between main code and ISR must be declared volatile to tell the compiler that their variable can change at any time
volatile bool isButton = false;


float getBatteryVoltage()
{
    digitalWrite(MEDIR_CAP, HIGH);
    
    sodaq_wdt_safe_delay(100);
    
    uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(CAP_V));

    /*
    Serial.println("Analog Read");
    Serial.println(analogRead(CAP_V));
    */
    digitalWrite(MEDIR_CAP, LOW);
    sodaq_wdt_safe_delay(100);

    return (float)voltage;
}

void alarmMatch()
{
  isClock = true;
  
}

int reset_cause = 0; //0 - normal power on ; 1 - Reset requested by System ; 2 - Reset by Watchdog ; 3 - External reset requested ; 4 - Reset Brown Out 3.3V; 5 - Reset Brown Out 1.2V
bool first_keepalive = true; //just send reset cause on first keepalive

//#define REG_PM_RCAUSE (0x40000438U)


//COMMUNICATION STUFF

#define SERIAL_DEBUG Serial  // serial USB debug - Arm M0
#define SERIAL_MODEM Serial1 // serial to Quectel BC66



int delay_var = 0;

#include "secrets.h"
#include "TinyLoRa.h"
#include <SPI.h>

TinyLoRa lora = TinyLoRa(RFM_DIO0, RFM_CS, RFM_RST);

int msg_count = 0;

#define EN_SENSOR 4

int num_lux_read = 3;

int get_lux()
{ 
    int lux_out = 0;
    for (int i = 0; i < num_lux_read ; i++)
    {
        lux_out = lux_out + analogRead(LUX_ANALOG); 
        sodaq_wdt_safe_delay(100/(num_lux_read));
    }
    lux_out = lux_out / num_lux_read;
    return lux_out;
}

/*
 * Reads RGB values from RGB light sensor.
 * Returns code of the color read.
 * 0 - too low light intensity, no color, light off
 * 1 - red
 * 2 - green
 * 3 - blue
 */
int get_color()
{ 
    int color_code = 0;
    
    uint16_t r, g, b, c;
    //create some variables to store the color data in
    //wait for color data to be ready
    while(!apds.colorDataReady()){
      delay(5);
    }    
    //get the data and print the different channels
    apds.getColorData(&r, &g, &b, &c);
    /*Serial.print("red: ");Serial.print(r);
    Serial.print(" green: "); Serial.print(g);
    Serial.print(" blue: "); Serial.print(b);
    Serial.print(" clear: "); Serial.println(c);*/

    if(c > 200){
      if(r > g && r > b)
        color_code = 1;
      else if(g > r && g > b)
        color_code = 2;
      else if(b > r && b > g)
        color_code = 3;
    }
    //Serial.println("out of get_color()");
    return color_code;
}

void send_lora(int color)
{   
    /*enable sensors */
    digitalWrite(EN_SENSOR,HIGH);
    digitalWrite(LUX_ENABLE,HIGH);
    
    sodaq_wdt_safe_delay(50);
    lpp.reset();

    //Serial.println("Sending LoRa Data...");
    
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
      Serial.println("Couldn't find SHT31");
      
      float t = 0.0;
      float h = 0.0;
      float b_volt = 0;   
      b_volt = getBatteryVoltage();
      b_volt = b_volt/(100.0);

      lpp.addTemperature(0, t);
      lpp.addRelativeHumidity(1, h);    
      lpp.addAnalogInput(2, b_volt);
      lpp.addLuminosity(3, color);
      
      lora.frameCounter = msg_count;      
      lora.sendData(lpp.getBuffer(), lpp.getSize(), lora.frameCounter);
      
      //Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
      lora.frameCounter++;
      msg_count++;
    }
    else{      
      float t = sht31.readTemperature();
      float h = sht31.readHumidity();
    
      if (! isnan(t)) {  // check if 'is not a number'
        Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
      } else { 
        Serial.println("Failed to read temperature");
      }    
      if (! isnan(h)) {  // check if 'is not a number'
        Serial.print("Hum. % = "); Serial.println(h);
      } else { 
        Serial.println("Failed to read humidity");
      }

      float b_volt = 0;
      b_volt = getBatteryVoltage();
      b_volt = b_volt/(100.0);
      
      lpp.addTemperature(0, t);   
      lpp.addRelativeHumidity(1, h);
      lpp.addAnalogInput(2, b_volt);
      lpp.addLuminosity(3, color);
      
      lora.frameCounter = msg_count;
      lora.sendData(lpp.getBuffer(), lpp.getSize(), lora.frameCounter);

      //Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
      lora.frameCounter++;
      msg_count++;
    }
    /* Turn off Sensors */
    digitalWrite(EN_SENSOR,LOW);
    digitalWrite(LUX_ENABLE,LOW);
}


void setup_BOD33()
{
    Serial.println(F("Fuse settings before:"));
    Serial.println((*(uint32_t*)NVMCTRL_USER), HEX);         // Display the current user word 0 fuse settings
    Serial.println((*(uint32_t*)(NVMCTRL_USER + 4)), HEX);   // Display the current user word 1 fuse settings
    uint32_t userWord0 = *((uint32_t*)NVMCTRL_USER);            // Read fuses for user word 0
    uint32_t userWord1 = *((uint32_t*)(NVMCTRL_USER + 4));      // Read fuses for user word 1
    NVMCTRL->CTRLB.bit.CACHEDIS = 1;                            // Disable the cache
    NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS / 2;               // Set the address
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_EAR |                // Erase the auxiliary user page row
                         NVMCTRL_CTRLA_CMDEX_KEY;
    while(!NVMCTRL->INTFLAG.bit.READY)                          // Wait for the NVM command to complete
    NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;                 // Clear the error flags
    NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS / 2;               // Set the address
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC |                // Clear the page buffer
                         NVMCTRL_CTRLA_CMDEX_KEY;
    while(!NVMCTRL->INTFLAG.bit.READY)                          // Wait for the NVM command to complete
    NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;                 // Clear the error flags
    *((uint32_t*)NVMCTRL_USER) = userWord0 & ~FUSES_BOD33_EN_Msk;  // Disable the BOD33 enable fuse in user word 0
    *((uint32_t*)(NVMCTRL_USER + 4)) = userWord1;               // Copy back user word 1 unchanged
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_WAP  |               // Write to the user page
                         NVMCTRL_CTRLA_CMDEX_KEY;
    while(!NVMCTRL->INTFLAG.bit.READY)                          // Wait for the NVM command to complete
    NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;                 // Clear the error flags
    NVMCTRL->CTRLB.bit.CACHEDIS = 0;                            // Enable the cache
    Serial.println(F("Fuse settings after:"));
    Serial.println((*(uint32_t*)NVMCTRL_USER), HEX);         // Display the current user word 0 fuse settings
    Serial.println((*(uint32_t*)(NVMCTRL_USER + 4)), HEX);   // Display the current user word 1 fuse settings

}

void nvm_wait_states() //maybe not necessary
{
     NVMCTRL->CTRLB.bit.RWS = 3; 
}


void setup_rtc_debug()
{
   byte h,mm,s = 0;
    
    rtc.begin(); // initialize RTC

    rtc.setTime(h, mm, s);
    
    rtc.setAlarmTime(h, mm, s+15);
    
    rtc.enableAlarm(rtc.MATCH_SS); //rtc.MATCH_MMSS   RITAAAAA
    
    rtc.attachInterrupt(alarmMatch);
}


void i2c_scan()
{
     byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Serial.println("Scanning before begin ...");
    Wire.beginTransmission(address);
    Serial.println("Scanning after begin ...");
    
    error = Wire.endTransmission();
    Serial.println("Scanning after end trans ...");
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}


//
//enable sensor - PA8


void setup()
{
    asm(".global _printf_float"); //very impotant for sprintf with float
      
    Serial.begin(115200);
      // Bootup sodaq_wdt_safe_delay to programm the board.

    setup_BOD33();
    
    nvm_wait_states();
    
    //Find Reset Cause    
    if (REG_PM_RCAUSE == PM_RCAUSE_SYST){
      Serial.println("Reset requested by system");
      reset_cause = 1;
    }
    if (REG_PM_RCAUSE == PM_RCAUSE_WDT){
      Serial.println("Reset requested by Watchdog");
      reset_cause = 2;
    }
    if (REG_PM_RCAUSE == PM_RCAUSE_EXT){
      Serial.println("External reset requested");
      reset_cause = 3;
    }
    if (REG_PM_RCAUSE == PM_RCAUSE_BOD33){
      Serial.println("Reset brown out 3.3V");
      reset_cause = 4;
    }
    if (REG_PM_RCAUSE == PM_RCAUSE_BOD12){
      Serial.println("Reset brown out 1.2v");
      reset_cause = 5;
    }
    if (REG_PM_RCAUSE == PM_RCAUSE_POR){
      Serial.println("Normal power on reset");
      reset_cause = 0;
    }
    //End find reset cause

    Wire.begin();

      
    sodaq_wdt_safe_delay(5000);
    
    sodaq_wdt_enable(WDT_PERIOD_8X); //ENABLE WDT

    setup_rtc_debug();

    
    pinMode(MEDIR_CAP,OUTPUT);
    digitalWrite(MEDIR_CAP,LOW);
    pinMode(CAP_V,INPUT);
    analogReference(AR_DEFAULT);

    //LUX TINGS
    pinMode(LUX_ENABLE,OUTPUT);
    digitalWrite(LUX_ENABLE,LOW);
    
    pinMode(LUX_ANALOG,INPUT); //965,966 in a lit room

    pinMode(EN_SENSOR,OUTPUT);
    digitalWrite(EN_SENSOR,LOW);

    lora.setChannel(MULTI);
    // set datarate
    //lora.setDatarate(SF7BW125);
    lora.setDatarate(SF9BW125);
    //lora.setDatarate(SF10BW125);
    //lora.setDatarate(SF11BW125);
      if(!lora.begin())
      {
        Serial.println("Failed");
        Serial.println("Check your radio");
        while(true)
        {
          delay(1000);
          Serial.println("Failed");
          Serial.println("Check your radio");
          delay(1000);
        }
      }
      else{
        Serial.println("INIT LORA SUCCESS");
      }

     if(!apds.begin()){
      Serial.println("failed to initialize device! Please check your wiring.");
      delay(500); 
    }
    else Serial.println("Device initialized!");
    //enable color sensign mode
    apds.enableColor(true);

    int color = 0;
    color = get_color();
    last_color = color;

    send_lora(color);
    sodaq_wdt_reset();

    /*
    //stuff to allow falling interrupt
    // Set the XOSC32K to run in standby
    SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;

    // Configure EIC to use GCLK1 which uses XOSC32K 
    // This has to be done after the first call to attachInterrupt()
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | 
                        GCLK_CLKCTRL_GEN_GCLK1 | 
                        GCLK_CLKCTRL_CLKEN;
    
    */

    
    lora.sleep();
    systemSleep();
    
    
}

char buf;




/**
  Powers down all devices and puts the system to deep sleep.
*/
void systemSleep()
{

    //digitalWrite(EN_SENSOR,LOW);
  
    
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    
    lora.sleep();
    
    // Disable USB
    //USBDevice.detach();
    
    Serial.flush();
    Serial.end();
    USBDevice.detach();
    USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
    //rtc.standbyMode();

    // Disable systick interrupt
    
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    sodaq_wdt_disable();
    
    //BARRIER INSTRUCTION - WASNT HERE BEFORE
    __DSB();

    
    // SAMD sleep
    __WFI();

      
    // Enable systick interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    sodaq_wdt_enable(WDT_PERIOD_8X); //ENABLE WDT

    //ENABLE 3.3V PWR_SUP


}

int num_of_keep_alives = 0;

int timer = 0;

void loop(){
  if (sodaq_wdt_flag) {
    // Reset watchdog
    sodaq_wdt_reset();
    sodaq_wdt_flag = false;
    if ((~isButton)&&(~isClock)&&(~sodaq_wdt_flag)){
      systemSleep();
    }  
  }
  if (isClock){
    isClock = false;
    sodaq_wdt_reset();
    rtc.setAlarmSeconds((rtc.getAlarmSeconds() + 15) % 60); //reschedule the alarm for the next 12h -- rtc.setAlarmMinutes((rtc.getAlarmMinutes() + 10) % 60);
    
    int color = get_color();
    //String colors[4] = {"off", "red", "green", "blue"};
    //Serial.print("Light color = "); Serial.print(color); Serial.print(" - "); Serial.println(colors[color]);  
    delay(1000);
    if(color != last_color){
      if(!lora.begin()){
        Serial.println("Failed");
        Serial.println("Check your radio");   
      }else{
        send_lora(color);
      }
      last_color = color;
    }
    if ((~isButton)&&(~isClock)&&(~sodaq_wdt_flag)){
      systemSleep();
    }
  }
}
