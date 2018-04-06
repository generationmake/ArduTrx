/* software for arduino DRA shield
 * bernhard@generationmake.de
 *
 * Version 0.1   - 16.05.2016 - initial version
 * Version 0.2   - 19.05.2016 - corrected frequency string to DRA818
 *                            - set PD, H/L and PTT to definded levels
 *                            - added 1750 Hz tone
 *                            - added rx/tx split from 145.600 to 145.800
 * Version 0.3   - 25.03.2018 - corrected 1750 Hz tone: now it stops after releasing the select button
 *                            - added boot up message with version
 *                            - added callsign
 *                            - updated menu and selection of options
 *                            - added software switch to change power level
 *                            - display rx/squelch in display (* at last position)
 * Version 0.4   - 27.03.2018 - added power level switch - by Mathias Metzner DH7AHO
 *                            - added defines for input and output pins
 * Version 0.5   - 04.04.2018 - added functions to store data in eeprom
 *                            - simplified encoder switch functions
 *                            - pullups on encoder inputs
 *                            - restore factory settings when encoder pressed during start up
 * Version 0.6   - 06.04.2018 - added simple menu
 *                            - menu option for factory setting
 *                            - filter can be set
 */

#define MY_CALLSIGN "ArduTrx"            // callsign here will display on line 1 

#include <LiquidCrystal.h>

// define inputs and outputs
#define IN_SQ   2
#define OUT_MIC 3
#define OUT_PTT 11
#define OUT_PD  12
#define OUT_H_L 13
// define encoder pins
#define IN_encoder0PinA  18
#define IN_encoder0PinB  17
#define IN_encoder0PinSW 19
// define menu
#define MENU_LENGTH 6

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//user parameters, will be stored in eeprom
struct userparameters
{
  byte ardutrx_version;     // version identifier
  int encoder0Pos = 11654;      // start frequency 145.675MHz
  byte vol=5;                // volume level
  byte sql=5;                // squelch level
  byte power_level=0;       // power level
  byte ctcss=10;            // ctcss 67, 71.9, 74.4, 77, 79.7, 82.5, 85.4, 88.5, 91.5,  94.8, 97.4, 100, 103.5, 107.2, 110.9, 114.8, 118.8, 123,   127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9,  173.8, 179.9, 186.2, 192.8, 203.5, 210.7, 218.1, 225.7, 233.6,241.8, 250.3
  byte filter_pre_de_emph=0;
  byte filter_highpass=0;
  byte filter_lowpass=0;
};

struct userparameters u;

#include <EEPROM.h>   // eeprom library for settings

//variables for transceiver
int update=1; // update of frequency and suelch necessary
int update_filter=1; // update of filter settings necessary
int sel=0;  // pointer for menu
byte menu_in=0;  // in menu?
unsigned long last_settings_update;   // variable to store when setting were changed last.

// read the buttons
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor 
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 250)  return btnUP; 
  if (adc_key_in < 450)  return btnDOWN; 
  if (adc_key_in < 650)  return btnLEFT; 
  if (adc_key_in < 850)  return btnSELECT;  

  // For V1.0 comment the other threshold and use the one below:
/*
  if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 195)  return btnUP; 
  if (adc_key_in < 380)  return btnDOWN; 
  if (adc_key_in < 555)  return btnLEFT; 
  if (adc_key_in < 790)  return btnSELECT;   
*/

  return btnNONE;  // when all others fail, return this...
}

// Install Pin change interrupt for a pin, can be called multiple times
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

//set frequency and squelch of dra818 
void send_dra(char *frxbuffer, char *ftxbuffer, int squ, byte ctcss)
{
  Serial.print("AT+DMOSETGROUP=0,");         // begin message
  Serial.print(ftxbuffer);
  Serial.print(",");
  Serial.print(frxbuffer);
  Serial.print(",00");
  if(ctcss<10) Serial.print("0");   // arduino generates no leading zeros
  Serial.print(ctcss);            // print ctcss
  Serial.print(",");    
  Serial.print(squ);
  Serial.println(",0000");
}

//set volume of dra818
void send_dravol(int vol)
{
  Serial.print("AT+DMOSETVOLUME=");         // begin message
  Serial.println(vol);
}
//set filter of dra818
void send_drafilter(byte pre_de_emph, byte highpass, byte lowpass)
{
  Serial.print("AT+SETFILTER=");         // begin message
  Serial.print(pre_de_emph);
  Serial.print(",");
  Serial.print(highpass);
  Serial.print(",");
  Serial.println(lowpass);
}
// set output power level of dra818
void set_power_level(byte level)
{
  if(level==1) digitalWrite(OUT_H_L,LOW); // 1 W
  else digitalWrite(OUT_H_L,HIGH); // 0,5 W
}

// display and menu routines
//set cursor to the right menu point
void display_cursor(int sel)
{
  if(sel==3) lcd.setCursor(13,1);       // menu entry
  else if(sel==2) lcd.setCursor(10,1);  // power level
  else if(sel==1) lcd.setCursor(7,1);   // squelch
  else lcd.setCursor(3,1);              // volume
}
void display_power_level(byte level)
{
  lcd.setCursor(9,1);  
  if(level==1) lcd.print("Hi");
  else lcd.print("Lo");  
}
void display_main_screen(void)
{
  lcd.clear();  // clear display
  lcd.setCursor(0,0);
  lcd.print(MY_CALLSIGN); // print my callsign
  lcd.setCursor(0,1);
  lcd.print("VOL  SQ  Lo  M"); // print menu in second line
  lcd.setCursor(7,1);
  lcd.print(u.sql);    // display squelch
  lcd.setCursor(3,1);
  lcd.print(u.vol);    // display volume
  display_power_level(u.power_level);  // display power level
  display_cursor(sel);   // position cursor for menu  
}
//menu functions
void display_menu(byte action)
{
  static byte menu_pointer=0; // pointer for active menu
  static byte menu_sub=0; // submenu active
  if(action==1) // right
  {
    if(menu_sub==0) menu_pointer++;
    else    // submenu active
    {
      if(menu_pointer==0) // ctcss
      {
        if(u.ctcss<38)
        {
          u.ctcss++;
          update=1;
        }
      }
      if(menu_pointer==1) // filter PRE/DE-EMPH
      {
        u.filter_pre_de_emph=1;
        update_filter=1;
      }
      if(menu_pointer==2) // filter highpass
      {
        u.filter_highpass=1;
        update_filter=1;
      }
      if(menu_pointer==3) // filter lowpass
      {
        u.filter_lowpass=1;
        update_filter=1;
      }
      if(menu_pointer==4) reset_factory_settings(); // factory settings
    }
  }
  if(action==2) // left
  {
    if(menu_sub==0) menu_pointer+=(MENU_LENGTH-1);
    else    // submenu active
    {
      if(menu_pointer==0) // ctcss
      {
        if(u.ctcss>0)
        {
          u.ctcss--;
          update=1;
        }
      }
      if(menu_pointer==1) // filter PRE/DE-EMPH
      {
        u.filter_pre_de_emph=0;
        update_filter=1;
      }
      if(menu_pointer==2) // filter highpass
      {
        u.filter_highpass=0;
        update_filter=1;
      }
      if(menu_pointer==3) // filter lowpass
      {
        u.filter_lowpass=0;
        update_filter=1;
      }
      if(menu_pointer==4) reset_factory_settings(); // factory settings
    }
  }
  if(action==3) // up
  {
    menu_sub=0;   // disable submenu
    if(menu_pointer==5) menu_in=0;    // back to main screen
  }
  if(action==4) // down
  {
    menu_sub=1;   // enable sub menu
    if(menu_pointer==5) menu_in=0;    // back to main screen
  }
  menu_pointer%=MENU_LENGTH;
  if(menu_in==1)    // if menu is active
  {
    lcd.clear();
    lcd.setCursor(0,0);
    if(menu_pointer==0) lcd.print("CTCSS"); // print menu line 1
    if(menu_pointer==1) lcd.print("Filt PRE/DE-EMPH"); // print menu line 1
    if(menu_pointer==2) lcd.print("Filter Highpass"); // print menu line 1
    if(menu_pointer==3) lcd.print("Filter Lowpass"); // print menu line 1
    if(menu_pointer==4) lcd.print("factory settings"); // print menu line 1
    if(menu_pointer==5) lcd.print("back to main"); // print menu line 1
    if(menu_sub==1)
    {
      lcd.setCursor(0,1); // print menu line 2 if submenu is active
      if(menu_pointer==0) lcd.print(u.ctcss);
      if(menu_pointer==1) lcd.print(u.filter_pre_de_emph);
      if(menu_pointer==2) lcd.print(u.filter_highpass);
      if(menu_pointer==3) lcd.print(u.filter_lowpass);
      if(menu_pointer==4) lcd.print("press right");
    }
  }
  else      // gi back to main screen
  {
    menu_pointer=0;   // reset menupointers
    menu_sub=0;
    update=1;     // to display frequency
    display_main_screen();  // show main screen
  }
}

void factory_settings()
{
  EEPROM.put(0,u);  // save all user parameters to EEprom
  delay(1000);
}

void reset_factory_settings()
{
  u.ardutrx_version=0;
  EEPROM.put(0,u);  // destroy version; after reset default values will be used
  lcd.setCursor(0,1);
  lcd.print("press reset");
  while(1);
}

void setup()
{
  u.ardutrx_version=1;
// set pins
  pinMode(IN_SQ,INPUT_PULLUP); // SQ
  pinMode(OUT_PTT,OUTPUT); // PTT low=rx, high=tx
  pinMode(OUT_PD,OUTPUT); // PD low=sleep, high=normal
  pinMode(OUT_H_L,OUTPUT); // H_L low=1 W, high=0.5 W
  digitalWrite(OUT_PTT,LOW); // rx
  digitalWrite(OUT_PD,LOW); // normal
  digitalWrite(OUT_H_L,HIGH); // 0.5 W
  pinMode (IN_encoder0PinA,INPUT_PULLUP);  // encoder input pins with pullup to avoid problems when encoder no correctly connected
  pinMode (IN_encoder0PinB,INPUT_PULLUP);
  pinMode (IN_encoder0PinSW,INPUT_PULLUP);

// init serial
  Serial.begin(9600); // start serial for communication with dra818
// init display
  lcd.begin(16, 2);  // start display library
  lcd.setCursor(0,0);
  lcd.print("  ArduTrx - 0.6 "); // print boot message 1
  lcd.setCursor(0,1);
  lcd.print("   06.04.2018   ");  // print boot message 2
  delay(2000);    // wait 2 seconds

// check version number of eeprom content and reset if old
  byte old_version;
  EEPROM.get(0, old_version); // previous sketch version
  if (!digitalRead(IN_encoder0PinSW) || (old_version != u.ardutrx_version)) {
    lcd.setCursor(0,1);
    lcd.print("setting defaults");  // print boot message 2
    delay(2000);    // wait 2 seconds
    factory_settings();
  }

  EEPROM.get(0,u);    // get EEprom settings

  display_main_screen();  // show main screen
  lcd.blink();    // enable blink funktion of cursor

//enable interrupts
  pciSetup(A3);  // enable pin change interrupt for encoder
  pciSetup(A4);
}
 
void loop()
{
  char frxbuffer[10];  // buffer for rx frequency string
  char ftxbuffer[10];  // buffer for tx frequency string
  int freqtx,freqrx;
  int freqa,freqb;
  int updatevol=0;  // set to 1 when update of volume necessary
  int sqin;   // variable for squelch input
  static int sqin_old=0;  // variable for squelch input to store old value
  int press = 0;
  int Merker = 0;

// write settings to eeprom
  if(millis()>(last_settings_update+5000))  // 5 sencons past with no user action
  {
    EEPROM.put(0,u);  // save all user parameters to EEprom  - checks if data in eeprom is the same, so no risc to destroy eeprom
  }

// squelch
  sqin=digitalRead(IN_SQ);    // read squelch input
  if(sqin!=sqin_old)    // compare if squelch has changed
  {
    sqin_old=sqin;      // store new value of squelch
    lcd.setCursor(15,1);    // go to last position of display
    if(sqin) lcd.print(" ");  // print blank if no rx
    else lcd.print("*");      // print * if rx
    display_cursor(sel);         // set cursor to menu position
  }

//encoder push button
  press = digitalRead(IN_encoder0PinSW);
  if((press == 0) && (Merker == 0))
  {
    if(u.power_level==0) u.power_level=1;    // output = 1W
    else u.power_level=0;    // output = 0.5W
    last_settings_update=millis();  // trigger update
    set_power_level(u.power_level); // send it to dra818
    Merker = 1;
    display_power_level(u.power_level);  // display power level
    display_cursor(sel);         // set cursor to menu position
    delay(10);   
  }
  if((press == 1) && (Merker == 1))
  {
    Merker = 0;
    delay(10);
  }

  lcd_key = read_LCD_buttons();  // read the buttons

// menu
  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:               // go to next position
      {
        if(menu_in==1)          // if in menu

        {
          display_menu(1);       // send it direct to menu function
        }
        else
        {
          sel++;
          sel=sel%4;        // our menu has 4 points
          display_cursor(sel);
        }
        delay(200);
        break;
      }
    case btnLEFT:                // go to previous position
      {
        if(menu_in==1)          // if in menu
        {
          display_menu(2);       // send it direct to menu function
        }
        else
        {
          sel+=3;
          sel=sel%4;        // our menu has 4 points
          display_cursor(sel);
        }
        delay(200);
        break;
      }
    case btnUP:       // button up
      {
        if(menu_in==1)          // if in menu
        {
          display_menu(3);       // send it direct to menu function
        }
        else
        {
          if(sel==0)    // volume
          {
            if(u.vol<8)u.vol++;
            updatevol=1;
          }
          if(sel==1)    // squelch
          {
            if(u.sql<8)u.sql++;
            update=1;
          }
          if(sel==2)    // power level
          {
            u.power_level=1;    // output = 1W
            last_settings_update=millis();  // trigger update
            set_power_level(u.power_level); // send it to dra818
            display_power_level(u.power_level);  // display power level
            lcd.setCursor(10,1);  
          }
          if(sel==3)    // menu
          {
            menu_in=1;
            display_menu(0);
          }
        }
        delay(200);
        break;
      }
    case btnDOWN:   // button down
      {
        if(menu_in==1)          // if in menu
        {
          display_menu(4);       // send it direct to menu function
        }
        else
        {
          if(sel==0)    // volume
          {
            if(u.vol>1)u.vol--;
            updatevol=1;
          }
          if(sel==1)    // squelch
          {
            if(u.sql>0)u.sql--;
            update=1;
          }
          if(sel==2)    // power level
          {
            u.power_level=0;    // output = 0.5W
            last_settings_update=millis();  // trigger update
            set_power_level(u.power_level); // send it to dra818
            display_power_level(u.power_level);  // display power level
            lcd.setCursor(10,1);  
          }
          if(sel==3)    // menu
          {
            menu_in=1;
            display_menu(0);
          }
        }
        delay(200);
        break;
      }
    case btnSELECT:   // select button
      {
        if(menu_in==1)          // if in menu
        {
          display_menu(2);       // send it direct to menu function
          delay(200);
        }
        else
        {
          digitalWrite(OUT_PTT, HIGH);  // enable PTT
          tone(OUT_MIC,1750);   // enable 1750 Hz tone
        }
        break;
      }
    case btnNONE:
      {
        noTone(OUT_MIC);    // disable 1750 Hz tone again
        digitalWrite(OUT_PTT, LOW);  // disable PTT
        break;
      }
  }

  if(update==1)    // update frequency or squelch
  {
    update=0;
    last_settings_update=millis();  // trigger update
    //limit encoder values
    freqrx=u.encoder0Pos;
    if(freqrx>13920) freqrx=13920;  // 174.0000 MHz
    if(freqrx<10720) freqrx=10720;  // 134.0000 MHz
     
    lcd.setCursor(7,0);
    if((freqrx>=11648)&&(freqrx<=11664))   // Relais 145.6000 - 145.8000 MHz
    {
      freqtx=freqrx-48;   // set tx frequency 600 kHz lower
      lcd.print("-");     // display -
    }
    else 
    {
      freqtx=freqrx;    // set tx frequency = rx frequency
      lcd.print(" ");
    }

    freqa=(freqrx/80);  // frequency integral part
    freqb=(freqrx%80)*125;  // frequency fractional part
    sprintf(frxbuffer,"%03i.%04i",freqa,freqb);  // generate frequency string
    freqa=(freqtx/80);  // frequency integral part
    freqb=(freqtx%80)*125;  // frequency fractional part
    sprintf(ftxbuffer,"%03i.%04i",freqa,freqb);  // generate frequency string
    lcd.setCursor(8,0);
    lcd.print(frxbuffer);      // display frequency
    send_dra(frxbuffer,ftxbuffer,u.sql,u.ctcss);    // update volume on dra818
    lcd.setCursor(7,1);
    lcd.print(u.sql);    // display squelch
    display_cursor(sel);     // display menu
  }
  if(updatevol==1)    // update volume
  {
    updatevol=0;
    last_settings_update=millis();  // trigger update
    send_dravol(u.vol);  // update volume on dra818
    lcd.setCursor(3,1);
    lcd.print(u.vol);    // display volume
    display_cursor(sel);   // display menu
  }
  if(update_filter==1)    // update filter
  {
    update_filter=0;
    last_settings_update=millis();  // trigger update
    send_drafilter(u.filter_pre_de_emph,u.filter_highpass, u.filter_lowpass);  // update filter settings on dra818
  }
}

ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  static int encoder0PinALast = HIGH;
  static int encoder0PinBLast = HIGH;
  int na = LOW;
  int nb = LOW;

  na = digitalRead(IN_encoder0PinA);
  nb = digitalRead(IN_encoder0PinB);
  if(encoder0PinBLast!=nb)
  {
    if(nb==HIGH)
    {
      if(na==HIGH) u.encoder0Pos++;
      else u.encoder0Pos--;
    }
    else
    {
      if(na==HIGH) u.encoder0Pos--;
      else u.encoder0Pos++;
    }
    update=1;
  }
  encoder0PinALast = na;
  encoder0PinBLast = nb;
}  

