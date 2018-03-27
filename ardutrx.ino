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
 * Version 0.4   - 27.03.2018 - added power level switch - by mathias.metzner@freenet.de
 *                            - added defines for input and output pins
 */

#define MY_CALLSIGN "ArduTrx"            // callsign here will display on line 1 

#include <LiquidCrystal.h>

int press = 0;
int Merker = 0;

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

int encoder0Pos = 0;

//variables for transceiver
int update=1; // update of frequency and suelch necessary
int sel=0;  // pointer for menu
int vol=5;
int sql=5;

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
void send_dra(char *frxbuffer, char *ftxbuffer, int squ)
{
  Serial.print("AT+DMOSETGROUP=0,");         // begin message
  Serial.print(ftxbuffer);
  Serial.print(",");
  Serial.print(frxbuffer);
  Serial.print(",0010,");    // CTCSS-Tone 94.8Hz
//  Serial.print(",0000,");
  Serial.print(squ);
  Serial.println(",0000");
}

//set volume of dra818
void send_dravol(int vol)
{
  Serial.print("AT+DMOSETVOLUME=");         // begin message
  Serial.println(vol);
}
//set cursor to the right menu point
void setcursor(int sel)
{
  if(sel==2) lcd.setCursor(10,1);       // power level
  else if(sel==1) lcd.setCursor(7,1);   // squelch
  else lcd.setCursor(3,1);              // volume
}

void setup()
{
// set pins
  pinMode(IN_SQ,INPUT_PULLUP); // SQ
  pinMode(OUT_PTT,OUTPUT); // PTT low=rx, high=tx
  pinMode(OUT_PD,OUTPUT); // PD low=sleep, high=normal
  pinMode(OUT_H_L,OUTPUT); // H_L low=1 W, high=0.5 W
  digitalWrite(OUT_PTT,LOW); // rx
  digitalWrite(OUT_PD,LOW); // normal
  digitalWrite(OUT_H_L,HIGH); // 0.5 W
  pinMode (IN_encoder0PinA,INPUT);  // set encoder pins to interrupt
  pinMode (IN_encoder0PinB,INPUT);
  pinMode (IN_encoder0PinSW,INPUT);

// init display
  Serial.begin(9600); // start serial for communication with dra818
  lcd.begin(16, 2);  // start display library
  lcd.setCursor(0,0);
  lcd.print("  ArduTrx - 0.4 "); // print boot message 1
  lcd.setCursor(0,1);
  lcd.print("   27.03.2018   ");  // print boot message 2
  delay(2000);    // wait 2 seconds
  lcd.clear();  // clear display
  lcd.setCursor(0,0);
  lcd.print(MY_CALLSIGN); // print my callsign
  lcd.setCursor(0,1);
  lcd.print("VOL  SQ  Lo"); // print menu in second line
  lcd.setCursor(7,1);
  lcd.print(sql);    // display squelch
  lcd.setCursor(3,1);
  lcd.print(vol);    // display volume
  setcursor(sel);   // position cursor for menu
  lcd.blink();    // enable blink funktion of cursor

  encoder0Pos=11654;  // initial value for frequency 145.675MHz
//  encoder0Pos=11600;  // initial value for frequency
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

// squelch
  sqin=digitalRead(IN_SQ);    // read squelch input
  if(sqin!=sqin_old)    // compare if squelch has changed
  {
    sqin_old=sqin;      // store new value of squelch
    lcd.setCursor(15,1);    // go to last position of display
    if(sqin) lcd.print(" ");  // print blank if no rx
    else lcd.print("*");      // print * if rx
    setcursor(sel);         // set cursor to menu position
  }

//encoder push button
  press = digitalRead(IN_encoder0PinSW);
  if((press == 1) && (Merker == 0))
  {
    digitalWrite(OUT_H_L, HIGH);
    Merker = 1;
    lcd.setCursor(9,1);
    lcd.print("Lo");
    setcursor(sel);         // set cursor to menu position
    delay(10);   
  }
  if((press == 0) && (Merker == 1))
  {
    Merker = 2;
    delay(10);
  }
  if((press == 1) && (Merker == 2))
  {
    digitalWrite(OUT_H_L, LOW);
    Merker = 3;
    lcd.setCursor(9,1);
    lcd.print("Hi");
    setcursor(sel);         // set cursor to menu position
    delay(10);
  }
  if((press == 0) && (Merker == 3))
  {
    Merker= 0;
    delay (10);
  }

  lcd_key = read_LCD_buttons();  // read the buttons

// menu
  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:               // go to next position
      {
        sel++;
        sel=sel%3;        // our menu has 3 points
        setcursor(sel);
        delay(200);
        break;
      }
    case btnLEFT:                // go to previous position
      {
        sel+=2;
        sel=sel%3;        // our menu has 3 points
        setcursor(sel);
        delay(200);
        break;
      }
    case btnUP:       // button up
      {
        if(sel==0)    // volume
        {
          if(vol<8)vol++;
          updatevol=1;
        }
        if(sel==1)    // squelch
        {
          if(sql<8)sql++;
          update=1;
        }
        if(sel==2)    // power level
        {
          digitalWrite(OUT_H_L,LOW); // 1 W
          lcd.setCursor(9,1);  
          lcd.print("Hi");
          lcd.setCursor(10,1);  
        }
        delay(200);
        break;
      }
    case btnDOWN:   // button down
      {
        if(sel==0)    // volume
        {
          if(vol>1)vol--;
          updatevol=1;
        }
        if(sel==1)    // squelch
        {
          if(sql>0)sql--;
          update=1;
        }
        if(sel==2)    // power level
        {
          digitalWrite(OUT_H_L,HIGH); // 0.5 W
          lcd.setCursor(9,1);  
          lcd.print("Lo");
          lcd.setCursor(10,1);  
        }
        delay(200);
        break;
      }
    case btnSELECT:   // select button
      {
        digitalWrite(OUT_PTT, HIGH);  // enable PTT
        tone(OUT_MIC,1750);   // enable 1750 Hz tone
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
    //limit encoder values
    if(encoder0Pos>13920) encoder0Pos=13920;  // 174.0000 MHz
    if(encoder0Pos<10720) encoder0Pos=10720;  // 134.0000 MHz
     
    freqrx=encoder0Pos;
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
    send_dra(frxbuffer,ftxbuffer,sql);    // update volume on dra818
    lcd.setCursor(7,1);
    lcd.print(sql);    // display squelch
    setcursor(sel);     // display menu
  }
  if(updatevol==1)    // update volume
  {
    updatevol=0;
    send_dravol(vol);  // update volume on dra818
    lcd.setCursor(3,1);
    lcd.print(vol);    // display volume
    setcursor(sel);   // display menu
  }
}

ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  static int encoder0PinALast = LOW;
  static int encoder0PinBLast = LOW;
  int na = LOW;
  int nb = LOW;

  na = digitalRead(IN_encoder0PinA);
  nb = digitalRead(IN_encoder0PinB);
  if(encoder0PinBLast!=nb)
  {
    if(nb==HIGH)
    {
      if(na==HIGH) encoder0Pos++;
      else encoder0Pos--;
    }
    else
    {
      if(na==HIGH) encoder0Pos--;
      else encoder0Pos++;
    }
    update=1;
  }
  encoder0PinALast = na;
  encoder0PinBLast = nb;
}  

