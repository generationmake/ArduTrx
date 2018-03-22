/* software for arduino DRA shield
 * bernhard@generationmake.de
 *
 * Version 0.1   - 16.05.2016 - initial version
 * Version 0.2   - 19.05.2016 - corrected frequency string to DRA818
 *                            - set PD, H/L and PTT to definded levels
 *                            - added 1750 Hz tone
 *                            - added rx/tx split from 145.600 to 145.800
 */

#include <LiquidCrystal.h>

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

// define encoder pins
int encoder0PinA = 18;
int encoder0PinB = 17;
int encoder0PinSW = 19;
int encoder0Pos = 0;
int update=1;
int sel=0;
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
  Serial.print(",0000,");
  Serial.print(squ);
  Serial.println(",0000");
}

//set volume of dra818
void send_dravol(int vol)
{
  Serial.print("AT+DMOSETVOLUME=");         // begin message
  Serial.println(vol);
}

void setup()
{
  Serial.begin(9600); 
  lcd.begin(16, 2);  // start display library
  lcd.setCursor(0,0);
  lcd.print("DRA818"); // print menu
  lcd.setCursor(0,1);
  lcd.print(">VOL   SQ ");
 
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(11,LOW);
  digitalWrite(12,LOW);
  digitalWrite(13,LOW);
  pinMode (encoder0PinA,INPUT);  // set encoder pins to interrupt
  pinMode (encoder0PinB,INPUT);
  encoder0Pos=11600;  // initial value for frequency
  pciSetup(A3);  // enable pin change interrupt for encoder
  pciSetup(A4);
}
 
void loop()
{
  char frxbuffer[10];  // buffer for rx frequency string
  char ftxbuffer[10];  // buffer for tx frequency string
  int freqtx,freqrx;
  int freqa,freqb;
  int updatevol=0;

  lcd.setCursor(0,1);            // move to the begining of the second line
  lcd_key = read_LCD_buttons();  // read the buttons

// menu
  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:               // set squelch
      {
        sel=1;
        lcd.print(" VOL  >SQ ");
        delay(200);
        break;
      }
    case btnLEFT:                // set volume
      {
        noTone(3);
        sel=0;
        lcd.print(">VOL   SQ ");
        delay(200);
        break;
      }
    case btnUP:
      {
        if(sel==0)
        {
          if(vol<8)vol++;
          updatevol=1;
        }
        if(sel==1)
        {
          if(sql<8)sql++;
          update=1;
        }
        delay(200);
        break;
      }
    case btnDOWN:
      {
        if(sel==0)
        {
          if(vol>1)vol--;
          updatevol=1;
        }
        if(sel==1)
        {
          if(sql>0)sql--;
          update=1;
        }
        delay(200);
        break;
      }
    case btnSELECT:
      {
        tone(3,1750);
        break;  // no action for select
      }
    case btnNONE:
      {
        break;
      }
  }

  lcd.setCursor(9,1);
  lcd.print(sql);    // display squelch
  lcd.setCursor(4,1);
  lcd.print(vol);    // display volume

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
      freqtx=freqrx-48;
      lcd.print("-");
    }
    else 
    {
      freqtx=freqrx;
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
    send_dra(frxbuffer,ftxbuffer,sql);
  }
  if(updatevol==1)    // update volume
  {
    updatevol=0;
    send_dravol(vol);
  }
}

ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  static int encoder0PinALast = LOW;
  static int encoder0PinBLast = LOW;
  int na = LOW;
  int nb = LOW;

  na = digitalRead(encoder0PinA);
  nb = digitalRead(encoder0PinB);
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

