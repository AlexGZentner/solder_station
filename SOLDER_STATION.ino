
//*********************************
//Board:        Arduino Nano 
//CPU:          ATMega168
//Programmer:   AVR ISP
//*********************************
#include <GyverEncoder.h>
#include <SoftPWM.h>
#include <SoftPWM_timer.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "MyLCD.h"
#include "MyTimer.h"

#define PHASE_PIN 2

#define TC_IN     A1
#define BUZZER    A0
#define ENC_CLK   3 
#define ENC_DT    4 
#define ENC_SW    5

#define  LCD_RS 6
#define  LCD_RW 7
#define  LCD_EN 8
#define  LCD_D4 9
#define  LCD_D5 10
#define  LCD_D6 11
#define  LCD_D7 12

#define  STATE_STOP  0x01
#define  STATE_SLEEP 0x02
#define  STATE_RUN   0x04

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  return result;
}

//rs, rw, enable, d4, d5, d6, d7
MyLCD mlcd(LCD_RS,LCD_RW,LCD_EN, LCD_D4,LCD_D5,LCD_D6,LCD_D7);
Encoder enc(ENC_CLK, ENC_DT, ENC_SW, TYPE1);

const double a1 = 2.508355E-2;
const double a2 = 7.860106E-8;
const double a3 = -2.503131E-10;
const double a4 = 8.315270E-14;

const double Wt_SCALE=0.0042775;
const double T_SCALE=50.0;
const uint16_t CONTROL_LENGTH=1000;
const uint16_t SLEEP_TIMEOUT_SEC=600;

unsigned int ADCValue;
double Voltage;
double Vcc,Vraw;
double Vin=0.0;
double Tact=0.0;
double Tprev=0.0;
uint16_t Tset=180;
uint16_t Toffset=20.0;

byte ScanCycle=0;
byte nSamples= 100;
byte PhaseOut=LOW;
byte FlipFlag=0;

uint16_t ActionTime=0;
uint16_t IdleTime=0;
uint16_t RunState=STATE_STOP;
byte Menu=0; 
uint16_t SleepTimeout=SLEEP_TIMEOUT_SEC; 

msec_TIMER PhaseDutyTimer;
msec_TIMER PhaseIdleTimer;
msec_TIMER OneSecondTimer;
msec_TIMER ControlTimer;

//PID constants
//////////////////////////////////////////////////////////
float kp = 2.0;   float ki = 0.04;   float kd = 11500.0;
//////////////////////////////////////////////////////////

float PID_p = 0.0; 
float PID_i = 0.0;  
float PID_d = 0.0;
float PID_error = 0.0;
float PID_error_prev= 0.0;
long PID_value = 0;

void setup()
{
  String str;
  Serial.begin(9600);
  EEPROM.get(sizeof(uint16_t)*0, Tset);
  EEPROM.get(sizeof(uint16_t)*1, Toffset);
  EEPROM.get(sizeof(uint16_t)*2, nSamples);
  mlcd.begin();
  ActionTime=0;  IdleTime=CONTROL_LENGTH;
  RunState=STATE_RUN;
  Menu=0;
  SoftPWMBegin();
  SoftPWMSet(BUZZER, 0);
  pinMode(PHASE_PIN,OUTPUT);
}

void loop()
{
 String str;
 ScanCycle++;
 Vcc = readVcc();
 Vraw= analogRead(TC_IN);
 Vin+=(((Vraw*Vcc)/ 1024.0));

 TIMER_set(&OneSecondTimer,1000);
 TIMER_set(&PhaseDutyTimer,ActionTime);
 TIMER_set(&PhaseIdleTimer,IdleTime);
 TIMER_set(&ControlTimer,ActionTime+IdleTime);

 if (ScanCycle>=nSamples)
 {
   Vin=double((Vin/ScanCycle)/1000);
   Tact=((Vin/0.032)*10.0)+Toffset;
  // Tact=Toffset + (Vin*(a1 + Vin*(a2 + Vin*(a3+a4))))*10;
  // Serial.println("------------------------------");
  // Serial.print("Vin: "); Serial.print(Vin,4); Serial.print(" Tact: "); Serial.println(Tact,4);
   ScanCycle=0; Vin=0.0;
   mlcd.clear(); 
  
 }
 if ( TIMER_state(&OneSecondTimer) &&  (RunState& STATE_RUN) ) 
  { 
    if (SleepTimeout<=5) 
    { 
      Menu=2;
      FlipFlag=!FlipFlag;
      if (SleepTimeout<=0) 
      { RunState=STATE_SLEEP; Menu=0; SleepTimeout=SLEEP_TIMEOUT_SEC;FlipFlag=0;}
    }
    SoftPWMSetPercent(BUZZER, FlipFlag*200);
    SleepTimeout--;
    TIMER_reset(&OneSecondTimer);
}

 if ( TIMER_state(&ControlTimer) && !(RunState & STATE_STOP) ) 
 {   
   PID_error = ((RunState & STATE_SLEEP) ? 150 : Tset ) - Tact;    
   PID_p = kp * PID_error;                                   //Calculate the P value
   PID_i = PID_i + (ki * PID_error);                        //Calculate the I value 
   PID_d = kd*((PID_error - PID_error_prev)/CONTROL_LENGTH);     //Calculate the D value 
   PID_value = PID_p + PID_i + PID_d;                            //Final total PID value is the sum of P + I + D
   PID_error_prev = PID_error;
   PID_value > CONTROL_LENGTH/2 ?  PID_value = CONTROL_LENGTH/2 : PID_value;  
   ActionTime=(PID_value > 0 ? PID_value : 0); IdleTime=CONTROL_LENGTH-PID_value;
   //PID_value < 0 ? PID_value = 0 : PID_value;
//    Serial.println("------------------------------");
//    Serial.print("Tset: "); Serial.println(Tset); Serial.print(" Tact: ");
//    Serial.print("Sleep: "); Serial.println(SleepTimeout);
//    Serial.println("------------------------------");
//    Serial.print("Tset: "); Serial.println(Tset); Serial.print(" Tact: "); Serial.println(Tact); Serial.print(" ERR: "); Serial.println(PID_error);
//   Serial.print("P: "); Serial.print(PID_p); Serial.print(" I: "); Serial.print(PID_i); Serial.print(" D: "); Serial.print(PID_d); Serial.println("   ");
//   Serial.print("ACT: "); Serial.print(ActionTime); Serial.print(" IDL: "); Serial.print(IdleTime);
//    Serial.println();
  
   TIMER_reset(&ControlTimer);
 };
 if ( (TIMER_state(&PhaseIdleTimer) && PhaseOut==LOW) )
 {   PhaseOut=HIGH; TIMER_reset(&PhaseDutyTimer);  }
 if ( (TIMER_state(&PhaseDutyTimer) && PhaseOut==HIGH) )
 {   PhaseOut=LOW; TIMER_reset(&PhaseIdleTimer);  }
  
 if ((RunState & STATE_STOP) || Tact>=450.0)
  digitalWrite(PHASE_PIN,LOW);
 else
  digitalWrite(PHASE_PIN,PhaseOut);

  enc.tick();
   if (enc.isRight())
   { 
    Tset>=450 ? Tset=450 : Tset+=1; 
    SleepTimeout=SLEEP_TIMEOUT_SEC;  Menu=0;
    FlipFlag=0;
     //isTimeUp(&DisplayUpdateTimer,0);
     //inpSetTemp--;
   }
   if (enc.isLeft()) 
   {
    Tset<=150 ? Tset=150 : Tset-=1; 
    SleepTimeout=SLEEP_TIMEOUT_SEC;   Menu=0;
    FlipFlag=0;
     //isTimeUp(&DisplayUpdateTimer,0);
     //inpSetTemp--;
   }
   if (enc.isClick() )
   {
    RunState = (RunState & STATE_RUN) ? STATE_SLEEP : STATE_RUN; PID_p=0; PID_i=0; PID_d=0; ActionTime=0; IdleTime=CONTROL_LENGTH;
    SleepTimeout=SLEEP_TIMEOUT_SEC;   Menu=0;
    FlipFlag=0;
     //isTimeUp(&DisplayUpdateTimer,0);
     //inpSetTemp--;
   }
   if (enc.isHolded() )
   {
     Menu=1;
     SleepTimeout=SLEEP_TIMEOUT_SEC;
     //isTimeUp(&DisplayUpdateTimer,0);
     //inpSetTemp--;
   }
/*
 if(TIMER_state(&keyDelayTimer))
 {
  key_code=!digitalRead(KEY_OK) | (!digitalRead(KEY_UP)<<1) | (!digitalRead(KEY_DOWN)<<2);
  switch (key_code)
  {
   case 0:
    keyDelay=250; key_hold=0;
    if (key_prev==1)
    {
     if (Menu==0)
     { Stop=!Stop; PID_p=0; PID_i=0; PID_d=0; ActionTime=0; IdleTime=CONTROL_LENGTH; key_prev=0; }
     if (Menu==2)
     { Menu=3; key_prev=0; break; }
     if (Menu==3)
     { Menu=4; key_prev=0; break; }
     if (Menu==4)
     { Menu=2; key_prev=0; break; }
    }
   break;
   case 1:
    key_prev=1; key_hold++;
    if (key_hold>=5)
     {Menu=1; Stop=true; PID_p=0; PID_i=0; PID_d=0; ActionTime=0; IdleTime=CONTROL_LENGTH;}
   break;
   case 6:
    if (Menu==0)
     Menu=2;              
    else
     Menu=0;
   break;
   case 2:
    if (Menu==0 || Menu==2) {   Tset>=420 ? Tset=420 : Tset+=1; }
    if (Menu==3) {   Toffset>=40 ? Toffset=40 : Toffset+=1; }
    if (Menu==4) {   nSamples>=250 ? nSamples=250 : nSamples+=1; }
    keyDelay>=10 ? keyDelay-=10: keyDelay=10;
   break;
   case 4:
    if (Menu==0 || Menu==2) {  Tset<=150 ? Tset=150 : Tset-=1; }
    if (Menu==3) {   Toffset<=1 ? Toffset=0 : Toffset-=1; }
    if (Menu==4) {   nSamples<=10 ? nSamples=10 : nSamples-=1; }
    keyDelay>=10 ? keyDelay-=10: keyDelay=10;
   break;
  }
  TIMER_reset(&keyDelayTimer);
 }
*/ 
 switch (Menu)
 {
  case 0:
   if ( RunState & STATE_STOP )
    str=SYM_OFF;
   else if(RunState & STATE_SLEEP)
    str=SYM_LOOP;
   else
    str=SYM_HEAT;
    
   str+=int(Tact); str+=SYM_CELSIUS; str+=' ';
   str+=SYM_THERMO; str+=Tset; str+=SYM_CELSIUS;str+=' ';
   str+=SYM_CLOCK; str+=ActionTime;str+="  ";
  break;
  case 1:
   EEPROM.put(sizeof(uint16_t)*0, Tset);
   EEPROM.put(sizeof(uint16_t)*1, Toffset);
   EEPROM.put(sizeof(uint16_t)*2, nSamples);
   str=" EEPROM saved!  ";
   mlcd.SetLCDText(str);
   mlcd.Prn(0); 
   delay(1500);
   Menu=0; 
  break;
  case 2:
    str="...zZz in: "; str+=SleepTimeout; str+=" s.  ";
   //str=SYM_LOOP; str+=" ";
   //str+="Tset:"; str+=Tset; str+=SYM_CELSIUS;
   //str+="      ";
  break;
  case 3:
   str=SYM_LOOP; str+=" ";
   str+="Tofs:"; str+=Toffset; str+=SYM_CELSIUS;
   str+="      ";
  break;
  case 4:
   str=SYM_LOOP; str+=" ";
   str+="Sampl:"; str+=nSamples; 
   str+="      ";
  break;
 }
 mlcd.SetLCDText(str);
 mlcd.Prn(0);    

}
