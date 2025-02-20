#include <LiquidCrystal.h>

#define SYM_LOOP    ( (char)1 )
#define SYM_OFF     ( (char)2 )
#define SYM_THERMO  ( (char)3 )
#define SYM_HEAT    ( (char)4 )
#define SYM_CELSIUS ( (char)5 )
#define SYM_CLOCK   ( (char)6 )
class MyLCD : public LiquidCrystal
{
  private:
   const uint8_t _nTOTAL_ROWS = 16;
   String  _TEXT;
   int _nCursorPos;
   int _nTEXTPos;
   uint8_t _nLines;
   uint8_t _nRows;
   byte _ArrowUp[8] =   {0b00100,0b01110,0b11111,0b00000,0b00000,0b00000,0b00000,0b00000};
   byte _ArrowDown[8] = {0b00000,0b00000,0b00000,0b00000,0b11111,0b01110,0b00100,0b00000};
   byte _Loop[8] = {B00000,B01111,B01010,B01010,B01010,B11110,B00000,B00000};
   byte _Off[8] = {B00000,B00000,B00000,  B00000,  B00000,  B00000,B01110,B11111};
   byte _Thermometer[8] = {B00100,B01010,  B01010,B01110,B11111,B11111,B01110,B00000};
   byte _Celsius[8] = {B01000,B10100,B01000,B00011,B00100,B00100,B00011,B00000};
   byte _Heat[8] = {B10010,B01001,B10010,B01001,B10010,B00000,B01110,B11111};
   byte _Clock[8]={0b00000,0b01110,0b10011,0b10101,0b10001,0b01110,0b00000,0b00000};

  public:
    MyLCD(uint8_t rs, uint8_t rw, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) : LiquidCrystal (rs,rw,enable,d0,d1,d2,d3) 
    {_TEXT.reserve(256); _nCursorPos=0; _nRows=8 ; _nLines=2; };
    
  public: void begin() 
    { LiquidCrystal::begin(8,2); 
      this->createChar(1,_Loop);
      this->createChar(2,_Off);
      this->createChar(3,_Thermometer);
      this->createChar(4,_Heat);
      this->createChar(5,_Celsius);
      this->createChar(6,_Clock);
    };

  public: void SetLCDText(String str)
    { _TEXT=str;};
    
  private: void setCursor(uint8_t Row)
    {
      if (Row >= _nRows )
       LiquidCrystal::setCursor(Row-_nRows,1);
      else
        LiquidCrystal::setCursor(Row,0);
    };

  public: void Prn(int8_t Row=0)
    {
      _nCursorPos=Row;
      if (Row<0)
      {  _nTEXTPos=abs(Row); Row=0; }
      else
      {_nTEXTPos=0;}
      for (int i=_nTEXTPos; i<_TEXT.length(); i++)
      {
        if (Row>_nTOTAL_ROWS) break;
        setCursor(Row);
        LiquidCrystal::write(_TEXT[i]);  
        Row++;
      }
    };
  public: int moveRight()
    {
      if (_nCursorPos >= _nTOTAL_ROWS) return(-255);
      for(int i=0; i<=_nCursorPos;i++)
      { setCursor(i); LiquidCrystal::write(" ");}
      _nCursorPos++;
      Prn(_nCursorPos);
      return(_nCursorPos);
    };
    
  public: int moveLeft()
    {
      int tmp=-1*_TEXT.length();
      if ( _nCursorPos<= tmp ) {return(-255); }
      for(int i=(_nCursorPos+_TEXT.length()-1); i<=_nTOTAL_ROWS;i++)
      { setCursor(i); LiquidCrystal::write(" ");    
      }
      _nCursorPos--;
      Prn(_nCursorPos);
      return(_nCursorPos);
    };
};
