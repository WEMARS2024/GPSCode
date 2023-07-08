#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 23, TXPin = 22;
static const uint32_t GPSBaud = 9600;


// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
#define BAUD_RATE 9600
EspSoftwareSerial::UART ss;


void setup()
{
  Serial.begin(115200);
  ss.begin(BAUD_RATE, EspSoftwareSerial::SWSERIAL_8N1, RXPin, TXPin);

    
}

//If bits are one then updated bit 0 is location,
unsigned int uiTime;
unsigned int uiDate;
unsigned int uiSatNumber;

float fLat;
float fLon;
float fKPH;
float fDegree;
float fAlt;
float fHDOP;

unsigned int uiInByte;
char strGPS[100];
char strValid[8]; 
char strLat[15];
char strLon[15];
char strKPH[6];
char strDegree[6];
char strAlt[10];
char strHDOP[6];



void loop()
{
   if (Serial.available() > 0)
   {
    // get incoming byte:
    uiInByte = Serial.read();
   }
  // Dispatch incoming characters
  while (ss.available() > 0)

    gps.encode(ss.read());
  
  
  if (gps.time.isValid())
  {
    if(gps.time.value() != 0)
    {
      strValid[0] = '1';
    }
    else
    {
      strValid[0] = '0';

    }
    uiTime = gps.time.value();
    
  }
  

  if (gps.location.isValid())
  {
    if((gps.location.lat() != 0.0)&&(gps.location.lng()))
    {
      strValid[1] = '1';
      
    }    
    else
    {
      strValid[1] = '0';
    }
    fLat = gps.location.lat();
    fLon = gps.location.lng();
  }
  
  if (gps.date.isValid())
  {
    if(gps.date.value() != 0)
    {
      strValid[2] = '1';
    }    
    else
    {
      strValid[2] = '0';
    }
    uiDate = gps.date.value();
  }
  
  if (gps.speed.isValid())
  {
    strValid[3] = '?';
    fKPH = gps.speed.value();
  }
  
  if (gps.course.isValid())
  {
    strValid[4] = '?';
    fDegree = gps.course.value();
  }
  
  if (gps.altitude.isValid())
  {
    strValid[5] = '?';
    fAlt = gps.altitude.value();
  }
  
  if (gps.satellites.isValid())
  {
    if(gps.satellites.value() != 0)
    {
     strValid[6] = '1';
    }    
    else
    {
      strValid[6] = '0';
    }
    uiSatNumber = gps.satellites.value();
  
  }
  
  if (gps.hdop.isValid())
  {
    if(gps.hdop.hdop() != 99.99)
    {
     strValid[7] = '1';
    }    
    else
    {
     strValid[7] = '0';
    }
    fHDOP = gps.hdop.hdop();
  }
  
  if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));


  if(uiInByte == 'G')
  {
     
     dtostrf( fLat, 3, 8, strLat);
     dtostrf( fLon, 3, 8, strLon);
     dtostrf( fKPH, 3, 2, strKPH);
     dtostrf( fDegree, 3, 2, strDegree);
     dtostrf( fAlt, 5, 2, strAlt);
     dtostrf( fHDOP, 2, 2, strHDOP);

    sprintf(strGPS,"%s,%i,%s,%s,%i,%d,%d,%d,%i,%d/n",strValid,uiTime,strLat,strLon,uiDate,strKPH,strDegree,strAlt,uiSatNumber,strHDOP);
    Serial.println(strGPS);
  }

  // if(Updated && 0x1)
  // { 
  //   Updated &= 0xFE;
  //   Serial.println("");
  //   Serial.print(F("Time = "));
  //   Serial.print(gps.time.value());
  // }
  // if(Updated && 0x2)
  // {
  //   Updated &= 0xFD;
  //   Serial.print(F(" ,Lat = "));
  //   Serial.print(gps.location.lat(), 8);
  //   Serial.print(F(" ,Long = "));
  //   Serial.print(gps.location.lng(), 8);
  // }
  // if(Updated && 0x4)
  // { 
  //   Updated &= 0xFB;
  //   Serial.print(F(" ,Date = "));
  //   Serial.print(gps.date.value());
  // }
  // if(Updated && 0x8)
  // {
  //   Updated &= 0xF7;
  //   Serial.print(F(" ,km/h = "));
  //   Serial.print(gps.speed.kmph());
  // }

  // if(Updated && 0x10)
  // {
  //   Updated &= 0xEF;
  //   Serial.print(F(" ,Deg = "));
  //   Serial.print(gps.course.deg());
  // }

  // if(Updated && 0x20)
  // {
  //   Updated &= 0xDF;
  //   Serial.print(F(" ,Alt Meters = "));
  //   Serial.print(gps.altitude.meters());
  // }
  // if(Updated && 0x40)
  // {
  //   Updated &= 0xBF;
  //   Serial.print(F(" ,Sat = "));
  //   Serial.print(gps.satellites.value());
  // }
  // if(Updated && 0x80)
  // {
  //   Updated &= 0x7F;
  //   Serial.print(F(" ,hdop = "));
  //   Serial.print(gps.hdop.hdop());
  // }
 
}