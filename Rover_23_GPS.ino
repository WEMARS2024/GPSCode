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
float fMPS;
float fDegree;
float fAlt;
float fHDOP;

unsigned int uiInByte;
char strGPS[100];
char strValid[8]; 
char strLat[15];
char strLon[15];
char strMPS[6];
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
    fMPS = gps.speed.mps();
  }
  
  if (gps.course.isValid())
  {
    strValid[4] = '?';
    fDegree = gps.course.deg();
  }
  
  if (gps.altitude.isValid())
  {
    strValid[5] = '?';
    fAlt = gps.altitude.meters();
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


  if(uiInByte == 'g')
  {
     
     dtostrf( fLat, 3, 8, strLat);
     dtostrf( fLon, 3, 8, strLon);
     dtostrf( fMPS, 3, 2, strMPS);
     dtostrf( fDegree, 3, 2, strDegree);
     dtostrf( fAlt, 5, 2, strAlt);
     dtostrf( fHDOP, 2, 2, strHDOP);

    sprintf(strGPS,"%s,%i,%s,%s,%i,%s,%s,%s,%i,%s\n",strValid,uiTime,strLat,strLon,uiDate,strMPS,strDegree,strAlt,uiSatNumber,strHDOP);
    Serial.print(strGPS);
    strcpy(strGPS,"");
  }
 
}