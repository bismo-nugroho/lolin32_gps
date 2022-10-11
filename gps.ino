
#include <arduino.h>
#include <SoftwareSerial.h>

// GPS Port
#define RX_GPS 32
#define TX_GPS 33

//Serial GPS
SoftwareSerial gps_port;//(RX_GPS, TX_GPS);




float getLat(void);
float getLon(void);
String getDateTimeLoc(void);
String getDayName(void);



static const uint32_t GPSBaud = 9600;



byte nmea = 0;
byte lnmea = 0;
bool blink = false;


String datelocal = "";

char timeloc[9];
char dateloc[11];


int delaypush = 300;
int delayhold = 1500;

void getSaved(void);
void setTZ(void);


double timestartgps = 0;
double timefixgps = 0;

int contentflag = 0;
CircularBuffer<String, 15> contentsms;
CircularBuffer<String, 30> serialdata;

/*
  int signalstat = 0;
  int netstat = 0;
  String ipaddr = "";
  int inetstat = 0;
  bool result = 0;
  int atstat = 0;
  int initstat = -1;

  String atmsg = "";
  String flagat = "";
*/
byte speeds = 0;

//satellite logo
const unsigned char u8g_logo_sat[] = {
  0x04, 0x00, 0x0A, 0x00, 0x11, 0x00, 0x22, 0x00, 0xE4, 0x00, 0xF8, 0x00,
  0xF0, 0x01, 0x74, 0x02, 0x44, 0x04, 0x9D, 0x08, 0x01, 0x05, 0x07, 0x02
};



const unsigned char  logo_envelope[] = {
  0xff, 0xff, 0x0f, 0xff, 0xff, 0x0f, 0xff, 0xff, 0x0f, 0x03, 0x00, 0x0e, 0xe3, 0x3f, 0x0e,
  0xcb, 0x9f, 0x0e, 0xdb, 0xcf, 0x0e, 0x3b, 0xe7, 0x0e, 0x3b, 0xe0, 0x0e, 0x9b, 0xc8, 0x0e,
  0xcb, 0x9d, 0x0e, 0xe3, 0x3f, 0x0e, 0x03, 0x00, 0x0e, 0xff, 0xff, 0x0f, 0xff, 0xff, 0x0f,
  0xff, 0xff, 0x0f, 0xff, 0xff, 0x0f
};





const unsigned char  logo_start[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xf0, 0xff, 0x00, 0x06, 0x00, 0x80, 0xff, 0x0f, 0x30, 0x00, 0xff, 0xff, 0xff, 0xff, 0x01,
  0x00, 0xfe, 0xff, 0xcf, 0x07, 0x00, 0xf8, 0xff, 0xff, 0x3c, 0x00, 0xfc, 0xff, 0xff, 0xff,
  0x0f, 0x80, 0xff, 0xff, 0xff, 0x07, 0x00, 0xff, 0xff, 0xff, 0x3f, 0x00, 0xf8, 0xff, 0xff,
  0xff, 0x3f, 0xc0, 0x3f, 0xc0, 0xff, 0x07, 0xc0, 0xff, 0xff, 0xff, 0x3f, 0x00, 0xe0, 0xff,
  0xff, 0xff, 0x7f, 0xc0, 0x1f, 0x00, 0xfc, 0x07, 0xe0, 0xff, 0x07, 0xfc, 0x3f, 0x00, 0xc0,
  0xff, 0x03, 0xf0, 0xff, 0xe0, 0x3f, 0x00, 0xf0, 0x07, 0xf0, 0xff, 0x01, 0xc0, 0x3f, 0x00,
  0x80, 0xff, 0x03, 0xe0, 0xff, 0xe1, 0xff, 0x0f, 0xc0, 0x07, 0xf8, 0x7f, 0x00, 0x00, 0x3e,
  0x00, 0x80, 0xff, 0x03, 0xe0, 0xff, 0xe1, 0xff, 0xff, 0x1f, 0x07, 0xfc, 0x7f, 0x00, 0x00,
  0x38, 0x00, 0x80, 0xff, 0x03, 0xe0, 0xff, 0xe1, 0xff, 0xff, 0xff, 0x07, 0xfe, 0x3f, 0x00,
  0x00, 0x30, 0x00, 0x80, 0xff, 0x03, 0xf0, 0xff, 0xc1, 0xff, 0xff, 0xff, 0x0f, 0xfe, 0x3f,
  0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0x1f, 0xfe,
  0x3f, 0xf8, 0xff, 0xff, 0x1f, 0x80, 0xff, 0xff, 0xff, 0x7f, 0x80, 0xff, 0xff, 0xff, 0x3f,
  0xfe, 0x3f, 0xe0, 0xff, 0xff, 0x07, 0x80, 0xff, 0xff, 0xff, 0x1f, 0xc0, 0xfe, 0xff, 0xff,
  0x7f, 0xfe, 0x3f, 0x80, 0xff, 0xff, 0x01, 0x80, 0xff, 0xff, 0xff, 0x03, 0xc0, 0xf1, 0xff,
  0xff, 0x7f, 0xfc, 0x7f, 0x00, 0xfe, 0xff, 0x00, 0x80, 0xff, 0x03, 0x00, 0x00, 0xc0, 0x03,
  0xf8, 0xff, 0x7f, 0xfc, 0x7f, 0x00, 0xf8, 0x3f, 0x00, 0x80, 0xff, 0x03, 0x00, 0x00, 0xc0,
  0x0f, 0x00, 0xe0, 0x7f, 0xf8, 0xff, 0x00, 0xf8, 0x3f, 0x00, 0xc0, 0xff, 0x07, 0x00, 0x00,
  0xc0, 0x7f, 0x00, 0xc0, 0x7f, 0xf0, 0xff, 0x03, 0xfe, 0x3f, 0x00, 0xe0, 0xff, 0x0f, 0x00,
  0x00, 0xc0, 0xff, 0x07, 0xe0, 0x3f, 0xe0, 0xff, 0xff, 0xff, 0x3f, 0x00, 0xf8, 0xff, 0x3f,
  0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x1f, 0x80, 0xff, 0xff, 0xff, 0x3f, 0x00, 0xfc, 0xff,
  0x7f, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xfc, 0xff, 0xff, 0x3c, 0x00, 0xff,
  0xff, 0xff, 0x01, 0x00, 0xc0, 0xe3, 0xff, 0xff, 0x03, 0x00, 0xc0, 0xff, 0x07, 0x30, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xfc, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f,
  0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x3f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x3f, 0xfe, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0x3f, 0xfe, 0x77, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xef,
  0xff, 0xff, 0xff, 0x3f, 0xfe, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff,
  0xef, 0xff, 0xff, 0xff, 0x3f, 0xfe, 0xfb, 0x2f, 0xfc, 0xf0, 0xc3, 0x0f, 0x3d, 0x3c, 0xf0,
  0xc3, 0x03, 0x3f, 0xfc, 0xe2, 0x3f, 0xfe, 0xfb, 0xcf, 0x7b, 0xef, 0xbd, 0xf7, 0xdc, 0xbb,
  0xed, 0xbd, 0xef, 0xdf, 0xfb, 0xdc, 0x3f, 0xfe, 0xc7, 0xef, 0xb7, 0xdf, 0x7e, 0xfb, 0xed,
  0xb7, 0xed, 0x7e, 0xef, 0xef, 0xf7, 0xfe, 0x3f, 0xfe, 0x1f, 0xee, 0xb7, 0xdf, 0x7e, 0xfb,
  0xed, 0xb7, 0xed, 0x7e, 0xef, 0xef, 0xf7, 0xfe, 0x3f, 0xfe, 0xff, 0xed, 0x37, 0xc0, 0x00,
  0xfb, 0xed, 0xb7, 0xed, 0x00, 0xef, 0x0f, 0xf0, 0xfe, 0x3f, 0xfe, 0xff, 0xed, 0xb7, 0xff,
  0xfe, 0xfb, 0xed, 0xb7, 0xed, 0xfe, 0xef, 0xef, 0xff, 0xfe, 0x3f, 0xfe, 0xfb, 0xed, 0xb7,
  0xff, 0xfe, 0xfb, 0xed, 0xb7, 0xed, 0xfe, 0xef, 0xef, 0xff, 0xfe, 0x3f, 0xfe, 0xf3, 0xcc,
  0x7b, 0xdf, 0x7d, 0xf7, 0xdc, 0xbb, 0xed, 0x7d, 0xef, 0xdf, 0xf7, 0xfe, 0x3f, 0xfe, 0x07,
  0x2f, 0xfc, 0xe0, 0x83, 0x0f, 0x3d, 0xbc, 0xed, 0x83, 0x1f, 0x3f, 0xf8, 0xfe, 0x3f, 0xfe,
  0xff, 0xef, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f,
  0xfe, 0xff, 0xef, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x3f, 0xfe, 0xff, 0xef, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x3f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0c, 0x00, 0x00, 0x0c, 0x8c, 0x01, 0x80,
  0x0f, 0x03, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0c, 0x00, 0x00, 0x0c, 0x8c, 0x01,
  0x80, 0x19, 0x03, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0c, 0x00, 0x00, 0x0c, 0x8c,
  0x01, 0x80, 0x31, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x60, 0x8c, 0x87, 0x87, 0x0d,
  0x8c, 0xd9, 0x80, 0x31, 0xf3, 0x6c, 0x8c, 0x67, 0x0c, 0x00, 0x00, 0xe0, 0xcf, 0x4c, 0xcc,
  0x0e, 0x8c, 0xb9, 0x81, 0x31, 0x9b, 0xdd, 0x4c, 0x6c, 0x0c, 0x00, 0x00, 0x60, 0xcc, 0x0c,
  0xcf, 0x0c, 0x8c, 0x99, 0x81, 0x31, 0x3b, 0xcc, 0x0c, 0xcf, 0x06, 0x00, 0x00, 0x60, 0xcc,
  0x8f, 0xcd, 0x0c, 0x8c, 0x99, 0x81, 0x31, 0xf3, 0xcc, 0x8c, 0xcd, 0x06, 0x00, 0x00, 0x60,
  0xcc, 0xc0, 0xcc, 0x0c, 0x8c, 0x99, 0x81, 0x31, 0xc3, 0xcd, 0xcc, 0xcc, 0x06, 0x00, 0x00,
  0x60, 0xcc, 0xcc, 0xcc, 0x0e, 0x8c, 0xb9, 0x81, 0x19, 0x9b, 0xdd, 0xcc, 0x8c, 0x03, 0x00,
  0x00, 0x60, 0x8c, 0x87, 0x8f, 0x0d, 0xf8, 0xd8, 0x80, 0x0f, 0xf3, 0x6c, 0x8c, 0x8f, 0x03,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x80,
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x0c, 0x00,
  0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x0c,
  0x00, 0xe0, 0x01, 0x00
};



//Program variables
double Lat, oLat = 0;
double Long, oLong = 0;

double lastlat;
double lastlong;

double odometer = 0;
double lastodometer = 0;

float gps_speed = 0;
float hdop = 0;
int alt;
byte num_sat, satinview, page, pages;

//String heading;
static NMEAGPS gps; // This parses the GPS characters

//int timer, timers, blank, msg;
NeoGPS::time_t localTime; //(utcTime + (UTC_offset * (60 * 60)));

//include "espnow";

#define U8LOG_WIDTH 20
#define U8LOG_HEIGHT 8
//uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];
//U8G2LOG u8g2log;





String getDayName(){
  switch (localTime.day)
  {
    case 1:  return "SUN";   break;
    case 2:  return "MON";   break;
    case 3:  return "TUE";   break;
    case 4:  return "WED";   break;
    case 5:  return "THU";;   break;
    case 6:  return "FRI";   break;
    default: return "SAT";
  }
}

void setTZ()
{
  NeoGPS::clock_t utcTime = gps.fix().dateTime; // convert to seconds
  localTime = (utcTime + (adjhour * (60 * 60)));
  Serial.print("localtime");
  Serial.println(localTime);
/*
 last_second = gps.time.second();

        // set current UTC time
        setTime(Hour, Minute, Second, Day, Month, Year);
        // add the offset to get local time
        adjustTime(time_offset);

        // update time array
        Time[0]  = hour()   / 10 + '0';
        Time[1]  = hour()   % 10 + '0';
        Time[3]  = minute() / 10 + '0';
        Time[4] = minute() % 10 + '0';
        Time[6] = second() / 10 + '0';
        Time[7] = second() % 10 + '0';

        // update date array
        Date[0]  =  day()   / 10 + '0';
        Date[1]  =  day()   % 10 + '0';
        Date[3]  =  month() / 10 + '0';
        Date[4] =  month() % 10 + '0';
        Date[8] = (year()  / 10) % 10 + '0';
        Date[9] =  year()  % 10 + '0';
        // print time & date
        print_wday(weekday());   // print day of the week
  */
}


float getLat() {
  if (valid_location) {
    return  Lat;
  }
  return 0.00000;
}

float getLon() {
  if (valid_location) {
    return   Long;
  }
  return 0.00000;
}


String getDateTimeLoc() {
  return datelocal;
}

gpsdata_struct getGPSData() {
  gpsdata_struct data;
  data.Lat = Lat;
  data.Long = Long;
  data.odometer = odometer;
  data.alt = alt;
  if (gps_speed > MIN_KPH && gps_speed < 150 && Lat < 0 && Long > 80)// and hdop <= 3000 and hdop > 0 )
  {
    data.gps_speed = gps_speed;
  } else {
    data.gps_speed = 0;
  }
  data.num_sat = num_sat;
  data.satinview = satinview;
  data.datelocal = datelocal;
  data.valid_location = valid_location;

  return data;

}


char *cardinal(double course)
{
  char *directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}




void gps_setup() {
  gps_port.begin(GPSBaud, SWSERIAL_8N1, RX_GPS, TX_GPS, false);
}


String getIDStart(NeoGPS::time_t localtime) {
  String res = "";
  res.concat(localtime.full_year());

  if (localtime.month < 10)
    res.concat("0");
  res.concat(localtime.month);

  if (localtime.date < 10)
    res.concat("0");

  if (localtime.hours < 10)
    res.concat("0");

  res.concat(localtime.hours);

  if (localtime.minutes < 10)
    res.concat("0");

  res.concat(localtime.minutes);


  if (localtime.seconds < 10)
    res.concat("0");

  res.concat(localtime.seconds);
  return res;
}


/* static */
double long distanceBetween(double lat1, double long1, double lat2, double long2)
{
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

String getdatetime(NeoGPS::time_t localtime) {
  String res = "";
  res.concat(localtime.full_year());
  res.concat("-");

  if (localtime.month < 10)
    res.concat("0");
  res.concat(localtime.month);

  res.concat("-");
  if (localtime.date < 10)
    res.concat("0");
  res.concat(localtime.date);

  res.concat(" ");

  if (localtime.hours < 10)
    res.concat("0");
  res.concat(localtime.hours);
  res.concat(":");
  if (localtime.minutes < 10)
    res.concat("0");

  res.concat(localtime.minutes);
  res.concat(":");
  if (localtime.seconds < 10)
    res.concat("0");

  res.concat(localtime.seconds);
  return res;
}


void gps_handle() {

  while (gps_port.available())
  {
    char c = gps_port.read();

    if (usbdebug) {
      // if (counter == 1 && page == 4 ) {
      //   u8g.setFont(u8g2_font_5x8_mr);  // set the font for the terminal window
      //   u8g2log.print(c);               // print to display
      // }
      Serial.write(c);

    }

    if (gps.decode(c) == NMEAGPS::DECODE_COMPLETED)
    {
      if (nmea > 200)
        nmea = 0;

      nmea++;

      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC)
      {

        //  If your device emits a GGA then an RMC each second,
        //    change the above GGA to RMC.  Or if you don't get all
        //    the attributes (e.g., alt, heading, speed and satellites)
        //    try changing the above test.  NMEAorder.ino can be
        //    used to determine which one is last (should be used above).

        //  BTW, this is the safest place to do any time-consuming work,
        //    like updating the display.  Doing it elsewhere will cause GPS
        //    characters to be lost, and some fix data won't be available/valid.
        //    And if you take too long here, you could still lose characters.

        const gps_fix &fix = gps.fix();

        //if (fix.speed_kph() > 0)
        // {
        //  fix.valid.speed = true;
        // }
        

        valid_time = fix.valid.time;

        if (localTime.full_year() != 2000 and idstart == "") {
          idstart = getdatetime(localTime);
        }
              NeoGPS::clock_t utcTime = fix.dateTime; // convert to seconds
              localTime = (utcTime + (adjhour * (60 * 60)));
              
        datelocal = getdatetime(localTime);

        String date1 = datelocal.substring(0, 10); //getStringPartByNr(dates, ' ', 0);
        String time1 = datelocal.substring(11, 19); //getStringPartByNr(dates, ' ', 1);
        date1.toCharArray(dateloc, 11);
        time1.toCharArray(timeloc, 9);
        //strcpy(dateloc,date1.toCharArray(dateloc, 10));
        //strcpy(timeloc,time1.toCharArray(timeloc,8));

        if (fix.valid.heading)
        {
          heading_cd = fix.heading();
          heading = cardinal((double)heading_cd);
        }
        //Serial.print("Heading ");
        //Serial.print(heading);
        //num_sat = gps.sat_used; //
        num_sat = fix.satellites;
        satinview = gps.sat_view;
        //Serial.println("Sat view"+String(satinview));
        //                                                                                                                                                                                                                                                                                                                                                       Serial.println("Sat View "+String(gps.sat_view,0));
        alt = fix.altitude_cm() / 100;
        hdop = fix.hdop;

        valid_location = fix.valid.location;

        //gps_speed = (fix.valid.speed ? (float) fix.speed_kph() : 0);
        //if  (fix.speed_kph() > 0)
        if (fix.valid.speed)
          gps_speed = (float) fix.speed_kph();
        else
          gps_speed = 0;

        //if (gps.fix().valid.speed)
        //  gps_speed = (float)gps.fix().speed_kph();
        //else
        //  gps_speed = 0;

        if (fix.valid.location)
        {
          timefixgps = millis();

          Lat = (float)fix.latitude();
          Long = (float)fix.longitude();

          //gps.fix().valid.location = true;

          //indonesia
          //if (Lat > 0)
          //gps.fix().valid.location = false;

          //if (Long < 80)
          //gps.fix().valid.location = false;
        }
        else
        {
          if ( timefixgps > 0) {
            timefixgps = 0;
            timestartgps = millis();


          }


          //gps.fix().valid.location = false;
          //heading = "";
          //heading_cd = 0;
        }



        if (fix.valid.location)
        {

          if (millis() - timers > 1000)
          {
            //adjwarn = 50;
            //gps_speed = gps_speed * 50;

            //if ((byte)(gps_speed/1000) > adjwarn) buzzeron = true;

            //else buzzeron = false;




            if (gps_speed > MIN_KPH && gps_speed < 150 && Lat < 0 && Long > 80 and hdop <= 3000 and hdop > 0)
            {
              //gps_speed += adjspeed;


              buzzeron = (((byte)gps_speed > adjwarn) ? true : false);
              buzzer = buzzeron;

              if (oLat != 0 && oLong != 0)
              {
                double distance = distanceBetween(Lat, Long, oLat, oLong);
                //heading = headingTo(Lat,Long,oLat,oLong);
                int blanks = 0;
                double blankdistance = 0;
                double estdist = 0;

                if (blank > 0)
                {
                  blanks = millis() - blank;
                  blankdistance = gps_speed * 1000 / 60 / 60 * (blanks / 1000);
                }

                //estdist = (gps_speed * 1000 / 60 / 60 * ((millis() - timers) / 1000)) + blankdistance;
                /*
                  Serial.print("speed : ");
                  Serial.print(gps_speed);


                  Serial.print(", timeer : ");
                  Serial.print(millis()-timers);


                  Serial.print(", timer blank : ");
                  Serial.print(blanks);

                  Serial.print(", distance gps : ");
                  Serial.print(distance);
                  Serial.print(",distance blank = ");
                  Serial.print(blankdistance);
                */
                if (distance > 0)
                  odometer += distance;
                /*
                  if (distance <= estdist && distance > 0)
                  {
                   // distance by lat long is make sense use it;
                   odometer += distance;
                   //Serial.print(", Use distance By Lat Long");
                  }
                  else
                  {
                   // use distance as averagspeed x time in seconds
                   blanks = millis() - timers;
                   blankdistance = gps_speed * 1000 / 60 / 60 * (blanks / 1000);
                   //Serial.print(", Use distance By average speed = ");
                   //Serial.print(estdist);
                   odometer += estdist;
                  }
                */
                //Serial.println("");
              }
              oLat = Lat;
              oLong = Long;
            }
            else
            {
              buzzeron = false;
            }

            if (millis() - timerupdate > (1000 * UPDATE_INTERVAL)  && counter < 20) {
              double odo = odometer - lastodometer;

              NeoGPS::clock_t utcTime = fix.dateTime; // convert to seconds
              localTime = (utcTime + (adjhour * (60 * 60)));

              // Serial.println("Collectiong data to send.");
              /*
                String cont = "device[]=" + simei + "&start[]=";
                cont.concat(idstart);
                cont.concat("&cid[]=");
                cont.concat(sunique);
              */
              /*
                cont.concat("&lat[]=");
                cont.concat(String(Lat, 6));
                cont.concat("&lon[]=");
                cont.concat(String(Long, 6));
                cont.concat("&speed[]=");
                cont.concat(gps_speed);
                cont.concat("&satused[]=");
                cont.concat(num_sat);
                cont.concat("&hdop[]=");
                cont.concat(hdop);
                cont.concat("&distance[]=");
                cont.concat(odometer);
                cont.concat("&dir[]=");
                cont.concat(heading_cd);
                cont.concat("&time[]=");
                cont.concat(getdatetime(localTime));
                cont.concat("&ct[]=");
                cont.concat(coollant);
                cont.concat("&bv[]=");
                cont.concat(volt);
                cont.concat("&tmp[]=");
                cont.concat(temp);
                cont.concat("&acst[]=");
                cont.concat(acstat);
                cont.concat("&fanst[]=");
                cont.concat(fanst);
                cont.concat("&sgnlv[]=");
                cont.concat(signallvl);
                cont.concat("&blv[]=");
                cont.concat(batlvl);
                cont.concat("&gprsst[]=");
                cont.concat(gprsst);
                cont.concat("&end=");
              */

              String cont = "dt[]=";

              if (gps_speed > MIN_KPH && gps_speed < 150 && Lat < 0 && Long > 80)// and hdop <= 3000 and hdop > 0 )
              {
                cont.concat(String(Lat, 6));
                cont.concat("|");
                cont.concat(String(Long, 6));
              } else {
                if (lastlat > 0) {
                  cont.concat(String(lastlat, 6));
                  cont.concat("|");
                  cont.concat(String(lastlong, 6));
                } else {
                  cont.concat(String(Lat, 6));
                  cont.concat("|");
                  cont.concat(String(Long, 6));
                }
              }
              cont.concat("|");
              cont.concat(gps_speed);
              cont.concat("|");
              cont.concat(num_sat);
              cont.concat("|");
              cont.concat(hdop);
              cont.concat("|");
              cont.concat(odometer);
              cont.concat("|");
              cont.concat(heading_cd);
              cont.concat("|");
              cont.concat(getdatetime(localTime));
              cont.concat("|");
              cont.concat(coollant);
              cont.concat("|");
              cont.concat(volt);
              cont.concat("|");
              cont.concat(temp);
              cont.concat("|");
              cont.concat(acstat);
              cont.concat("|");
              cont.concat(fanst);
              cont.concat("|");
              cont.concat(signallvl);
              cont.concat("|");
              cont.concat(batlvl);
              cont.concat("|");
              cont.concat(gprsst);
              //cont.concat("&end=");
              //contents.unshift(cont);
              //Serial.print("Data : ");
              //Serial.println(cont);
              timerupdate = millis();
               idxcount = addcontentsend(cont, idxcount);
              lastodometer = odometer;

              //initsend();

              if ( gps_speed > MIN_KPH && gps_speed < 150 && Lat < 0 && Long > 80 ) // and hdop <= 3000 and hdop > 0)
              {
                lastlat = Lat;
                lastlong = Long;

              }

            }



            //oLat = Lat;
            //oLong = Long;
            timers = millis();
          }
          blank = 0;
        }
        else
        {
          blank = millis();
        }


        //timer = millis(); // reset the timer
      }
      //lnmea = nmea;
    }
  }
}
