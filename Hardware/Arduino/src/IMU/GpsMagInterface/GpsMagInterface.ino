//// Code for accessing Compass data

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Message counters
uint8_t send_counter = 0;       // Message sending counter
uint8_t packet_counter = 0;     // Packet counter

void reportCompassDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

struct BearingData {
  long x;
  long y;
  long z;
  long heading;
};

BearingData magData;

void initializeMAG() {
  if(!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Cannot connect to compass");
    while(1);
  }
  /* Display some basic information on this sensor */
  //reportCompassDetails();
}

void updateMAG(void) {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  //float declinationAngle = 0.22;
  //heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  //float headingDegrees = heading * 180/M_PI; 
  
  magData.x = (long)(event.magnetic.x*1000.0);
  magData.y = (long)(event.magnetic.y*1000.0);
  magData.z = (long)(event.magnetic.z*1000.0);
  magData.heading = (long)(heading*1000.0);
  
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //Serial.print("X:"); Serial.print(event.magnetic.x);
  //Serial.print(" Y:"); Serial.print(event.magnetic.y);
  //Serial.print(" Z:"); Serial.print(event.magnetic.z);
  /* Display heading */
  //Serial.print(" Heading:"); Serial.println(headingDegrees);
  
  //delay(10);
}

//// Code for accessing GPS data

byte gpsConfigsSent;  // number of cfg msgs sent
byte gpsConfigTimer;  // 0 = no more work, 1 = send now, >1 wait

static const unsigned char UBX_5HZ[] = {0xb5,0x62,0x06,0x08,0x06,0x00,0xc8,0x00,0x01,0x00,0x01,0x00,0xde,0x6a};
const unsigned long gpsBaudRates[] = { 9600L, 19200L, 38400L, 57600L, 115200L};
 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum {
  GPS_INVALID_ACCURACY = 0xFFFFFFFF, 
  GPS_INVALID_AGE = 0xFFFFFFFF, 
  GPS_INVALID_ANGLE = 0x7FFFFFFF, 
  GPS_INVALID_ALTITUDE = 2147483647,//999999999, 
  GPS_INVALID_DATE = 0,
  GPS_INVALID_TIME = 0xFFFFFFFF, 
  GPS_INVALID_SPEED = 999999999, 
  GPS_INVALID_FIX_TIME = 0xFFFFFFFF
};

enum { 
    GPS_DETECTING = 0, 
    GPS_NOFIX = 1,
    GPS_FIX2D = 2,
    GPS_FIX3D = 3,
    GPS_FIX3DD = 4 // differential fix 
};
#define UBLOX_5HZ   {UBX_5HZ,sizeof(UBX_5HZ)}
#define UBLOX_38400 {(unsigned char *)"$PUBX,41,1,0003,0003,38400,0*24\r\n",0}
#define UBLOX_CONFIGS UBLOX_5HZ,UBLOX_38400
#define GPS_MAXIDLE_DETECTING 200 // 2 seconds at 100Hz
#define GPS_MAXIDLE 500           // 5 seconds at 100Hz

/////////////////////////////////////////////////////////////////////////////////////////////////
struct GeodeticPosition {
  long latitude;
  long longitude;
  long altitude;
};
struct gpsData {
    int32_t  lat,lon;  // position as degrees (*10E7)
    int32_t  course;   // degrees (*10E5)
    uint32_t speed;    // cm/s
    int32_t  height;   // mm (from ellipsoid)
    uint32_t accuracy; // mm
    uint32_t fixage;   // fix 
    uint32_t fixtime;  // fix 
    uint32_t sentences; // sentences/packets processed from gps (just statistics)
    uint8_t  state;    // gps state
    uint8_t  sats;     // number of satellites active
    uint8_t  baudrate; // current baudrate (index) - used by autodetection
    uint8_t  type;     // current type - used by autodetection
    uint32_t idlecount; // how many times gpsUpdate has been called without getting a valid message
};

struct gpsConfigEntry {
  const unsigned char *data;
  const unsigned char len;
};


////////////////////////////////////////////////////////////////////////////////////////////////////
struct gpsData gpsData; // This is accessed by the parser functions directly !
struct gpsConfigEntry gpsConfigEntries[] = {UBLOX_CONFIGS};
GeodeticPosition currentPosition;

// Transmission packet structure
static const uint8_t gps_mag_packet_size = 44;
uint8_t gps_mag_packet[gps_mag_packet_size] = {
  '$',                    // Starting code        0
  0x00,                   // Message uint8_t      1
  0,0,0,0,                // Time int32_t         2-5
  0,                      // Gps State uint8_t    6
  0,0,0,0,                // Latitude int32_t     7-10
  0,0,0,0,                // Longitude int32_t    11-14
  0,0,0,0,                // Altitude int32_t     15-18
  0,0,0,0,                // Accuracy uint32_t    19-22
  0,                      // Num sats uint8_t     23
  0,0,0,0,                // Mag X int32_t        24-27
  0,0,0,0,                // Mag Y int32_t        28-31
  0,0,0,0,                // Mag Z int32_t        32-35
  0,0,0,0,                // Heading int32_t      36-39
  0x00,                   // Packet number        40
  0x00, '\r', '\n'        // Termination code     41-43
};

void PacketBuildingHelper_uint8(uint8_t idx, uint8_t dat) {
  gps_mag_packet[idx] = dat;
}

void PacketBuildingHelper_uint32(uint8_t idx, uint32_t dat) {
  gps_mag_packet[idx]   = (uint8_t) (dat);
  gps_mag_packet[idx+1] = (uint8_t) (dat >> 8);
  gps_mag_packet[idx+2] = (uint8_t) (dat >> 16);
  gps_mag_packet[idx+3] = (uint8_t) (dat >> 24);  
}

void PacketBuildingHelper_long(uint8_t idx, long dat) {
  gps_mag_packet[idx]   = (uint8_t) (dat);
  gps_mag_packet[idx+1] = (uint8_t) (dat >> 8);
  gps_mag_packet[idx+2] = (uint8_t) (dat >> 16);
  gps_mag_packet[idx+3] = (uint8_t) (dat >> 24);  
}

void ReportData() {

  // Report as text
  /*Serial.print("Time:"); Serial.print(gpsData.fixtime);  
  Serial.print(" St:");  Serial.print(gpsData.state);
  Serial.print(" Lat:"); Serial.print(gpsData.lat);
  Serial.print(" Lon:"); Serial.print(gpsData.lon);
  Serial.print(" Alt:"); Serial.print(gpsData.height);
  Serial.print(" Acc:"); Serial.print(gpsData.accuracy);
  Serial.print(" Sats:"); Serial.print(gpsData.sats);
  Serial.print(" Mag: X:"); Serial.print(magData.x);
  Serial.print(" Y:"); Serial.print(magData.y);
  Serial.print(" Z:"); Serial.print(magData.z);
  Serial.print(" Heading:"); Serial.println(magData.heading);*/

  // Build packet
  packet_counter++;   // Rollsover at 0xFF
  gps_mag_packet[1] = packet_counter;

  PacketBuildingHelper_uint32 (2,  gpsData.fixtime);
  PacketBuildingHelper_uint8  (6,  gpsData.state);
  PacketBuildingHelper_long   (7,  gpsData.lat);
  PacketBuildingHelper_long   (11, gpsData.lon);
  PacketBuildingHelper_long   (15, gpsData.height);
  PacketBuildingHelper_uint32 (19, gpsData.accuracy);
  PacketBuildingHelper_uint8  (23, gpsData.sats);
  PacketBuildingHelper_long   (24, magData.x);
  PacketBuildingHelper_long   (28, magData.y);
  PacketBuildingHelper_long   (32, magData.z);
  PacketBuildingHelper_long   (36, magData.heading);

  if (true) {
    send_counter++;
    gps_mag_packet[40] = send_counter;
    Serial.write(gps_mag_packet, gps_mag_packet_size);  
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void gpsSendConfig() {
  
  if (gpsConfigEntries[gpsConfigsSent].data) {
    if (gpsConfigEntries[gpsConfigsSent].len) {
      for (int i=0; i<gpsConfigEntries[gpsConfigsSent].len; i++) {
        Serial1.write(gpsConfigEntries[gpsConfigsSent].data[i]);
      }
      gpsConfigTimer=gpsConfigEntries[gpsConfigsSent].len;
    }
    else {
      Serial1.print((char*)gpsConfigEntries[gpsConfigsSent].data);
      gpsConfigTimer=strlen((char*)gpsConfigEntries[gpsConfigsSent].data);
    }
   
   if (gpsConfigTimer<10) {
      gpsConfigTimer=10;
    }
    gpsConfigsSent++;
  }

}


void updateGps() {
  
  gpsData.idlecount++;
  
  if (gpsData.idlecount == 200) {
    //Serial.println("NO data");
    gpsData.idlecount = 0;
  }
  
  while (Serial1.available()) {
    unsigned char c = Serial1.read();
    int ret=0;

    if (gpsData.state == GPS_DETECTING) {
      ret = ubloxProcessData(c);
      if (ret) {
        // found GPS device start sending configuration
        gpsConfigsSent = 0;
        gpsConfigTimer = 1;
        break;
      } 
    } 
    else {
      // Normal operation just execute the detected parser
      ret = ubloxProcessData(c);
    }

    if (gpsConfigTimer) {
      if (gpsConfigTimer==1) {
        gpsSendConfig();
      }
      gpsConfigTimer--;
    }
    
    // Upon a successfully parsed sentence, zero the idlecounter and update position data
    if (ret) {
      if (gpsData.state == GPS_DETECTING) {
        gpsData.state = GPS_NOFIX; // make sure to lose detecting state (state may not have been updated by parser)
      }
      gpsData.idlecount=0;
      currentPosition.latitude=gpsData.lat;
      currentPosition.longitude=gpsData.lon;
      currentPosition.altitude=gpsData.height;
      //Serial.print("Stt:");Serial.print(gpsData.state);
      //Serial.print(" Lat:");Serial.print(gpsData.lat);
      //Serial.print(" Lon:");Serial.print(gpsData.lon);
      //Serial.print(" Alt:");Serial.print(gpsData.height);
      //Serial.print(" Acc:");Serial.print(gpsData.accuracy);
      //Serial.print(" Sats:"); Serial.print(gpsData.sats);
      //Serial.print(" Time:"); Serial.println(gpsData.fixtime);
      
      // Update compass data
      updateMAG();
      
      // Report data
      ReportData();
      
    }
  }

  // Schedule config sending if needed
  if (gpsConfigTimer) {
    if (gpsConfigTimer==1) {
      gpsSendConfig();
    }
    gpsConfigTimer--;
  }

}  // updateGps


void initializeGpsData() {
  gpsData.lat = GPS_INVALID_ANGLE;
  gpsData.lon = GPS_INVALID_ANGLE;
  gpsData.course = GPS_INVALID_ANGLE;
  gpsData.speed = GPS_INVALID_SPEED;
  gpsData.height = GPS_INVALID_ALTITUDE;
  gpsData.accuracy = GPS_INVALID_ACCURACY;
  gpsData.fixage = GPS_INVALID_AGE;
  gpsData.state = GPS_DETECTING;
  gpsData.sentences = 0;
  gpsData.sats = 0;
  gpsData.fixtime = 0xFFFFFFFF;
}


void initializeGps() {
  gpsData.baudrate = 2;
  Serial1.begin(gpsBaudRates[gpsData.baudrate]);
  ubloxInit();
  initializeGpsData();
}


////////////////////////////////////////////////////////////////////////////////////////////////////

struct ublox_NAV_STATUS { // 01 03 (16)
  uint32_t iTow;
  uint8_t  gpsFix;
  uint8_t  flags;
  uint8_t  fixStat;
  uint8_t  flags2;
  uint32_t ttfx;
  uint32_t msss;
};

struct ublox_NAV_POSLLH { // 01 02 (28)
  uint32_t iTow;
  int32_t lon; // 1e-7 degrees
  int32_t lat; // 1e-7 degrees
  int32_t height; // mm
  int32_t hMSL; // mm
  uint32_t hAcc; //mm
  uint32_t vAcc; //mm
};

struct ublox_NAV_SOL { // 01 6 (52)
  uint32_t iTow;
  int32_t  fTow;
  int16_t  week;
  uint8_t  gspFix;
  uint8_t  flags;
  int32_t  ecefX;
  int32_t  ecefY;
  int32_t  ecefZ;
  int32_t  pAcc;
  int32_t  ecefVX;
  int32_t  ecefVY;
  int32_t  ecefVZ;
  int32_t  sAcc;
  uint16_t pDOP;
  uint8_t  res1;
  uint8_t  numSV;
  uint32_t res2;
};

struct ublox_NAV_VELNED { // 01 12h (36)
  uint32_t iTow;
  int32_t  velN; // cm/s
  int32_t  velE; // cm/s
  int32_t  velD; // cm/s
  uint32_t  speed; // cm/s
  uint32_t  gSpeed; // cm/s
  int32_t  heading; // dev 1e-5
  uint32_t sAcc; // cm/s
  uint32_t cAcc; // deg 1e-5
};

unsigned short ubloxClass,ubloxId;
unsigned char  ubloxCKA,ubloxCKB;
unsigned short ubloxExpectedDataLength;
unsigned short ubloxDataLength;

union ublox_message {
  struct ublox_NAV_STATUS nav_status;
  struct ublox_NAV_POSLLH nav_posllh;
  struct ublox_NAV_VELNED nav_velned;
  struct ublox_NAV_SOL nav_sol;
  unsigned char raw[52];
} ubloxMessage;

/////////////////////////////////////////////////////////////////////////////////////////////////////
enum ubloxState{ WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID,
GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB  } ubloxProcessDataState;

void ubloxInit() {
  ubloxProcessDataState = WAIT_SYNC1;
}

int ubloxParseData() { // uses public vars
  
  int parsed = 0;

  gpsData.sentences++;
  if (ubloxClass==1) { // NAV
    if (ubloxId==2) { // NAV:POSLLH
      gpsData.lat = ubloxMessage.nav_posllh.lat;
      gpsData.lon = ubloxMessage.nav_posllh.lon;
      gpsData.height = ubloxMessage.nav_posllh.height;
      gpsData.accuracy = ubloxMessage.nav_posllh.hAcc;
      //Serial.print("Accuracy: ");Serial.println(gpsData.accuracy);
      gpsData.fixtime = ubloxMessage.nav_posllh.iTow;
      parsed = 1;
    }
    else if (ubloxId==3) { //NAV:STATUS
      switch (ubloxMessage.nav_status.gpsFix) {
        case 2: 
          gpsData.state = GPS_FIX2D;
          //Serial.println(" 2D Fix ");
          break;
		  
        case 3:
          gpsData.state = GPS_FIX3D;
          //Serial.println(" 3D Fix ");
          break;
		  
        default:
          gpsData.state = GPS_NOFIX;
          //Serial.println(" No Fix ");
          break;
      }
    }
    else if (ubloxId==6) { // NAV:SOL
      gpsData.sats = ubloxMessage.nav_sol.numSV;
      //Serial.print("No. of Sats: "); Serial.println(gpsData.sats);
    }
    else if (ubloxId==18) { // NAV:VELNED
      gpsData.course = ubloxMessage.nav_velned.heading / 100; // 10E-5 to millidegrees
      gpsData.speed = ubloxMessage.nav_velned.gSpeed;
    }
  }
  return parsed;
}  // ubloxParseData

int ubloxProcessData(unsigned char data) {
  
  int parsed = 0;
  
  switch (ubloxProcessDataState) {
    
   case WAIT_SYNC1:
    if (data == 0xb5) {
      ubloxProcessDataState = WAIT_SYNC2;
    }
    break;
	
   case WAIT_SYNC2:
    if (data == 0x62) {
      ubloxProcessDataState = GET_CLASS;
    }
    else if (data == 0xb5) {
      // ubloxProcessDataState = GET_SYNC2;
    }
    else {
      ubloxProcessDataState = WAIT_SYNC1;
    }
    break;
   case GET_CLASS:
    ubloxClass=data;
    ubloxCKA=data;
    ubloxCKB=data;
    ubloxProcessDataState = GET_ID;
    break;
	
   case GET_ID:
    ubloxId=data;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_LL;
    break;
	
   case GET_LL:
    ubloxExpectedDataLength = data;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_LH;
    break;
	
   case GET_LH:
    ubloxExpectedDataLength += data << 8;
    ubloxDataLength=0;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    if (ubloxExpectedDataLength <= sizeof(ubloxMessage)) {
      ubloxProcessDataState = GET_DATA;
    }
    else {
      // discard overlong message
      ubloxProcessDataState = WAIT_SYNC1;
    }
    break;
	
   case GET_DATA:
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    // next will discard data if it exceeds our biggest known msg
    if (ubloxDataLength < sizeof(ubloxMessage)) {
      ubloxMessage.raw[ubloxDataLength++] = data;
    }
    if (ubloxDataLength >= ubloxExpectedDataLength) {
      ubloxProcessDataState = GET_CKA;
    }
    break;
	
   case GET_CKA:
    if (ubloxCKA != data) {
      ubloxProcessDataState = WAIT_SYNC1;
    } 
	else {
      ubloxProcessDataState = GET_CKB;
    }
    break;
	
   case GET_CKB:
    if (ubloxCKB == data) {
      //parsed = 1;
      parsed = ubloxParseData();
    }
    ubloxProcessDataState = WAIT_SYNC1;
    break;
	
  }
  return parsed;
}

//////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  initializeMAG();
  initializeGps();
}

void loop() {
  updateGps();
  //updateMAG();
  //delay(10);
}

