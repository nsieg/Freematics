/*************************************************************************
* Vehicle Telemetry Data Logger for Freematics ONE+
*
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products/freematics-one-plus for more info
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*************************************************************************/

#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>
//#include <httpd.h>
#include <FreematicsPlus.h>
#include "datalogger.h"
#include "config.h"
//#include <BLEDevice.h>
//#include <BLEScan.h>
#include "NimBLEDevice.h"
#include "list.h"
//#include "ESPAsyncTCP.h"
#include "ESPAsyncWebServer.h"

AsyncWebServer server(80);

// states
#define STATE_STORE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_CELL_GPS_FOUND 0x10
#define STATE_MEMS_READY 0x20
#define STATE_FILE_READY 0x40
#define STATE_STANDBY 0x80

void serverProcess(int timeout);
bool serverSetup();
bool serverCheckup(int wifiJoinPeriod = WIFI_JOIN_TIMEOUT);
void processBLE(int timeout);
void initMesh();

uint32_t startTime = 0;
uint32_t pidErrors = 0;
uint32_t fileid = 0;
// live data
char vin[18] = {0};
int16_t batteryVoltage = 0;

typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
};

#if USE_MEMS
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
float accBias[3];
float temp = 0;
ORIENTATION ori = {0};
#endif

#if USE_GNSS
GPS_DATA* gd = 0;
uint32_t lastGPStime = 0;
#endif

FreematicsESP32 sys;

#if ENABLE_NMEA_SERVER
WiFiServer nmeaServer(NMEA_TCP_PORT);
WiFiClient nmeaClient;
#endif

class DataOutputter : public FileLogger
{
    void write(const char* buf, byte len)
    {
#if ENABLE_SERIAL_OUT
        Serial.println(buf);
#endif
    }
};

COBD obd;

#if STORAGE == STORAGE_SD
SDLogger store(new DataOutputter);
#elif STORAGE == STORAGE_SPIFFS
SPIFFSLogger store(new DataOutputter);
#else
DataOutputter store;
#endif

#if USE_MEMS
MEMS_I2C* mems = 0;

void calibrateMEMS()
{
    // MEMS data collected while sleeping
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    for (n = 0; n < 100; n++) {
      mems->read(acc);
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC Bias:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
}
#endif
/*
int handlerLiveData(UrlHandlerParam* param)
// Removed
*/

void listDir(fs::FS &fs, const char * dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);
    fs::File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    fs::File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print(file.name());
            Serial.print(' ');
            Serial.print(file.size());
            Serial.println(" bytes");
        }
        file = root.openNextFile();
    }
}

class DataLogger
{
public:
    void init()
    {
#if USE_GNSS == 1 || USE_GNSS == 2
        if (!checkState(STATE_GPS_FOUND)) {
            Serial.print("GNSS:");
#if USE_GNSS == 1
            if (sys.gpsBegin()) {
#else
            if (sys.gpsBeginExt(GPS_SERIAL_BAUDRATE)) {
#endif
                setState(STATE_GPS_FOUND);
                Serial.println("OK");
                //waitGPS();
            } else {
                Serial.println("NO");
            }
        }
#elif USE_GNSS >= 3
// Removed
#endif

#if USE_OBD
// Removed
#endif
        startTime = millis();
    }
#if USE_GNSS
    void logLocationData(GPS_DATA* gd)
    {
        if (lastGPStime == gd->time) return;

        store.setTimestamp(millis());
        store.log(PID_GPS_DATE, gd->date);
        store.log(PID_GPS_TIME, gd->time);
        store.logFloat(PID_GPS_LATITUDE, gd->lat);
        store.logFloat(PID_GPS_LONGITUDE, gd->lng);
        store.log(PID_GPS_ALTITUDE, gd->alt); /* m */
        float kph = gd->speed * 1852 / 1000;
        store.log(PID_GPS_SPEED, kph);
        store.log(PID_GPS_SAT_COUNT, gd->sat);
        // set GPS ready flag
        setState(STATE_GPS_READY);

        Serial.print("[GNSS] ");

        char buf[32];
        int len = sprintf(buf, "%02u:%02u:%02u.%c",
            gd->time / 1000000, (gd->time % 1000000) / 10000, (gd->time % 10000) / 100, '0' + (gd->time % 100) / 10);
        Serial.print(buf);
        Serial.print(' ');
        Serial.print(gd->lat, 6);
        Serial.print(' ');
        Serial.print(gd->lng, 6);
        Serial.print(' ');
        Serial.print((int)kph);
        Serial.print("km/h");
        if (gd->sat) {
            Serial.print(" SATS:");
            Serial.print(gd->sat);
        }
        Serial.println();

        lastGPStime = gd->time;
        setState(STATE_GPS_READY);
    }
#endif
#if USE_GNSS == 1 || USE_GNSS == 2
    void processGPSData()
    {
        // issue the command to get parsed GPS data
        if (checkState(STATE_GPS_FOUND) && sys.gpsGetData(&gd) ) {
            logLocationData(gd);
        }
    }
    void waitGPS()
    {
        int elapsed = 0;
        for (uint32_t t = millis(); millis() - t < 300000;) {
          int t1 = (millis() - t) / 1000;
          if (t1 != elapsed) {
            Serial.print("Waiting for GPS (");
            Serial.print(elapsed);
            Serial.println(")");
            elapsed = t1;
          }
          // read parsed GPS data
          if (sys.gpsGetData(&gd) && gd->sat != 0 && gd->sat != 255) {
            Serial.print("Sats:");
            Serial.println(gd->sat);
            break;
          }
        }
    }
#elif USE_GNSS >= 3
// Removed
#endif
    void standby()
    {
        store.close();
#if USE_GNSS == 1 || USE_GNSS == 2
        if (checkState(STATE_GPS_READY)) {
            Serial.print("GNSS:");
            sys.gpsEnd(); // turn off GPS power
            Serial.println("OFF");
        }
#elif USE_GNSS >= 3
        Serial.print("GNSS:");
        cellUninit();
        Serial.println("OFF");
#endif
        // this will put co-processor into a delayed sleep
        sys.resetLink();
        clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_FILE_READY);
        setState(STATE_STANDBY);
        Serial.println("Standby"); 
#if USE_MEMS
        if (checkState(STATE_MEMS_READY)) {
            calibrateMEMS();
            while (checkState(STATE_STANDBY)) {
                // calculate relative movement
                float motion = 0;
                unsigned int n = 0;
                for (uint32_t t = millis(); millis() - t < 1000; n++) {
                    mems->read(acc, 0, 0, &temp);
                    for (byte i = 0; i < 3; i++) {
                        float m = (acc[i] - accBias[i]);
                        motion += m * m;
                    }
#if (ENABLE_WIFI_STATION || ENABLE_WIFI_AP)
                    serverCheckup(WIFI_JOIN_TIMEOUT * 4);
                    serverProcess(100);
#else
                    processBLE(100);
#endif
                }
                // check movement
                if (motion / n >= WAKEUP_MOTION_THRESHOLD * WAKEUP_MOTION_THRESHOLD) {
                    Serial.print("Motion:");
                    Serial.println(motion / n);
                    break;
                }
            }
        }
#else
        while (!obd.init()) Serial.print('.');
#endif
        Serial.println("Wakeup");
        // this will wake up co-processor
        sys.reactivateLink();
        //ESP.restart();
        clearState(STATE_STANDBY);
    }
#if USE_GNSS >= 3
// Removed
#endif
    bool checkState(byte flags) { return (m_state & flags) == flags; }
    void setState(byte flags) { m_state |= flags; }
    void clearState(byte flags) { m_state &= ~flags; }
private:
    byte m_state = 0;
};

DataLogger logger;

void showStats()
{
    uint32_t t = millis() - startTime;
    uint32_t dataCount = store.getDataCount();
    // calculate samples per second
    float sps = (float)dataCount * 1000 / t;
    // output to serial monitor
    char buf[32];
    sprintf(buf, "%02u:%02u.%c", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
    Serial.print(buf);
    Serial.print(" | ");
    Serial.print(dataCount);
    Serial.print(" samples | ");
    Serial.print(sps, 1);
    Serial.print(" sps");
    uint32_t fileSize = store.size();
    if (fileSize > 0) {
        Serial.print(" | ");
        Serial.print(fileid);
        Serial.print(".CSV | ");
        Serial.print(fileSize);
        Serial.print(" bytes");

        uint8_t fileTooBig = fileSize >> 21; // fileSize / 2M
        if (fileTooBig) {
            uint32_t newId = store.incrementId();
            fileid = newId;
            Serial.print(" (new file)");
        } else {
            static uint8_t lastFlushCount = 0;
            uint8_t flushCount = fileSize >> 12; // fileSize / 4K
            if (flushCount != lastFlushCount) {
                store.flush();
                lastFlushCount = flushCount; 
                Serial.print(" (flushed)");
            }
        }
    }
    Serial.println();
}
 
void processBLE(int timeout)
{
#if ENABLE_BLE
// Removed
#else
    if (timeout) delay(timeout);
#endif
}

void showSysInfo()
{
  Serial.print("CPU:");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("MHz FLASH:");
  Serial.print(ESP.getFlashChipSize() >> 20);
  Serial.println("MB");
  Serial.print("IRAM:");
  Serial.print(ESP.getHeapSize() >> 10);
  Serial.print("KB");
#if BOARD_HAS_PSRAM
  Serial.print(" PSRAM:");
  Serial.print(esp_spiram_get_size() >> 20);
  Serial.print("MB");
#endif
  Serial.println();
}

BLEScan* pBLEScan;
BeaconList beaconList = BeaconList(5);

class MyAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        string address = advertisedDevice->getAddress().toString().c_str();
        int rssi = advertisedDevice->getRSSI();
        beaconList.add(Beacon(address, rssi));
    }
};

void processBleBeacons() {
    if(pBLEScan->isScanning() == false) {
        pBLEScan->start(0, nullptr, false);
    }
    store.log(PID_BLE, beaconList.toCsvString().c_str());
}

void initBleBeacons() {
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(97);
    pBLEScan->setWindow(37);
    pBLEScan->setMaxResults(0);     
}

unsigned int idFromFileName(string name) {
    char suffix[] = ".CSV";
    int idxEnd = strlen(name) - strlen(suffix);  
    char idbuf[24];
    strncpy(idbuf, name, idxEnd);
    unsigned int id = atoi(idbuf);
    return id;
}

void setup()
{
    delay(500);
 
    Serial.begin(115200);
    showSysInfo(); 

    

#ifdef PIN_LED
    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED, HIGH);
#endif

#if ENABLE_BLE
    ble_init();
#endif

#if USE_OBD
// Removed
#else
    sys.begin(true, USE_GNSS >= 3);
#endif

    //initMesh();

#if USE_MEMS
    if (!logger.checkState(STATE_MEMS_READY)) do {
        Serial.print("MEMS:");
        mems = new ICM_42627;
        byte ret = mems->begin();
        if (ret) {
            logger.setState(STATE_MEMS_READY);
            Serial.println("ICM-42627");
            break;
        }
        delete mems;
        mems = new ICM_20948_I2C;
        ret = mems->begin();
        if (ret) {
            logger.setState(STATE_MEMS_READY);
            Serial.println("ICM-20948");
            break;
        }
        delete mems;
        mems = new MPU9250;
        ret = mems->begin();
        if (ret) {
            logger.setState(STATE_MEMS_READY);
            Serial.println("MPU-9250");
            break;
        } 
        Serial.println("NO");
    } while (0);
#endif

#if STORAGE == STORAGE_SD
    Serial.print("SD:");
    int volsize = store.begin();
    if (volsize > 0) {
      Serial.print(volsize);
      Serial.println("MB");
      logger.setState(STATE_STORE_READY);
    } else {
      Serial.println("NO");
    }
#elif STORAGE == STORAGE_SPIFFS
    Serial.print("SPIFFS...");
    int freebytes = store.begin();
    if (freebytes >= 0) {
      Serial.print(freebytes >> 10);
      Serial.println(" KB free");
      logger.setState(STATE_STORE_READY);
      listDir(SPIFFS, "/", 0);
    } else {
      Serial.println("NO");
    }
#endif

#if ENABLE_WIFI_STATION || ENABLE_WIFI_AP
#if ENABLE_HTTPD
// Removed
#endif
    serverCheckup();
#if ENABLE_NMEA_SERVER
    nmeaServer.begin();
#endif
#endif

server.serveStatic("/fs", SD, "/DATA");

server.on("^\\/delete\\/(.+)$", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String fileName = request->pathArg(0);
    Serial.println("Deletion of " + fileName + " requested!");
    if (removal) {
        request->send(200);
    } else {
        request->send(500);
    }    
});

server.on("^\\/list$", HTTP_GET, [] (AsyncWebServerRequest *request) {
    fs::File root = SD.open("/DATA");
    
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response.setCode(200);
    response->print("[");

    fs::File file = root.openNextFile();
    bool firstElement = true;
    while(file) {
        if(file.isDirectory()){
            continue
        }

        unsigned int id = idFromFileName(file.name());
        
        if(!firstElement) {
            response->print(",");
        }
        firstElement = false;
        response->print("{");
        response->printf("\"name\":\"%s\",", file.name());
        
        if (id == fileid) {
           response->print("\"active\":\"true\","); 
        }

        response->printf("\"size\":\"%u\"", file.size());
        response->print("}");

        file = root.openNextFile();
    }

    response->print("]");
    request->send(response);  
});

server.begin();

#ifdef PIN_LED
    pinMode(PIN_LED, LOW);
#endif

    initBleBeacons();

    logger.init();
}

void loop()
{
#if USE_OBD
// Removed
#endif

    // if file not opened, create a new file
    if (logger.checkState(STATE_STORE_READY) && !logger.checkState(STATE_FILE_READY)) {
      fileid = store.open();
      if (fileid) {
        logger.setState(STATE_FILE_READY);
      }
    }

    uint32_t ts = millis();

#if USE_GNSS == 1 || USE_GNSS == 2
    if (logger.checkState(STATE_GPS_FOUND)) {
        logger.processGPSData();
    }
#elif USE_GNSS >= 3
    if (logger.checkState(STATE_CELL_GPS_FOUND)) {
      logger.processCellGPS();
    }
#endif

    // poll and log OBD data
    store.setTimestamp(ts);
#if USE_OBD
// Removed
#if USE_GNSS == 1 || USE_GNSS == 2
        if (logger.checkState(STATE_GPS_FOUND)) {
            logger.processGPSData();
        }
#endif
        processBLE(0);
        if (tier > 1) break;
    }
#endif

#if USE_MEMS
    if (logger.checkState(STATE_MEMS_READY)) {
      bool updated;
#if ENABLE_ORIENTATION
      updated = mems->read(acc, gyr, mag, &temp, &ori);
      if (updated) {
        Serial.print("Orientation: ");
        Serial.print(ori.yaw, 2);
        Serial.print(' ');
        Serial.print(ori.pitch, 2);
        Serial.print(' ');
        Serial.println(ori.roll, 2);
        store.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
        store.log(PID_GYRO, (int16_t)(gyr[0] * 100), (int16_t)(gyr[1] * 100), (int16_t)(gyr[2] * 100));
        store.log(PID_ORIENTATION, (int16_t)(ori.yaw * 100), (int16_t)(ori.pitch * 100), (int16_t)(ori.roll * 100));
      }
#else
      updated = mems->read(acc, gyr, mag, &temp);
      if (updated) {
        store.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
        store.log(PID_GYRO, (int16_t)(gyr[0] * 100), (int16_t)(gyr[1] * 100), (int16_t)(gyr[2] * 100));
      }
#endif
    }
#endif

#if USE_OBD
// Removed
#endif

#if !ENABLE_SERIAL_OUT
    showStats();
#endif

    processBleBeacons();

#if ENABLE_HTTPD
// Removed
#endif

#if ENABLE_WIFI_AP || ENABLE_WIFI_STATION
    serverCheckup();
#endif

#if ENABLE_NMEA_SERVER
// Removed
#if ENABLE_HTTPD
// Removed
#else
        delay(1);
#endif
    } while (millis() - ts < MIN_LOOP_TIME);
#else
    ts = millis() - ts;
#if ENABLE_HTTPD
// Removed
#else
    if (ts < MIN_LOOP_TIME) delay(MIN_LOOP_TIME - ts);
#endif
#endif

//processBLE(0);
}