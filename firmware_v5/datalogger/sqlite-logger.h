#include "list.h"
#include <FS.h>
#include <sqlite3.h>

class SqliteLogger {
private:
    uint32_t id = 0;
    int getFileId(File& root);
    string filePath = "";
    sqlite3 *db;  
    int insertLog();
    int insertBeacon(string mac);
    void insertBeaconLog(int logid, int beaconid, int rssi);
    int getBeaconIdIfExists(string mac);
public:
    float Lat = 0.0;
    float Long = 0.0;
    int AccX = 0;
    int AccY = 0;
    int AccZ = 0;
    int Time = 0;
    int Date = 0;
    BeaconList Beacons = BeaconList(10);

    void write();
    void reset();
    int begin();
    uint32_t open();
    void close();
    uint32_t size();
    uint32_t openNext();
};

class FetchIdCallback {
private:
    static int id;
public:
    static int cb(void *data, int argc, char **argv, char **azColName){
        for (int i = 0; i<argc; i++){
            //Serial.print("cb i=");
            //Serial.print(i);
            //Serial.print(" col=");
            //Serial.print(azColName[i]);
            //Serial.print(" argv=");
            //Serial.print(argv[i]);
            //Serial.print(" strcmp=");
            //Serial.println(strcmp(azColName[i],"id"));

            if (!strcmp(azColName[i],"id")) {
               //Serial.print("col==id ");
               id = atoi(argv[i]);
               //Serial.println(id);
               return 0;
            }         
        } 
        return 0;
    }
    static void reset() {
        id = 0;
    }
    static int readAndReset() {
        //Serial.print("readReset tmp=");
        int tmp = id;
        //Serial.print(tmp);
        //Serial.print(" id=");
        //Serial.print(id);
        id = 0;
        //Serial.print(" tmp=");
        //Serial.println(tmp);
        return tmp;
    }
};
