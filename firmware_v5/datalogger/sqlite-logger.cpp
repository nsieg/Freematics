#include "sqlite-logger.h"
#include <FreematicsPlus.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <sqlite3.h>

int FetchIdCallback::id = 0;

int SqliteLogger::insertLog() {
    char dtbuff[30];
    sprintf(dtbuff, "20%02u-%02u-%02u %02u:%02u:%02u.%03u",
        this->Date % 100, (this->Date % 10000) / 100, this->Date / 10000, 
        this->Time / 1000000, (this->Time % 1000000) / 10000, (this->Time % 10000) / 100, this->Time % 100);

    char sql[300];
    sprintf(sql, "INSERT INTO logs (latitude,longitude,acc_x,acc_y,acc_z,millis) VALUES ('%f','%f','%i','%i','%i',unixepoch('%s'))", 
        this->Lat, this->Long, this->AccX, this->AccY, this->AccZ, dtbuff);

    //Serial.println(sql);    
    int rc = sqlite3_exec(this->db, sql, NULL, NULL, NULL);
    if (rc != SQLITE_OK) {
       Serial.printf("Error executing sql: %s\n", sqlite3_errmsg(this->db));
    }

    //Serial.println("SELECT MAX(id) AS id FROM logs");
    rc = sqlite3_exec(this->db, "SELECT MAX(id) AS id FROM logs", FetchIdCallback::cb, NULL, NULL);
    if (rc != SQLITE_OK) {
       Serial.printf("Error executing sql: %s\n", sqlite3_errmsg(this->db));
    }

    return FetchIdCallback::readAndReset();
}

int SqliteLogger::getBeaconIdIfExists(string mac) {
    char sql[100];
    sprintf(sql, "SELECT id,mac FROM beacon WHERE mac='%s'", mac.c_str());

    //Serial.println(sql);
    int rc = sqlite3_exec(this->db, sql, FetchIdCallback::cb, NULL, NULL);
    if (rc != SQLITE_OK) {
       Serial.printf("Error executing sql: %s\n", sqlite3_errmsg(this->db));
    }
    return FetchIdCallback::readAndReset();
}

int SqliteLogger::insertBeacon(string mac) {
    char sql[100];
    sprintf(sql, "INSERT INTO beacon (mac) VALUES ('%s')", mac.c_str());

    //Serial.println(sql);
    int rc = sqlite3_exec(this->db, sql, NULL, NULL, NULL);
    if (rc != SQLITE_OK) {
       Serial.printf("Error executing sql: %s\n", sqlite3_errmsg(this->db));
    }

    //Serial.println("SELECT MAX(id) AS id FROM beacon");
    rc = sqlite3_exec(this->db, "SELECT MAX(id) AS id FROM beacon", FetchIdCallback::cb, NULL, NULL);
    if (rc != SQLITE_OK) {
       Serial.printf("Error executing sql: %s\n", sqlite3_errmsg(this->db));
    }

    return FetchIdCallback::readAndReset();
}

void SqliteLogger::insertBeaconLog(int logid, int beaconid, int rssi) {
    char sql[200];
    sprintf(sql, "INSERT INTO logbeacon (logid, beaconid, rssi) VALUES ('%i','%i','%i')", logid, beaconid, rssi);

    //Serial.println(sql);
    int rc = sqlite3_exec(this->db, sql, NULL, NULL, NULL);
    if (rc != SQLITE_OK) {
       Serial.printf("Error executing sql: %s\n", sqlite3_errmsg(this->db));
    }
}

void SqliteLogger::write() {
    if (this->Lat == 0.0 || this->Long == 0.0 || this->Time == 0) {
        //Serial.println("Lat/Lon/Time not ready -> skip log");
        this->reset();
        return;
    }

    int logid = insertLog();
    //Serial.print("Got ");
    //Serial.print(this->Beacons.size());
    //Serial.println(" beacons");
    for (int i = 0; i < this->Beacons.size(); i++) {   
        int beaconid = getBeaconIdIfExists(this->Beacons.get(i).address.c_str());
        //Serial.print("id=");
        //Serial.println(beaconid);
        if (beaconid == 0) {
            beaconid = insertBeacon(this->Beacons.get(i).address.c_str());
        }

        insertBeaconLog(logid, beaconid, this->Beacons.get(i).rssi);
    }
    
    this->reset();    
}

void SqliteLogger::reset() {
    this->Lat = 0.0;
    this->Long = 0.0;
    this->AccX = 0;
    this->AccY = 0;
    this->AccZ = 0;
    this->Time = 0;
    this->Beacons = BeaconList(10);    
}

int SqliteLogger::begin() {
    SPI.begin();
    bool sdReady = false;
    sdReady = SD.begin(PIN_SD_CS, SPI, SPI_FREQ);

    while (!sdReady) {
        delay(1000);
        sdReady = SD.begin(PIN_SD_CS, SPI, SPI_FREQ);            
    }

    if (sdReady) {
        return SD.cardSize() >> 20;
    } else {
        return -1;
    }
}

uint32_t SqliteLogger::open() {
    File root = SD.open("/DATA");
    this->id = getFileId(root);
    if (this->id == 0) {
        SD.mkdir("/DATA");
        this->id = 1;
    }

    this->filePath = "/DATA/" +  std::to_string(this->id) + ".db3";
    std::string dbPath = "/sd" + this->filePath;
    Serial.print("File: ");
    Serial.println(this->filePath.c_str());

    int rc = sqlite3_open(dbPath.c_str(), &this->db);
    if (rc) {
       Serial.printf("Can't open database: %s\n", sqlite3_errmsg(this->db));
       this->id = 0;
    }

    string ddl = "CREATE TABLE beacon (id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE, mac TEXT NOT NULL UNIQUE);";
    ddl += "CREATE TABLE logs (id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE, latitude REAL NOT NULL, longitude REAL NOT NULL, acc_x INTEGER, acc_y INTEGER, acc_z INTEGER, millis INTEGER NOT NULL UNIQUE);";
    ddl += "CREATE TABLE logbeacon (logid INTEGER REFERENCES logs (id), beaconid INTEGER REFERENCES beacon (id), rssi INTEGER NOT NULL);";

    rc = sqlite3_exec(this->db, ddl.c_str(), NULL, NULL, NULL);
    if (rc != SQLITE_OK) {
       Serial.printf("Error executing sql: %s\n", sqlite3_errmsg(this->db));
       this->id = 0;
    }

    return this->id;
}

int SqliteLogger::getFileId(File& root) {
    int id = 0;
    if (root) {
        File file;
        while(file = root.openNextFile()) {
            char *p = strrchr(file.name(), '/');
            unsigned int n = atoi(p ? p + 1 : file.name());
            if (n > id) id = n;
        }
        return id + 1;
    }
    return 0;    
}

void SqliteLogger::close() {
    sqlite3_close(this->db);
}

uint32_t SqliteLogger::size() {
    File f = SD.open(this->filePath.c_str(), FILE_READ);
    uint32_t size = f.size();
    f.close();
    return size;
}

uint32_t SqliteLogger::openNext() {
    this->close();
    return this->open();   
}