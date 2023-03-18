#include "ESPAsyncWebServer.h"
#include <SPI.h>
#include <FS.h>
#include <SD.h>

// id of active file from datalogger.ino
extern uint32_t fileid;

void onDelete(AsyncWebServerRequest *request) {
    String fileName = request->pathArg(0);
    Serial.println("Deletion of " + fileName + " requested!");
    bool removal = SD.remove("/DATA/" + fileName);
    if (removal) {
        request->send(200);
    } else {
        request->send(500);
    }   
}

void onList(AsyncWebServerRequest *request) {
    fs::File root = SD.open("/DATA");
    
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->setCode(200);
    response->print("[");

    fs::File file = root.openNextFile();
    bool firstElement = true;
    while(file) {
        if(file.isDirectory()){
            continue;
        }

        unsigned int id = atoi(file.name());
        
        if(!firstElement) {
            response->print(",");
        }
        firstElement = false;
        response->print("{");
        response->printf("\"name\":\"%s\",", file.name());
        
        if (id == fileid) {
           response->print("\"active\":true,"); 
        }

        response->printf("\"size\":%u", file.size());
        response->print("}");

        file = root.openNextFile();
    }

    response->print("]");
    request->send(response);  
}
