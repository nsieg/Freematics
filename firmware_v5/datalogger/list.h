#include <string>
#include <memory>
#include <set>
using std::string;
using std::set;

#define PID_BLE 0xAA

class Beacon {
    public:
        Beacon();
        Beacon(string address, int rssi);
        const string address;
        const int rssi;  
        bool operator<(const Beacon& b) const {
            return this->address != b.address && this->rssi <= b.rssi;
        }   
        bool operator==(const Beacon& b) const {
            return this->address == b.address && this->rssi == b.rssi;
        }    
};

class BeaconList {
    public:
        BeaconList(int maxSize);
        Beacon get(int pos);
        void add(Beacon b);
        int size();
        string toCsvString();        
    private:
        int maxSize;
        set<Beacon> beacons;
};