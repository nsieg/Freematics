#include <string>
#include <memory>
#include <set>
using std::string;
using std::set;

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
    private:
        int maxSize;
        set<Beacon> beacons;
};