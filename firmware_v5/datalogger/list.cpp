#include <string>
#include "list.h"
using std::string;

Beacon::Beacon()
    : address(""),rssi(-300) {
}

Beacon::Beacon(string address, int rssi)
    : address(address),rssi(rssi) {
}

BeaconList::BeaconList(int maxSize)
    : maxSize(maxSize) {
}

Beacon BeaconList::get(int pos) {
    set<Beacon>::iterator it = this->beacons.begin();
    std::advance(it, pos);
    return *it;
}

int BeaconList::size() {
    return this->beacons.size();
}

void BeaconList::add(Beacon b) {
    this->beacons.erase(b);
    this->beacons.insert(b);

    if (this->beacons.size() > this->maxSize) {
        this->beacons.erase(this->beacons.begin());
    }
}