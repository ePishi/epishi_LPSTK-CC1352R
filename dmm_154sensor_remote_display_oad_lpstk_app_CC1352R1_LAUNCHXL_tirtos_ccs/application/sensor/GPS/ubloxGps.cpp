#include <stdio.h>
#include "sensor/GPS/SparkFun_Ublox_Arduino_Library.hpp"
#include "sensor/GPS/ubloxGps.h"

static SFE_UBLOX_GPS* ubloxGpsP = NULL;
static GpsLocation gpsLocation;
//static DebugStream debugStream;

void ubloxGps_init(void) {
}

bool ubloxGps_open(I2C_Handle i2cHandle) {

    if (ubloxGpsP == NULL) {
        ubloxGpsP = new SFE_UBLOX_GPS();
        //ubloxGpsP->setDebugStream(&debugStream);
        //ubloxGpsP->enableDebugging();
        //ubloxGpsP->enableNMEAOutput();
        /*
         * The library's begin() function returns the result of isConnected()
         * which is a zero read and write I2C transaction. TI doesn't seem to
         * like this and always fails. So we just call begin() and ignore the
         * return status
        if (!ubloxGps.begin(i2cHandle)) {
            printf("ubloxGps.begin() failed\n");
            return false;
        }
        */
        ubloxGpsP->begin(i2cHandle);
    }

    return true;
}

GpsLocation* ubloxGps_getLocation(void) {
    if (ubloxGpsP != NULL) {
        gpsLocation.latitude    = ubloxGpsP->getLatitude();
        gpsLocation.longitude   = ubloxGpsP->getLongitude();
        gpsLocation.altitude    = ubloxGpsP->getAltitude();
        gpsLocation.altitudeMsl = ubloxGpsP->getAltitudeMSL();
    }

    return &gpsLocation;
}
