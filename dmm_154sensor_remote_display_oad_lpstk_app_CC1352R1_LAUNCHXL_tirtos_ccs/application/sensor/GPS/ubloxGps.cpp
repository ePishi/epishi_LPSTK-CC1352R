#include <stdio.h>
#include "sensor/GPS/ubloxGps.h"
#include "sensor/GPS/SparkFun_Ublox_Arduino_Library.hpp"

static GpsHandle gpsHandle = NULL;
static GpsLocation gpsLocation = {0};
//static DebugStream debugStream;

void ubloxGps_init(void) {
}

GpsHandle ubloxGps_open(I2C_Handle i2cHandle) {
    if (i2cHandle == NULL)
        return NULL;

    if (gpsHandle == NULL) {
        SFE_UBLOX_GPS* ubloxGpsP = new SFE_UBLOX_GPS();
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

        gpsHandle = new GpsConfig();
        gpsHandle->i2cHandle = i2cHandle;
        gpsHandle->ubloxGpsP = ubloxGpsP;
    }

    return gpsHandle;
}

GpsLocation* ubloxGps_getLocation(GpsHandle gpsHandle) {
    if (gpsHandle == NULL)
        return NULL;

    SFE_UBLOX_GPS* ubloxGpsP = (SFE_UBLOX_GPS*) gpsHandle->ubloxGpsP;
    if (ubloxGpsP != NULL) {
        gpsLocation.latitude    = ubloxGpsP->getLatitude();
        gpsLocation.longitude   = ubloxGpsP->getLongitude();
        gpsLocation.altitude    = ubloxGpsP->getAltitude();
        gpsLocation.altitudeMsl = ubloxGpsP->getAltitudeMSL();
    }

    return &gpsLocation;
}
