#ifndef UBLOX_GPS_H
#define UBLOX_GPS_H

#include <ti/drivers/I2C.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct GpsConfig {
    I2C_Handle i2cHandle;
    void* ubloxGpsP;
} GpsConfig;

typedef struct GpsConfig *GpsHandle;

typedef struct {
    uint32_t latitude;
    uint32_t longitude;
    uint32_t altitude;
    uint32_t altitudeMsl;
} GpsLocation;

void ubloxGps_init(void);
GpsHandle ubloxGps_open(I2C_Handle);
GpsLocation* ubloxGps_getLocation(GpsHandle gpsHandle);

#ifdef __cplusplus
}
#endif

#endif /* UBLOX_GPS_H */
