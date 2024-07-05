#include "../include/helpers.h"
#include "../../shared/include/ipc_messages_navigation_system.h"

#include <coresrv/time/time_api.h>
#include <rtl/rtc.h>
#include <math.h>
#include <chrono>
#include <stdio.h>

#define R_EARTH 6371009
#define M_PI 3.14159265358979323846

#define LOG_POSITION 0

double toRadiansAngle(int coord) {
    return coord * 1e-7 * M_PI / 180;
}

double toMetersAltitude(int altitude) {
    return altitude * 1e-2;
}

double getDistance(double lon, double dLon, double lat, double dLat) {
    return 2 * R_EARTH * sqrt(
            pow(sin(dLon / 2) * cos(lat), 2) +
            pow(cos(dLon / 2) * sin(dLat / 2), 2)
    );
}

double getDistance(Position pos1, Position pos2) {
    double dLon = toRadiansAngle(abs(pos2.longitude - pos1.longitude));
    double dLat = toRadiansAngle(abs(pos2.latitude - pos1.latitude));
    double dAlt = toMetersAltitude(abs(pos2.altitude - pos1.altitude));

    return sqrt(
            pow(getDistance(
                    toRadiansAngle(pos1.longitude),
                    dLon,
                    toRadiansAngle(pos1.latitude),
                    dLat), 2) +
            pow(dAlt, 2)
    );
}


// review: lapin.m
Position getCopterPosition(Position position) {
    int latitude, longitude, altitude;

    if (!getCoords(latitude, longitude, altitude)) {
        fprintf(stderr, "[%s] Warning: Failed to get coordinates\n", ENTITY_NAME);
        return position;
    }

    position.latitude = latitude;
    position.longitude = longitude;
    position.altitude = altitude;


    if (LOG_POSITION) {
        fprintf(stderr, "[%s] Copter position:\n\tLatitude: %.5f\n\tLongitude: %.5f \n\tAltitude: %.5f\n",
                ENTITY_NAME,
                position.latitude / 1e7,
                position.longitude / 1e7,
                position.altitude / 1e2);
    }

    return position;
}

// review: lapin.m
long double currentTime() {
    RtlTimeSpec time;

    if (KnGetSystemTime(&time) == rcOk) {
//        fprintf(stderr, "[%s] DEBUG Current time %f\n",
//                ENTITY_NAME,
//                time.sec * 1e3 + time.nsec / 1e6);
    } else {
        fprintf(stderr, "[%s] ERROR get time\n", ENTITY_NAME);
    }

    return time.sec * 1e3 + time.nsec / 1e6;
}

double getCurrentSpeed(DynamicPosition dynamicPosition) {
    long double deltaTime = (currentTime() - dynamicPosition.lastTimeUpdatePosition) / 1000.0;
    double distance = getDistance(dynamicPosition.currentPosition,
                                  dynamicPosition.lastPosition);

    return distance / deltaTime;
}