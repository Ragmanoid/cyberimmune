#include "../include/helpers.h"
#include "../../shared/include/ipc_messages_navigation_system.h"

#include <math.h>
#include <chrono>
#include <stdio.h>

#define R_EARTH 6371009
#define M_PI 3.14159265358979323846

double toRadiansAngle(double coord) {
    return coord * 10e-7 * M_PI / 180;
}

double toMetersAltitude(double altitude) {
    return altitude * 10e-2;
}

double getDistance(double lon, double dLon, double lat, double dLat) {
    return 2 * R_EARTH * sqrt(
            pow(sin(dLon / 2) * cos(lat), 2) +
            pow(cos(dLon / 2) * sin(dLat / 2), 2)
    );
}

double getDistance(Position pos1, Position pos2) {
    double dLon = toRadiansAngle(pos2.longitude - pos1.longitude);
    double dLat = toRadiansAngle(pos2.latitude - pos1.latitude);
    double dAlt = toMetersAltitude(pos2.altitude - pos1.altitude);

    return sqrt(
            pow(getDistance(
                    toRadiansAngle(pos1.longitude),
                    dLon,
                    toRadiansAngle(pos1.latitude),
                    dLat), 2) +
            pow(dAlt, 2)
    );
}

Position getCopterPosition(Position position) {
    int32_t latitude, longitude, altitude;

    if (!getCoords(latitude, longitude, altitude)) {
        fprintf(stderr, "[%s] Warning: Failed to get coordinates\n", ENTITY_NAME);
        return position;
    }

    fprintf(stderr, "[%s] Copter position:\n\tLatitude: %.5f\n\tLongitude: %.5f \n\tAltitude: %.5f cm\n",
            ENTITY_NAME,
            latitude / 1e7,
            longitude / 1e7,
            altitude);

    position.latitude = latitude;
    position.longitude = longitude;
    position.altitude = altitude;

    return position;
}

long currentTime() {
    namespace sc = std::chrono;

    auto time = sc::system_clock::now(); // get the current time

    auto since_epoch = time.time_since_epoch(); // get the duration since epoch

    return sc::duration_cast<sc::milliseconds>(since_epoch);
}

double getCurrentSpeed(DynamicPosition dynamicPosition) {
    long deltaTime = (currentTime() - dynamicPosition.lastTimeUpdatePosition) / 1000.0;
    double distance = getDistance(dynamicPosition.currentPosition,
                                  dynamicPosition.lastPosition);

    return distance / deltaTime;
}