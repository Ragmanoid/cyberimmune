#include "../include/helpers.h"
#include "../include/signedMessages.h"
#include "../../shared/include/ipc_messages_navigation_system.h"

#include <coresrv/time/time_api.h>
#include <rtl/rtc.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


long double currentTime();

long double lastUpdateTime = currentTime();
long double checkPauseLastTime = currentTime();

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
    if (currentTime() - lastUpdateTime < 300)
        return position;

    int latitude, longitude, altitude;

    if (!getCoords(latitude, longitude, altitude)) {
        fprintf(stderr, "[%s] Warning: Failed to get coordinates\n", ENTITY_NAME);
        return position;
    }

    lastUpdateTime = currentTime();

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

    if (KnGetSystemTime(&time) != rcOk) {
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

void printPosition(Position position) {
    fprintf(stderr, "[%s] Info Position:\n\tLatitude: %.5f\n\tLongitude: %.5f \n\tAltitude: %.5f\n",
            ENTITY_NAME,
            position.latitude / 1e7,
            position.longitude / 1e7,
            position.altitude / 1e2);
}

double getDistance(Position p1, Position p2, Position point) {
    double x1 = toRadiansAngle(p1.latitude);
    double y1 = toRadiansAngle(p1.longitude);
    double z1 = toMetersAltitude(p1.altitude);

    double x2 = toRadiansAngle(p2.latitude);
    double y2 = toRadiansAngle(p2.longitude);
    double z2 = toMetersAltitude(p2.altitude);

    double dx = getDistance(x2, fabs(x2 - x1), y2, 0);
    double dy = getDistance(x2, 0, y2, y2 - y1);
    double d = getDistance(x2, fabs(x2 - x1), y2, fabs(y2 - y1));
    double dz = fabs(z2 - z1);

    double xc = toRadiansAngle(point.latitude);
    double yc = toRadiansAngle(point.longitude);
    double zc = toMetersAltitude(point.altitude);

    double dxc = getDistance(xc, fabs(xc - x1), yc, 0);
    double dyc = getDistance(xc, 0, yc, fabs(yc - y1));
    double dzc = fabs(zc - z1);

    double t = (dxc * dx + dyc * dy + dzc * dz) / (dx * dx + dy * dy + dz * dz);

    if (t > 1)
        t = 1;
    else if (t < 0)
        t = 0;

    double distance = sqrt(
            pow(-dxc + dx * t, 2) +
            pow(-dyc + dy * t, 2) +
            pow(-dzc + dz * t, 2));

    return distance;
}


bool needPauseMission(bool missionIsPaused) {
    // Arm: 1 - disarm
    // Arm: 0 - arm

    char armRespone[1024] = {0};
    if (missionIsPaused) {
        sendSignedMessage("/api/fly_accept", armRespone, "fly_accept", RETRY_DELAY_SEC);
    } else if (!sendFastSignedMessage("/api/fly_accept", armRespone)) {
        checkPauseLastTime = currentTime();
        return 0;
    }

    checkPauseLastTime = currentTime();

    if (strstr(armRespone, "$Arm: 0#") != NULL) {
        return 0;
    } else if (strstr(armRespone, "$Arm: 1#") != NULL) {
        return 1;
    }

    fprintf(stderr, "[%s] Error: needPauseMission - unknowrn response\n", ENTITY_NAME);
    return 0;
}