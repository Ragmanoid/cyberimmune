#pragma once
#include "./mission.h"

struct Position {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;

    Position(int32_t lat, int32_t lng, int32_t alt) {
        latitude = lat;
        longitude = lng;
        altitude = alt;
    }

    Position(CommandWaypoint waypoint) {
        latitude = waypoint.latitude;
        longitude = waypoint.longitude;
        altitude = waypoint.altitude;
    }
};


double getDistance(Position mission, Position drone);