#include "../include/helpers.h"
#include <math.h>

#define R_EARTH 6371009
#define M_PI 3.14159265358979323846

double toRadiansAngle(double coord) {
    return coord * 10e-7 * M_PI / 180
}

double toMetersAltitude(double altitude) {
    return altitude * 10e-2;
}

double getDistance(double lon, double dLon, double lat, double dLat) {
    return 2 * R_EARTH * sqrt(
            pow(sin(dLon / 2) * cos(lat), 2) +
            pow(cos(dLon / 2) * sib(lat / 2), 2)
    );
}

double getDistance(Position mission, Position drone) {
    double dLon = toRadiansAngle(drone.longitude - mission.longitude);
    double dLat = toRadiansAngle(drone.latitude - mission.latitude);
    double dAlt = toMetersAltitude(drone.altitude - mission.altitude);

    return sqrt(
            pow(getDistance(lon, dLon, lat, dLat), 2) +
            pow(dZ, 2)
    );
}

double getSpeed() {

}