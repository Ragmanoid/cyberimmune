#include "../include/validations.h"
#include "../include/helpers.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"

#include <stdio.h>

#define INACCURACY_CARGO 5 // Погрешность расстояния до сброса, м
#define MAX_SPEED 2 // Максимальная скорость, м/c
#define MAX_PERMITTED_DIST 5 // Максимальное отклонение от траектории, м

#define LOG_SPEED 0
#define LOG_CARGO 0
#define LOG_POS 1

bool lastCargoValue = true;

// review: lapin.m
int validateSpeed(DynamicPosition position) {
    double currentSpeed = getCurrentSpeed(position);

    if (LOG_SPEED) {
        fprintf(stderr, "[%s] DEBUG: Current speed: %.5f\n",
                ENTITY_NAME,
                currentSpeed);
    }

    if (currentSpeed > MAX_SPEED) {
        changeSpeed(MAX_SPEED);
    }

    return 1;
}

// review: lapin.m
int validateCargo(DynamicPosition position, Position cargo) {
    double distance = getDistance(position.currentPosition, cargo);
    // Что за приколы в названиях функции. setCargoLock(true) - даёт сборс груза
    bool currentCargoValue = distance < INACCURACY_CARGO;

    if (LOG_CARGO) {
        fprintf(stderr, "[%s] DEBUG: Distance to cargo: %.5f\n",
                ENTITY_NAME,
                distance);
    }

    // lapin.m Вернуть если нужно замедлится перед сбросом груза
//    if (distance < 10) {
//        double currentSpeed = getCurrentSpeed(position);
//
//        if (currentSpeed > 3)
//            changeSpeed(3);
//    }

    if (lastCargoValue != currentCargoValue) {
        if (setCargoLock(currentCargoValue))
            lastCargoValue = currentCargoValue;
    }

    return 1;
}

int validatePosition(Position dronePosition, Position prevWaypoint, Position nextWaypoint)
{
    int status = 0;
    double dist = getDistance(prevWaypoint, nextWaypoint, dronePosition);


    double b = getDistance(prevWaypoint, dronePosition);
    double c = getDistance(prevWaypoint, nextWaypoint);
    double a = getDistance(nextWaypoint, dronePosition);
    double cos_gamma = (a * a + b * b - c * c) / (2 * a * b);

    if (cos_gamma <= 0 && dist < MAX_PERMITTED_DIST)
        status = 1;
    else if (cos_gamma > 0 && (b < MAX_PERMITTED_DIST || a < MAX_PERMITTED_DIST) )
        status = 1;
    else
        status = 0;

    if (!status) {
        changeWaypoint(nextWaypoint.latitude, nextWaypoint.longitude, nextWaypoint.altitude);
        if (LOG_POS) {
        fprintf(stderr, "[%s] DEBUG: Waypoint is changed to: %.5f %.5f %.5f\n",
                ENTITY_NAME,
                nextWaypoint.latitude,
                nextWaypoint.longitude,
                nextWaypoint.altitude);
    }
    }
    return 1;
}