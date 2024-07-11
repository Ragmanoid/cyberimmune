#include "../include/validations.h"
#include "../include/helpers.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"

#include <stdio.h>
#include <math.h>

#define INACCURACY_CARGO 5 // Погрешность расстояния до сброса, м
#define MAX_SPEED 2 // Максимальная скорость, м/c
#define MAX_PERMITTED_DIST_KILL_SWITCH 3 // Максимальное отклонение от траектории для выключения, м
#define MAX_PERMITTED_DIST_CHANGE_WP 1 // Максимальное отклонение от траектории для выключения, м
#define MAX_ALT_DIF 20 // Погрешность высоты для переноса точки, см
#define MAX_ALT_KILL_SWITCH 100 // Погрешность высоты для выключения двигателей, см

// Для включения/отключения логгирования
#define LOG_SPEED 0
#define LOG_CARGO 0
#define LOG_POS 1
#define LOG_ALT 1

bool lastCargoValue = true;

long double lastChangeAltitudeTime = currentTime();

int validateSpeed(DynamicPosition position) {
    double currentSpeed = getCurrentSpeed(position);

    if (LOG_SPEED) {
        fprintf(stderr, "[%s] DEBUG: Current speed: %.5f\n",
                ENTITY_NAME,
                currentSpeed);
    }

    if (currentSpeed > MAX_SPEED + 0.5) {
        changeSpeed(MAX_SPEED);
    } 

    return 1;
}

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


//    dronePosition.altitude = nextWaypoint.altitude;

    double dist = getDistance(prevWaypoint, nextWaypoint, dronePosition);


    double b = getDistance(prevWaypoint, dronePosition);
    double c = getDistance(prevWaypoint, nextWaypoint);
    double a = getDistance(nextWaypoint, dronePosition);
    double cos_gamma = (a * a + b * b - c * c) / (2 * a * b);

    status = 0;

    if (cos_gamma <= 0 && dist < MAX_PERMITTED_DIST_KILL_SWITCH)
        status = 1;
    else if (cos_gamma > 0 && (b < MAX_PERMITTED_DIST_KILL_SWITCH || a < MAX_PERMITTED_DIST_KILL_SWITCH))
        status = 1;
    else status = 0;

    if (!status)
    {
        setKillSwitch(0);
        if (LOG_POS) {
        fprintf(stderr, "[%s] DEBUG: KILL SWITCH\n",
                ENTITY_NAME);
        fprintf(stderr, "[%s] DEBUG: dist = %.5f, cos_gamma = %.5f, a = %.5f, b = %.5f, status = %d\n",
                ENTITY_NAME,
                dist,
                cos_gamma,
                a,
                b,
                status);
        }
        return 0;
    }


    
    return 1;
}

int validateAltitude(Position dronePosition, Position nextWaypoint)
{
    static int error_count = 0;
    if (
        (dronePosition.altitude - nextWaypoint.altitude) > MAX_ALT_DIF &&
        abs(dronePosition.altitude - nextWaypoint.altitude) < MAX_ALT_KILL_SWITCH && 
        currentTime() - lastChangeAltitudeTime > 700)
    {
        if (changeAltitude(nextWaypoint.homeAltitude)) {
            lastChangeAltitudeTime = currentTime();
            if (LOG_ALT) {
                fprintf(stderr, "[%s] DEBUG: Drone altitude = %d, wp altitude = %d, dif = %d\n",
                            ENTITY_NAME,
                            dronePosition.altitude,
                            nextWaypoint.altitude,
                            abs(dronePosition.altitude - nextWaypoint.altitude));
                fprintf(stderr, "[%s] DEBUG: ALT IS CHANGED!\n",
                        ENTITY_NAME);
            }
            
        }
        return 1;
    }
    else if ((dronePosition.altitude - nextWaypoint.altitude) > MAX_ALT_KILL_SWITCH)
    {
        setKillSwitch(0);
        fprintf(stderr, "[%s] DEBUG: Drone altitude = %d, wp altitude = %d, dif = %d\n",
                            ENTITY_NAME,
                            dronePosition.altitude,
                            nextWaypoint.altitude,
                            abs(dronePosition.altitude - nextWaypoint.altitude));
        fprintf(stderr, "[%s] DEBUG: KILL SWITCH ALTITUDE\n",
                ENTITY_NAME);
        return 0;
    }
    return 1;
}