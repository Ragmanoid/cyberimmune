#include "../include/validations.h"
#include "../include/helpers.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"

#include <stdio.h>
#include <math.h>

#define INACCURACY_CARGO 5 // Погрешность расстояния до сброса, м
#define MAX_SPEED 2 // Максимальная скорость, м/c
#define MAX_PERMITTED_DIST_KILL_SWITCH 6 // Максимальное отклонение от траектории для выключения, м
#define MAX_PERMITTED_DIST_CHANGE_WP 1 // Максимальное отклонение от траектории для выключения, м
#define MAX_ALT_DIF 30 // Погрешность высоты для переноса точки, см
#define MAX_ALT_KILL_SWITCH 200 // Погрешность высоты для выключения двигателей, см
#define DIST_TOLERANCE 3

// Для включения/отключения логгирования
#define LOG_SPEED 0
#define LOG_CARGO 0
#define LOG_POS 1
#define LOG_ALT 1

bool lastCargoValue = true;

long double lastAltitudeTime = currentTime();
long double lastChangePositionTime = currentTime();
long double lastSpeedLogTime = currentTime();
long double lastDirectionTime = currentTime();
int killSwitchIsPermitted = 0;

double min_dist = 1e9;

int validateSpeed(DynamicPosition position) {
    double currentSpeed = getCurrentSpeed(position);

    // if (currentTime() - lastSpeedLogTime > 5000) {
    //     char msg[100] = {0}; 
    //     snprintf(msg, 100, "Current_speed:%.2f_Current_alt:%d", currentSpeed, position.currentPosition.altitude);
    //     sendLogs(msg);
    //     lastSpeedLogTime = currentTime();
    // }

    if (LOG_SPEED) {
        fprintf(stderr, "[%s] DEBUG: Current speed: %.5f\n",
                ENTITY_NAME,
                currentSpeed);
    }

    if (currentSpeed > MAX_SPEED) {
        changeSpeed(MAX_SPEED);
        sendLogs("Set_speed");
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
        if (setCargoLock(currentCargoValue)) {
            lastCargoValue = currentCargoValue;
            if (currentCargoValue)
            {
                sendLogs("Cargo_unlocked");
                killSwitchIsPermitted = 1;
            }
            else
                sendLogs("Cargo_locked");
        }
    }

    return 1;
}

int validatePosition(Position dronePosition, Position prevWaypoint, Position nextWaypoint)
{
    int status = 0;
    static int error_count = 0;

    if (currentTime() - lastChangePositionTime > 700)
    {
        lastChangePositionTime = currentTime();
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
            //changeWaypoint(nextWaypoint.latitude, nextWaypoint.longitude, nextWaypoint.altitude);
            if (LOG_POS) {
                char msg[100] = {0}; 
                snprintf(msg, 100, "Kill_switch_position:%.5f", dist);
                sendLogs(msg);
                //sendLogs("Kill_switch_position");
                fprintf(stderr, "[%s] DEBUG: KILL SWITCH POSITION\n",
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
    }

    return 1;
}

int validateAltitude(Position dronePosition, Position nextWaypoint)
{

    if ((dronePosition.altitude - nextWaypoint.altitude) > MAX_ALT_DIF &&
        abs(dronePosition.altitude - nextWaypoint.altitude) < MAX_ALT_KILL_SWITCH)
        {
            if (currentTime() - lastAltitudeTime > 700)
            {
                lastAltitudeTime = currentTime();
                changeAltitude(nextWaypoint.homeAltitude);
                if (LOG_ALT) {
                    fprintf(stderr, "[%s] DEBUG: Drone altitude = %d, wp altitude = %d, dif = %d\n",
                                ENTITY_NAME,
                                dronePosition.altitude,
                                nextWaypoint.altitude,
                                abs(dronePosition.altitude - nextWaypoint.altitude));
                    fprintf(stderr, "[%s] DEBUG: ALT IS CHANGED!\n",
                            ENTITY_NAME);
                    char msg[100] = {0}; 
                    snprintf(msg, 100, "Alt_is_changed_to_%d", nextWaypoint.homeAltitude);
                    sendLogs(msg);
                }
            }
        }
        else if ((dronePosition.altitude - nextWaypoint.altitude) > MAX_ALT_KILL_SWITCH)
        {
            if (killSwitchIsPermitted)
            {
                setKillSwitch(0);
                fprintf(stderr, "[%s] DEBUG: Drone altitude = %d, wp altitude = %d, dif = %d\n",
                                    ENTITY_NAME,
                                    dronePosition.altitude,
                                    nextWaypoint.altitude,
                                    abs(dronePosition.altitude - nextWaypoint.altitude));
                fprintf(stderr, "[%s] DEBUG: KILL SWITCH ALTITUDE\n",
                        ENTITY_NAME);
                sendLogs("Kill_switch_altitude");
                return 0;
            }
        }

    return 1;
}

int validateDirection(Position copter, Position nextWaypoint)
{
    static int error_count = 0;
    double dist;
    if (currentTime() - lastDirectionTime > 700)
    {
        lastDirectionTime = currentTime();
        dist = getDistance(copter, nextWaypoint);
        //if (dist - min_dist > DIST_TOLERANCE && killSwitchIsPermitted)
        if (dist - min_dist > DIST_TOLERANCE)
        {
            setKillSwitch(0);
            if (LOG_POS) {
                char msg[100] = {0}; 
                snprintf(msg, 100, "Kill_switch_direction:%.5f", dist);
                sendLogs(msg);
                //sendLogs("Kill_switch_position");
                fprintf(stderr, "[%s] DEBUG: KILL SWITCH DIRECTION\n",
                        ENTITY_NAME);
                fprintf(stderr, "[%s] DEBUG: dist = %.5f, prev_dist = %.5f\n",
                        ENTITY_NAME,
                        dist);
            }
            return 0;
        }
        if (dist < min_dist) min_dist = dist;
    }
    return 1;
}