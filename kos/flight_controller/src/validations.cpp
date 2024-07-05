#include "../include/validations.h"
#include "../include/helpers.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"

#include <stdio.h>

#define INACCURACY_CARGO 5 // Погрешность расстояния до сброса, м
#define MAX_SPEED 5 // Максимальная скорость, м/c

#define LOG_SPEED 0
#define LOG_CARGO 0

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