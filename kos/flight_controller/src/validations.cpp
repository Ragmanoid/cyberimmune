#include "../include/validations.h"
#include "../include/helpers.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"

#include <stdio.h>

#define INACCURACY_CARGO 0.5 // Погрешность расстояния до сброса, м
#define MAX_SPEED 8 // Максимальная скорость, м/c

#define LOG_SPEED 0
#define LOG_CARGO 0

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

int validateCargo(Position copter, Position cargo) {
    double distance = getDistance(copter, cargo);

    if (LOG_CARGO) {
        fprintf(stderr, "[%s] DEBUG: Distance to cargo: %.5f\n",
                ENTITY_NAME,
                distance);
    }

    return setCargoLock(distance < INACCURACY_CARGO);
}