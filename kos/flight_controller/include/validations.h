#include "./helpers.h"

#pragma once

int validateSpeed(DynamicPosition position);

int validatePosition();

int validateCargo(DynamicPosition position, Position cargoPosition);

int validatePosition(Position dronePosition, Position prevWaypoint, Position nextWaypoint);
int validateAltitude(Position dronePosition, Position nextWaypoint);