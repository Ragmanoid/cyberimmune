#include "./helpers.h"

#pragma once

/// Валидатор скорости
int validateSpeed(DynamicPosition position);

/// Валидатор сброса груза
int validateCargo(DynamicPosition position, Position cargoPosition);

/// Валидатор позиции
int validatePosition(Position dronePosition, Position prevWaypoint, Position nextWaypoint);

/// Валидатор высоты
int validateAltitude(Position dronePosition, Position nextWaypoint);

/// Валидатор отклонения от миссии
int validateDirection(Position copter, Position nextWaypoint);