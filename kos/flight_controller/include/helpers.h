#pragma once

#include "./mission.h"

#include <stdint.h>

struct DynamicPosition {
    Position currentPosition;
    Position lastPosition;
    long double lastTimeUpdatePosition;
};


double getDistance(Position pos1, Position pos2);

Position getCopterPosition(Position position);

/// Текущее время в мс
long double currentTime();

/// Текущая скорость в м/с
double getCurrentSpeed(DynamicPosition dynamicPosition);

void printPosition(Position position);
double getDistance(Position p1, Position p2, Position point);