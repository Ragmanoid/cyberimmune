#pragma once

#include "./mission.h"

#include <stdint.h>

struct DynamicPosition {
    Position currentPosition;
    Position lastPosition;
    long double lastTimeUpdatePosition;
};

/// Получить расстояние между точками
double getDistance(Position pos1, Position pos2);

/// Получить позицию дрона
Position getCopterPosition(Position position);

/// Получить текущее время в мс
long double currentTime();

/// Получить текущую скорость в м/с
double getCurrentSpeed(DynamicPosition dynamicPosition);

/// Логгирование структуры Position
void printPosition(Position position);

/// Получить расстояние от точки до отрезка.
/// p1, p2 - координаты концов отрезка.
/// point - точка до которой считается расстояние.
double getDistance(Position p1, Position p2, Position point);