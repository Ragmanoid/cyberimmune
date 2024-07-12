#pragma once

#include "./mission.h"

#include <stdint.h>

#define R_EARTH 6371009
#define M_PI 3.14159265358979323846
#define RETRY_DELAY_SEC 1

#define LOG_POSITION 0

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

/// Проверка разрешения на полет.
/// missionIsPaused - текущее состояние полета
bool needPauseMission(bool missionIsPaused);

/// Отправка логов
void sendLogs(char *message);