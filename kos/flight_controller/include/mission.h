#pragma once

#include <stdint.h>

/// HOME - Точка дома;
/// TAKEOFF - Взлёт;
/// WAYPOINT - Точка полёта;
/// LAND - Посадка;
/// SET_SERVO - Активировать сервопривод;
enum CommandType {
    HOME,
    TAKEOFF,
    WAYPOINT,
    LAND,
    SET_SERVO
};

/// DTO - взлёта
struct CommandTakeoff {
    int32_t altitude;

    CommandTakeoff(int32_t alt) {
        altitude = alt;
    }
};

/// DTO - точки полёта
struct CommandWaypoint {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;

    CommandWaypoint(int32_t lat, int32_t lng, int32_t alt) {
        latitude = lat;
        longitude = lng;
        altitude = alt;
    }
};


/// DTO - активации сервопривода
struct CommandServo {
    int32_t number;
    int32_t pwm;

    CommandServo(int32_t num, int32_t pwm_) {
        number = num;
        pwm = pwm_;
    }
};

/// DTO - данных миссии
union CommandContent {
    CommandTakeoff takeoff;
    CommandWaypoint waypoint;
    CommandServo servo;
};

/// DTO - данных миссии с типом миссии
struct MissionCommand {
    CommandType type;
    CommandContent content;
};

/// DTO - позиции
struct Position {
    int latitude;
    int longitude;
    int altitude;
    int homeAltitude;

    Position(int lat, int lng, int alt): latitude(lat), longitude(lng), altitude(alt) {
    }

    Position(int lat, int lng, int alt, int homeAlt): latitude(lat), longitude(lng), altitude(alt), homeAltitude(homeAlt) {
    }
};

/// Парсиинг миссии
int parseMission(char *response);

/// Логгирование миссии
void printMissions();

/// Добавить к координатам точки сброса груза, координату позиции дома
void addHomeAltitudeToCargo(int32_t homeAltitude);

/// Получить позицию сброса груза
Position getCargoPosition();

/// Получить список всех позиций
Position *getPositions();

/// Сохранить миссию в позиции
void saveMissionsToPositions(int homeAltitude);

/// Получить список команд в миссии
MissionCommand* getCommands();

/// Получить число команд в миссии
uint32_t getNumCommands();

/// Получить число позиций в миссии
uint32_t getWaypointCount();