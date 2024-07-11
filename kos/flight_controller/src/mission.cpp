#include "../include/mission.h"
#include "../include/helpers.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define COMMAND_MAX_STRING_LEN 32

uint32_t commandNum = 0;
uint32_t posCount = 0;
uint32_t waypointCount = 0;

MissionCommand *commands = NULL;
Position *positions = NULL;

int hasMission = false;

Position cargoPosition = {0, 0, 0};



int isStopSymbol(char character) {
    return ((character == '_') || (character == '&') || (character == '#'));
}

void calcCargoWaypoint(int currentI) {
    bool hasBaseCoords = false;
    cargoPosition = {0, 0, 0};

    for (; currentI >= 0; --currentI) {
        MissionCommand cmd = commands[currentI];

        if (cmd.type == CommandType::WAYPOINT) {
            if (!hasBaseCoords) {
                cargoPosition.latitude = cmd.content.waypoint.latitude;
                cargoPosition.longitude = cmd.content.waypoint.longitude;
                cargoPosition.altitude = cmd.content.waypoint.altitude;
                hasBaseCoords = true;
            }
        }

        if (cmd.type == CommandType::HOME) {
            cargoPosition.altitude += cmd.content.waypoint.altitude;
            fprintf(stderr, "[%s] Info: Cargo position lat: %d lon: %d alt: %d\n", ENTITY_NAME, cargoPosition.latitude,
                    cargoPosition.longitude, cargoPosition.altitude);
            break;
        }
    }
}

int parseInt(char *&string, int32_t &value, uint32_t numAfterPoint) {
    char stringValue[COMMAND_MAX_STRING_LEN];
    uint32_t strPtr = 0, valPtr = 0;
    while (!isStopSymbol(string[strPtr])) {
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            fprintf(stderr, "[%s] Warning: Failed to parse mission command\n", ENTITY_NAME);
            return 0;
        } else if (string[strPtr] == '.') {
            strPtr++;
            break;
        } else {
            stringValue[valPtr] = string[strPtr];
            strPtr++;
            valPtr++;
        }
    }
    int i = 0;
    while (!isStopSymbol(string[strPtr])) {
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            fprintf(stderr, "[%s] Warning: Failed to parse mission command\n", ENTITY_NAME);
            return 0;
        } else if (i < numAfterPoint) {
            stringValue[valPtr] = string[strPtr];
            strPtr++;
            valPtr++;
            i++;
        } else
            strPtr++;
    }
    string += strPtr + 1;
    for (int j = i; j < numAfterPoint; j++) {
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            fprintf(stderr, "[%s] Warning: Failed to parse mission command\n", ENTITY_NAME);
            return 0;
        } else {
            stringValue[valPtr] = '0';
            valPtr++;
        }
    }
    stringValue[valPtr] = '\0';
    value = atoi(stringValue);
    return 1;
}

int parseCommands(char *str) {
    commandNum = 0;
    for (uint32_t i = 0;; i++) {
        if (str[i] == '#')
            break;
        if (str[i] == 'H' || str[i] == 'T' || str[i] == 'W' || str[i] == 'L' || str[i] == 'S')
            commandNum++;
    }
    if (commandNum == 0) {
        fprintf(stderr, "[%s] Warning: Mission contains no commands\n", ENTITY_NAME);
        return 0;
    }

    commands = (MissionCommand *) malloc(commandNum * sizeof(MissionCommand));

    uint32_t ptr = 0;
    for (uint32_t i = 0; i < commandNum; i++) {
        uint32_t end = ptr;
        for (uint32_t j = ptr + 1;; j++)
            if (str[j] == '&' || str[j] == '#') {
                end = j;
                break;
            }
        char *stringPtr = str + ptr + 1;
        int32_t lat, lng, alt;
        switch (str[ptr]) {
            case 'H':
                if (!parseInt(stringPtr, lat, 7) || !parseInt(stringPtr, lng, 7) || !parseInt(stringPtr, alt, 2)) {
                    fprintf(stderr, "[%s] Warning: Failed to parse values for 'home' command\n", ENTITY_NAME);
                    free(commands);
                    return 0;
                }
                posCount++;
                commands[i].type = CommandType::HOME;
                commands[i].content.waypoint = CommandWaypoint(lat, lng, alt);
                break;
            case 'T':
                if (!parseInt(stringPtr, alt, 2)) {
                    fprintf(stderr, "[%s] Warning: Failed to parse values for 'takeoff' command\n", ENTITY_NAME);
                    free(commands);
                    return 0;
                }
                posCount++;
                commands[i].type = CommandType::TAKEOFF;
                commands[i].content.takeoff = CommandTakeoff(alt);
                break;
            case 'W': {
                int32_t hold;
                if (!parseInt(stringPtr, hold, 0) || !parseInt(stringPtr, lat, 7) || !parseInt(stringPtr, lng, 7) ||
                    !parseInt(stringPtr, alt, 2)) {
                    fprintf(stderr, "[%s] Warning: Failed to parse values for 'waypoint' command\n", ENTITY_NAME);
                    free(commands);
                    return 0;
                }
                posCount++;
                commands[i].type = CommandType::WAYPOINT;
                commands[i].content.waypoint = CommandWaypoint(lat, lng, alt);
                break;
            }
            case 'L':
                if (!parseInt(stringPtr, lat, 7) || !parseInt(stringPtr, lng, 7) || !parseInt(stringPtr, alt, 2)) {
                    fprintf(stderr, "[%s] Warning: Failed to parse values for 'land' command\n", ENTITY_NAME);
                    free(commands);
                    return 0;
                }
                commands[i].type = CommandType::LAND;
                commands[i].content.waypoint = CommandWaypoint(lat, lng, alt);
                break;
            case 'S': {
                int32_t num, pwm;
                if (!parseInt(stringPtr, num, 0) || !parseInt(stringPtr, pwm, 0)) {
                    fprintf(stderr, "[%s] Warning: Failed to parse values for 'set servo' command\n", ENTITY_NAME);
                    free(commands);
                    return 0;
                }
                commands[i].type = CommandType::SET_SERVO;
                commands[i].content.servo = CommandServo(num, pwm);
                calcCargoWaypoint(i - 1);
                break;
            }
            default:
                fprintf(stderr, "[%s] Warning: Cannot parse an unknown command %c\n", ENTITY_NAME, str[ptr]);
                free(commands);
                return 0;
        }
        ptr = end + 1;
    }

    positions = (Position *) malloc(posCount * sizeof(Position));
    //saveMissionsToPositions();

    hasMission = 1;
    return 1;
}

int parseMission(char *response) {
    if (strstr(response, "$-1#") != NULL) {
        fprintf(stderr, "[%s] Warning: No mission is available on the server\n", ENTITY_NAME);
        return 0;
    }

    char header[] = "$FlightMission ";
    char *start = strstr(response, header);
    if (start == NULL) {
        fprintf(stderr, "[%s] Warning: Response from the server does not contain mission\n", ENTITY_NAME);
        return 0;
    }
    start += strlen(header);

    return parseCommands(start);
}

void printMission(MissionCommand &cmd) {
    switch (cmd.type) {
        case CommandType::HOME:
            fprintf(stderr, "[%s] Info: Home: %d, %d, %d\n", ENTITY_NAME, cmd.content.waypoint.latitude,
                    cmd.content.waypoint.longitude, cmd.content.waypoint.altitude);
            break;
        case CommandType::TAKEOFF:
            fprintf(stderr, "[%s] Info: Takeoff: %d\n", ENTITY_NAME, cmd.content.takeoff.altitude);
            break;
        case CommandType::WAYPOINT:
            fprintf(stderr, "[%s] Info: Waypoint: %d, %d, %d\n", ENTITY_NAME, cmd.content.waypoint.latitude,
                    cmd.content.waypoint.longitude, cmd.content.waypoint.altitude);
            break;
        case CommandType::LAND:
            fprintf(stderr, "[%s] Info: Land: %d, %d, %d\n", ENTITY_NAME, cmd.content.waypoint.latitude,
                    cmd.content.waypoint.longitude, cmd.content.waypoint.altitude);
            break;
        case CommandType::SET_SERVO:
            fprintf(stderr, "[%s] Info: Set servo: %d, %d\n", ENTITY_NAME, cmd.content.servo.number,
                    cmd.content.servo.pwm);
            break;
        default:
            fprintf(stderr, "[%s] Warning: An unknown command\n", ENTITY_NAME);
            break;
    }
}

void printMissions() {
    if (!hasMission) {
        fprintf(stderr, "[%s] Info: No available mission\n", ENTITY_NAME);
        return;
    }
    fprintf(stderr, "[%s] Info: Mission: \n", ENTITY_NAME);
    for (int i = 0; i < commandNum; ++i)
        printMission(commands[i]);
}

void saveMissionsToPositions(int homeAltitude) {
    fprintf(stderr, "[%s] Info: Normalized positions %d: \n", ENTITY_NAME, posCount);

    Position lastHomePosition = {0, 0, 0};
    int currentPos = 0;

    for (int i = 0; i < commandNum; ++i) {
        MissionCommand cmd = commands[i];

        if (cmd.type != CommandType::HOME && cmd.type != CommandType::TAKEOFF && cmd.type != CommandType::WAYPOINT)
            continue;

        switch (cmd.type) {
            case CommandType::HOME:
                lastHomePosition = {
                        cmd.content.waypoint.latitude,
                        cmd.content.waypoint.longitude,
                        cmd.content.waypoint.altitude
                };
                positions[currentPos] = lastHomePosition;
                break;
            case CommandType::TAKEOFF:
                positions[currentPos] = {
                        positions[currentPos - 1].latitude,
                        positions[currentPos - 1].longitude,
                        homeAltitude + cmd.content.takeoff.altitude,
                        cmd.content.takeoff.altitude
                };
                break;
            case CommandType::WAYPOINT:
                positions[currentPos] = {
                        cmd.content.waypoint.latitude,
                        cmd.content.waypoint.longitude,
                        homeAltitude + cmd.content.waypoint.altitude,
                        cmd.content.waypoint.altitude
                };
                break;
        }

        printPosition(positions[currentPos]);
        currentPos++;
    }
    waypointCount = currentPos;
}


Position getCargoPosition() {
    return cargoPosition;
}

Position *getPositions() {
    return positions;
}

uint32_t getWaypointCount() {
    return waypointCount;
}