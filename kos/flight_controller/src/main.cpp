#include "../include/mission.h"
#include "../include/validations.h"
#include "../include/helpers.h"
#include "../include/signedMessages.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

#define SPEED_SCAN_RATE_MS 900 // Регулярность контроля скорости
#define PAUSE_MISSION_CHECK_RATE_MS 300 // Регулярность опроса ОРВД о паузе миссии
#define MAX_WAYPOINT_DIST 5 // Максимальное расстояние до точки, м

extern long double checkPauseLastTime;

int main(void) {
    //Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        fprintf(stderr,
                "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n",
                ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        fprintf(stderr,
                "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n",
                ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        fprintf(stderr,
                "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n",
                ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        fprintf(stderr,
                "[%s] Warning: Failed to receive initialization notification from Server Connector. Trying again in %ds\n",
                ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        fprintf(stderr,
                "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n",
                ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    //Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        fprintf(stderr, "[%s] Warning: Failed to enable buzzer at Periphery Controller\n", ENTITY_NAME);

    // Регистрация дрона в ОРВД
    char authResponse[1024] = {0};
    sendSignedMessage("/api/auth", authResponse, "authentication", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    //Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (true) {
        char missionResponse[1024] = {0};
        if (sendSignedMessage("/api/fmission_kos", missionResponse, "mission", RETRY_DELAY_SEC) &&
            parseMission(missionResponse)) {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMissions();
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    // The drone is ready to arm
    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true) {
        // Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            fprintf(stderr,
                    "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n",
                    ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        // When autopilot asked for arm, we need to receive permission from ORVD
        char armRespone[1024] = {0};
        sendSignedMessage("/api/arm", armRespone, "arm", RETRY_DELAY_SEC);

        if (strstr(armRespone, "$Arm: 0#") != NULL) {
            // If arm was permitted, we enable motors
            fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            while (!setKillSwitch(true)) {
                fprintf(stderr,
                        "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n",
                        ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
            break;
        } else if (strstr(armRespone, "$Arm: 1#") != NULL) {
            fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if (!forbidArm())
                fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
        } else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n",
                ENTITY_NAME);
    }

    // If we get here, the drone is able to arm and start the mission
    // The flight is need to be controlled from now on
    // Also we need to check on ORVD, whether the flight is still allowed or it is need to be paused

    // Обработка базовых координат взлёта
    DynamicPosition copter = {{0, 0, 0}, {0, 0, 0}, 0};
    copter.currentPosition = getCopterPosition(copter.currentPosition);
    int32_t homeAltitude = copter.currentPosition.altitude;
    saveMissionsToPositions(homeAltitude);
    addHomeAltitudeToCargo(homeAltitude);

    // Получим первичные значения положения
    bool missionIsPaused = false;
    long double dynamicLastUpdate = currentTime();
    Position cargoPosition = getCargoPosition();

    int waypointCount = getWaypointCount();
    Position *absolutePositions;
    absolutePositions = getPositions();
    int nextWaypointIdx = 1;
    double dist;
    bool speedIsSetted = false;

    fprintf(stderr, "[%s] DEBUG: Waypoints count : %d\n",
                ENTITY_NAME,
                waypointCount);

    while (true) {
        // Проверка на остановку миссии
        if (currentTime() - checkPauseLastTime > PAUSE_MISSION_CHECK_RATE_MS) {
            if (needPauseMission(missionIsPaused)) {
                if (!missionIsPaused) {
                    while(!pauseFlight())
                        sleep(1);
                    fprintf(stderr, "[%s] Info Mission is paused\n", ENTITY_NAME);
                    sendLogs("Mission_paused");
                }

                missionIsPaused = true;
                continue;
            } else if (missionIsPaused) {
                while(!resumeFlight())
                    sleep(1);
                fprintf(stderr, "[%s] Info Mission resumed\n", ENTITY_NAME);
                sendLogs("Mission_resumed");
                missionIsPaused = false;
            }
        }

        if (missionIsPaused)
            continue;

        copter.currentPosition = getCopterPosition(copter.currentPosition);

        // Определение точки, которую дрон достиг
        for (int wpIdx = nextWaypointIdx; wpIdx < waypointCount - 1; ++wpIdx) {
            double dist = getDistance(copter.currentPosition, absolutePositions[wpIdx]);
            if (dist < MAX_WAYPOINT_DIST && wpIdx < waypointCount) {
                    nextWaypointIdx = wpIdx + 1;
                    char msg[100] = {0}; 
                    snprintf(msg, 100, "Next_point_is_reached:%d", wpIdx);
                    fprintf(stderr, "[%s] DEBUG: %s\n", ENTITY_NAME, msg);
                    sendLogs(msg);
                    break;
                }
        }

        if (nextWaypointIdx > 2 && !speedIsSetted) {
            if (changeSpeed(2))
                speedIsSetted = true;
        }

        // Валидация позиции в высоту
        if (nextWaypointIdx > 3) 
            if(!validateAltitude(copter.currentPosition, absolutePositions[nextWaypointIdx]))
                return EXIT_FAILURE;

        // Валидация позиции в ширину
        // if(!validatePosition(copter.currentPosition, absolutePositions[nextWaypointIdx - 1], absolutePositions[nextWaypointIdx]))
        //     return EXIT_FAILURE;

        // Валидация выполнения полетного задания
        validateDirection(copter);


        // Валидация скорости
        if (currentTime() - dynamicLastUpdate > SPEED_SCAN_RATE_MS) {
            validateSpeed(copter);
            copter.lastPosition = copter.currentPosition;
            copter.lastTimeUpdatePosition = currentTime();
            dynamicLastUpdate = copter.lastTimeUpdatePosition;
        }

        // Валидация сброса груза
        validateCargo(copter, cargoPosition);
    }

    return EXIT_SUCCESS;
}