#include "../include/mission.h"
#include "../include/validations.h"
#include "../include/helpers.h"
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

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000
#define SPEED_SCAN_RATE_MS 300
#define PAUSE_MISSION_CHECK_RATE_MS 300
#define MAX_WAYPOINT_DIST 2


long double checkPauseLastTime = currentTime();
bool missionIsPaused = false;
bool needPauseMission();

int sendSignedMessage(char *method, char *response, char *errorMessage, uint8_t delay) {
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);

    while (!signMessage(message, signature)) {
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n",
                ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response)) {
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n",
                ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        fprintf(stderr,
                "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n",
                ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    return 1;
}

int sendFastSignedMessage(char *method, char *response) {
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);

    if (!signMessage(message, signature)) {
        return 0;
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    if (!sendRequest(request, response))
        return 0;

    uint8_t authenticity = 0;
    if (!checkSignature(response, authenticity) || !authenticity)
        return 0;

    return 1;
}

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

    //Copter need to be registered at ORVD
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

    //The drone is ready to arm
    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true) {
        //Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            fprintf(stderr,
                    "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n",
                    ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        //When autopilot asked for arm, we need to receive permission from ORVD
        char armRespone[1024] = {0};
        sendSignedMessage("/api/arm", armRespone, "arm", RETRY_DELAY_SEC);

        if (strstr(armRespone, "$Arm: 0#") != NULL) {
            //If arm was permitted, we enable motors
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

    //If we get here, the drone is able to arm and start the mission
    //The flight is need to be controlled from now on
    //Also we need to check on ORVD, whether the flight is still allowed or it is need to be paused

    DynamicPosition copter = {{0, 0, 0}, {0, 0, 0}, 0};
    copter.currentPosition = getCopterPosition(copter.currentPosition);

    copter.lastPosition = copter.lastPosition;
    long double dynamicLastUpdate = currentTime();
    Position cargoPosition = getCargoPosition();

    int waypointCount = getWaypointCount();
    Position *absolutePositions;
    absolutePositions = getPositions();
    int nextWaypointIdx = 1;
    double dist;

    fprintf(stderr, "[%s] DEBUG: Waypoints count : %d\n",
                ENTITY_NAME,
                waypointCount);

    while (true) {
        if (currentTime() - checkPauseLastTime > PAUSE_MISSION_CHECK_RATE_MS) {
            if (needPauseMission()) {
                if (!missionIsPaused) {
                    while(!pauseFlight())
                        sleep(1);
                    fprintf(stderr, "[%s] Info Mission is paused\n", ENTITY_NAME);
                }

                missionIsPaused = true;
                continue;
            } else if (missionIsPaused) {
                while(!resumeFlight())
                    sleep(1);
                fprintf(stderr, "[%s] Info Mission resumed\n", ENTITY_NAME);
                missionIsPaused = false;
            }
        }

        if (missionIsPaused)
            continue;

        //copter.currentPosition = getCopterPosition(copter.currentPosition);

        validateCargo(copter, cargoPosition);
        // validateSpeed(copter);

        // double dist = getDistance(copter.currentPosition, absolutePositions[nextWaypointIdx]);
        // if (dist < MAX_WAYPOINT_DIST && nextWaypointIdx < waypointCount) {
        //         nextWaypointIdx++;
        //         fprintf(stderr, "[%s] DEBUG: Next point is reached : %d\n",
        //         ENTITY_NAME,
        //         nextWaypointIdx);
        //     }

        // validatePosition(copter.currentPosition, absolutePositions[nextWaypointIdx - 1], absolutePositions[nextWaypointIdx]);
        if (currentTime() - dynamicLastUpdate > SPEED_SCAN_RATE_MS) {
            copter.lastPosition = copter.currentPosition;
            copter.lastTimeUpdatePosition = currentTime();
            dynamicLastUpdate = copter.lastTimeUpdatePosition;
        }
    }

    return EXIT_SUCCESS;
}

bool needPauseMission() {
    // Arm: 1 - disarm
    // Arm: 0 - arm

    char armRespone[1024] = {0};
    if (missionIsPaused) {
        sendSignedMessage("/api/fly_accept", armRespone, "fly_accept", RETRY_DELAY_SEC); 
    } else if (!sendFastSignedMessage("/api/fly_accept", armRespone)) { 
        checkPauseLastTime = currentTime();
        return 0;
    }

    checkPauseLastTime = currentTime();

    if (strstr(armRespone, "$Arm: 0#") != NULL) {
        return 0;
    } else if (strstr(armRespone, "$Arm: 1#") != NULL) {
        return 1;
    }

    fprintf(stderr, "[%s] Error: needPauseMission - unknowrn response\n", ENTITY_NAME);
    return 0;
}