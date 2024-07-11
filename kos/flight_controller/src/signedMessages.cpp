#include "../include/signedMessages.h"

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