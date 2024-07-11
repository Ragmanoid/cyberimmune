#pragma once

/// Долгая отправка сообщений в ОРВД
int sendSignedMessage(char *method, char *response, char *errorMessage, uint8_t delay);

/// Быстрая отправка сообщений в ОРВД
int sendFastSignedMessage(char *method, char *response);