#pragma once

/// Долгая отправка сообщений в ОРВД
int sendSignedMessage(char *method, char *response, char *errorMessage, unsigned char delay);

/// Быстрая отправка сообщений в ОРВД
int sendFastSignedMessage(char *method, char *response);