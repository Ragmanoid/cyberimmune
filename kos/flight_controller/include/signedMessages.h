#pragma once

/// Долгая отправка сообщений в ОРВД
int sendSignedMessage(char *method, char *response, char *errorMessage, unsigned char delay);

/// Долгая отправка сообщений в ОРВД без ответа
int sendSignedMessage(char *message, char *errorMessage, unsigned char delay);

/// Быстрая отправка сообщений в ОРВД
int sendFastSignedMessage(char *method, char *response);