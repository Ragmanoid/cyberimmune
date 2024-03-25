#include "../include/periphery_controller.h"
#include "../include/gpio.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <gpio/gpio.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryController.edl.h>

#define NAME_MAX_LENGTH 64
#define RETRY_DELAY_SEC 1

char gpio[] = "gpio0";
char gpioConfigSuffix[] = "default";

uint8_t pinBuzzer = 20;
uint8_t pinCargoLock = 21;
uint8_t pinKillSwitchFirst = 22;
uint8_t pinKillSwitchSecond = 27;

int initPeripheryController() {
    char boardName[NAME_MAX_LENGTH] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to get board name\n", ENTITY_NAME);
        return 0;
    }

    char gpioConfig[NAME_MAX_LENGTH];
    if (snprintf(gpioConfig, NAME_MAX_LENGTH, "%s.%s", boardName, gpioConfigSuffix) < 0) {
        fprintf(stderr, "[%s] Error: Failed to generate GPIO config name\n", ENTITY_NAME);
        return 0;
    }

    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize BSP ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = BspSetConfig(gpio, gpioConfig);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to set BSP config for GPIO %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpio, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = GpioInit();
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize GPIO ("RETCODE_HR_FMT")\n", ENTITY_NAME, RC_GET_CODE(rc));
        return 0;
    }

    return 1;
}

int initGpioPins() {
    if (!startGpio(gpio))
        return 0;

    uint8_t pins[] = { pinBuzzer, pinCargoLock, pinKillSwitchFirst, pinKillSwitchSecond };
    for (uint8_t pin : pins)
        if (!initPin(pin))
            return 0;

    return 1;
}

int setBuzzer(bool enable) {
    return setPin(pinBuzzer, enable);
}

int setKillSwitch(bool enable) {
    if (enable) {
        if (!setPin(pinKillSwitchFirst, false))
            return 0;
        return setPin(pinKillSwitchSecond, true);
    }
    else {
        if (!setPin(pinKillSwitchFirst, false))
            return 0;
        return setPin(pinKillSwitchSecond, false);
    }
}

int setCargoLock(bool enable) {
    return setPin(pinCargoLock, enable);
}