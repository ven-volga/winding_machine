#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motion/motion.h"
#include "ui/ui.h"
#include "logic/machine.h"

void app_main(void)
{
    // Init subsystems
    motion_init();
    ui_init();
    machine_init();

    // Main logic task
    while(true)
    {
        machine_update();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}