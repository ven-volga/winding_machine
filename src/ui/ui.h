#pragma once

// Ініціалізація UI (LCD + енкодер)
void ui_init();

// Головний цикл UI — викликати кожні ~100 мс з ui_task (ядро 1)
void ui_update();