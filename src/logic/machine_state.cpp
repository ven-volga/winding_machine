// machine_state.cpp
//
// Тут живе єдиний глобальний екземпляр стану машини.
// Всі інші модулі підключають machine_state.h і отримують
// доступ через extern MachineState machineState.

#include "machine_state.h"

// Єдине визначення об'єкту.
// Початкові значення задані прямо в структурі (machine_state.h).
MachineState machineState;