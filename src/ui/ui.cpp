// ui.cpp
//
// Головний UI модуль.
// Керує дисплеєм 2004 і обробляє енкодер.
//
// Два режими роботи:
//
//   UI_MODE_VIEW — головний екран, перегляд поточного стану.
//     Крутиш енкодер → переміщуєш курсор між параметрами.
//     Короткий натиск → нічого (захист від випадкового входу).
//     Довгий натиск → входиш в UI_MODE_EDIT.
//
//   UI_MODE_EDIT — редагування обраного параметра.
//     Крутиш енкодер → змінюєш значення.
//     Короткий натиск → вихід БЕЗ збереження.
//     Довгий натиск → зберегти і вийти.
//
// Макет екрану (4×20):
//
//   Рядок 0: SPD:300rpm  FWD>>>
//   Рядок 1: TURNS: 0000
//   Рядок 2: POS:  0.00mm
//   Рядок 3: DIA:0.20 W: 30.0mm
//
// Параметри для редагування:
//   0 — SPD  (швидкість, об/хв, крок 10)
//   1 — DIR  (напрямок >>> / <<<)
//   2 — DIA  (діаметр дроту, мм, крок 0.01)
//   3 — W    (ширина намотки, мм, крок 0.5)

#include "ui.h"
#include "encoder.h"
#include "../drivers/lcd.h"
#include "../include/machine_state.h"
#include "../include/config.h"
#include "../motion/motion.h"

#include <stdio.h>
#include <string.h>

// ─────────────────────────────────────────
// Режими UI
// ─────────────────────────────────────────

typedef enum {
    UI_MODE_VIEW,   // перегляд — курсор вибирає параметр
    UI_MODE_EDIT,   // редагування — крутиш змінюєш значення
} UiMode;

// ─────────────────────────────────────────
// Параметри які можна редагувати
// ─────────────────────────────────────────

typedef enum {
    PARAM_SPEED     = 0,
    PARAM_DIRECTION = 1,
    PARAM_DIAMETER  = 2,
    PARAM_WIDTH     = 3,
    PARAM_COUNT     = 4,
} ParamIndex;

// ─────────────────────────────────────────
// Внутрішній стан UI
// ─────────────────────────────────────────

static UiMode    uiMode       = UI_MODE_VIEW;
static uint8_t   selectedParam = PARAM_SPEED;  // поточний вибраний параметр

// Тимчасові значення під час редагування
// (зберігаємо в machineState тільки після підтвердження)
static uint32_t  editSpeed     = 0;
static float     editDiameter  = 0.0f;
static float     editWidth     = 0.0f;
static bool      editDirection = true;

// Прапорець що екран потребує повного перемалювання
static bool      needFullRedraw = true;

// ─────────────────────────────────────────
// Позиції курсора на екрані для кожного параметра
// ─────────────────────────────────────────

// {col, row} — де ставити курсор '>' для кожного параметра
static const uint8_t CURSOR_POS[PARAM_COUNT][2] = {
    {0, 0},   // PARAM_SPEED     — рядок 0, колонка 0
    {12, 0},  // PARAM_DIRECTION — рядок 0, колонка 12
    {0, 3},   // PARAM_DIAMETER  — рядок 3, колонка 0
    {9, 3},   // PARAM_WIDTH     — рядок 3, колонка 9
};

// ─────────────────────────────────────────
// Малювання екрану
// ─────────────────────────────────────────

// Намалювати весь головний екран
static void draw_screen()
{
    char buf[21]; // буфер рядка (20 символів + \0)

    // ── Рядок 0: SPD:300rpm  FWD>>> ─────────────────────────────
    lcd_set_cursor(0, 0);
    snprintf(buf, sizeof(buf), "SPD:%3lurpm         ",
             (unsigned long)machineState.spindleSpeed);
    // Обрізаємо до 20 символів і вставляємо напрямок
    buf[12] = machineState.directionForward ? 'F' : 'R';
    buf[13] = machineState.directionForward ? 'W' : 'E';
    buf[14] = machineState.directionForward ? 'D' : 'V';
    buf[15] = machineState.directionForward ? '>' : '<';
    buf[16] = machineState.directionForward ? '>' : '<';
    buf[17] = machineState.directionForward ? '>' : '<';
    buf[18] = ' ';
    buf[19] = ' ';
    buf[20] = '\0';
    lcd_print(buf);

    // ── Рядок 1: TURNS: 0000 ────────────────────────────────────
    lcd_set_cursor(0, 1);
    snprintf(buf, sizeof(buf), "TURNS: %04lu        ",
             (unsigned long)machineState.turns);
    buf[20] = '\0';
    lcd_print(buf);

    // ── Рядок 2: POS:  0.00mm ───────────────────────────────────
    lcd_set_cursor(0, 2);
    snprintf(buf, sizeof(buf), "POS: %6.2fmm      ",
             machineState.carriagePosition);
    buf[20] = '\0';
    lcd_print(buf);

    // ── Рядок 3: DIA:0.20 W: 30.0mm ────────────────────────────
    lcd_set_cursor(0, 3);
    snprintf(buf, sizeof(buf), "DIA:%.2f W:%5.1fmm",
             machineState.wireDiameter,
             machineState.windingWidth);
    buf[20] = '\0';
    lcd_print(buf);
}

// Намалювати курсор вибраного параметра
// Спочатку очищаємо всі позиції курсорів, потім ставимо активний
static void draw_cursor()
{
    // Очистити всі позиції курсорів (замінити на пробіл)
    for (int i = 0; i < PARAM_COUNT; i++)
    {
        lcd_set_cursor(CURSOR_POS[i][0], CURSOR_POS[i][1]);
        // В режимі VIEW — пробіл, в режимі EDIT — показуємо '*' на активному
        if (uiMode == UI_MODE_EDIT && i == selectedParam)
            lcd_write_char('*');  // '*' означає "зараз редагується"
        else if (i == selectedParam && uiMode == UI_MODE_VIEW)
            lcd_write_char('>');  // '>' означає "вибрано"
        else
            lcd_write_char(' ');
    }
}

// ─────────────────────────────────────────
// Логіка редагування параметрів
// ─────────────────────────────────────────

// Скопіювати поточні значення з machineState в тимчасові буфери
static void edit_begin()
{
    editSpeed     = machineState.spindleSpeed;
    editDiameter  = machineState.wireDiameter;
    editWidth     = machineState.windingWidth;
    editDirection = machineState.directionForward;
}

// Застосувати тимчасові значення до machineState
static void edit_confirm()
{
    machineState.directionForward = editDirection;
    motion_set_spindle_speed(editSpeed);
    motion_set_wire_diameter(editDiameter);
    machineState.windingWidth = editWidth;
}

// Змінити значення параметра на delta кроків
static void edit_apply_delta(int8_t delta)
{
    switch (selectedParam)
    {
        case PARAM_SPEED:
            // Крок 10 об/хв, діапазон 10–999
            if (delta > 0 && editSpeed < SPINDLE_SPEED_MAX) editSpeed += SPINDLE_SPEED_STEP;
            if (delta < 0 && editSpeed > SPINDLE_SPEED_MIN)  editSpeed -= SPINDLE_SPEED_STEP;
            break;

        case PARAM_DIRECTION:
            // Перемикання напрямку при будь-якому повороті
            if (delta != 0) editDirection = !editDirection;
            break;

        case PARAM_DIAMETER:
            // Крок 0.01 мм, діапазон 0.05–3.00
            if (delta > 0 && editDiameter < WIRE_DIAMETER_MAX) editDiameter += WIRE_DIAMETER_STEP;
            if (delta < 0 && editDiameter > WIRE_DIAMETER_MIN) editDiameter -= WIRE_DIAMETER_STEP;
            break;

        case PARAM_WIDTH:
            // Крок 0.5 мм, діапазон 1.0–200.0
            if (delta > 0 && editWidth < WINDING_WIDTH_MAX) editWidth += WINDING_WIDTH_STEP;
            if (delta < 0 && editWidth > WINDING_WIDTH_MIN)   editWidth -= WINDING_WIDTH_STEP;
            break;
    }
}

// Оновити тільки змінений рядок під час редагування
// Щоб не перемальовувати весь екран кожні 100 мс
static void edit_redraw_changed()
{
    char buf[21];

    switch (selectedParam)
    {
        case PARAM_SPEED:
            lcd_set_cursor(0, 0);
            snprintf(buf, sizeof(buf), " SPD:%3lurpm", (unsigned long)editSpeed);
            lcd_print(buf);
            break;

        case PARAM_DIRECTION:
            lcd_set_cursor(12, 0);
            if (editDirection)
                lcd_print("FWD>>>");
            else
                lcd_print("REV<<<");
            break;

        case PARAM_DIAMETER:
            lcd_set_cursor(0, 3);
            snprintf(buf, sizeof(buf), " DIA:%.2f", editDiameter);
            lcd_print(buf);
            break;

        case PARAM_WIDTH:
            lcd_set_cursor(9, 3);
            snprintf(buf, sizeof(buf), " W:%5.1fmm", editWidth);
            lcd_print(buf);
            break;
    }
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void ui_init()
{
    lcd_init();
    encoder_init();
    lcd_backlight(true);
    needFullRedraw = true;
}

void ui_update()
{
    // Опитати стан енкодера і кнопки
    encoder_update();

    int8_t delta      = encoder_get_delta();
    bool   click      = encoder_get_click();
    bool   longPress  = encoder_get_long_press();

    // ── Режим перегляду ─────────────────────────────────────────
    if (uiMode == UI_MODE_VIEW)
    {
        // Крутіння — переміщення курсора між параметрами
        if (delta != 0)
        {
            if (delta > 0)
                selectedParam = (selectedParam + 1) % PARAM_COUNT;
            else
                selectedParam = (selectedParam + PARAM_COUNT - 1) % PARAM_COUNT;

            draw_cursor();
        }

        // Довгий натиск — увійти в редагування
        if (longPress)
        {
            edit_begin();
            uiMode = UI_MODE_EDIT;
            draw_cursor(); // показати '*' замість '>'
        }

        // Короткий натиск в VIEW — ігноруємо
        (void)click;

        // Оновлюємо динамічні поля (витки, позиція)
        if (needFullRedraw)
        {
            draw_screen();
            draw_cursor();
            needFullRedraw = false;
        }
        else
        {
            // Оновити тільки рядки що змінюються під час роботи
            char buf[21];

            lcd_set_cursor(0, 1);
            snprintf(buf, sizeof(buf), "TURNS: %04lu        ",
                     (unsigned long)machineState.turns);
            buf[20] = '\0';
            lcd_print(buf);

            lcd_set_cursor(0, 2);
            snprintf(buf, sizeof(buf), "POS: %6.2fmm      ",
                     machineState.carriagePosition);
            buf[20] = '\0';
            lcd_print(buf);
        }
    }

    // ── Режим редагування ───────────────────────────────────────
    else if (uiMode == UI_MODE_EDIT)
    {
        // Крутіння — змінюємо значення параметра
        if (delta != 0)
        {
            edit_apply_delta(delta);
            edit_redraw_changed();
        }

        // Короткий натиск — вийти БЕЗ збереження
        if (click)
        {
            uiMode = UI_MODE_VIEW;
            needFullRedraw = true; // відновити екран з machineState
        }

        // Довгий натиск — зберегти і вийти
        if (longPress)
        {
            edit_confirm();
            uiMode = UI_MODE_VIEW;
            needFullRedraw = true;
        }
    }
}