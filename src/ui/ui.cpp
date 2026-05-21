// ui.cpp
//
// Головний UI модуль.
//
// Макет екрану (4×20), позиції фіксовані:
//
//   Col: 01234567890123456789
//   Row 0: >SPD:300rpm >FWD>>>
//   Row 1: TURNS: 0000
//   Row 2: POS:   0.00mm
//   Row 3: >DIA:0.20 >W: 30.0
//
// Курсори: '>' в col 0 (SPEED, DIAMETER) і col 11 (DIRECTION, WIDTH)
// Рядки 1 і 2 — тільки динамічні дані, без курсора.

#include "ui.h"
#include "encoder.h"
#include "../drivers/lcd.h"
#include "../include/machine_state.h"
#include "../include/config.h"
#include "../motion/motion.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// ─────────────────────────────────────────
// Режими UI
// ─────────────────────────────────────────

typedef enum {
    UI_MODE_VIEW,
    UI_MODE_EDIT,
} UiMode;

typedef enum {
    PARAM_SPEED     = 0,
    PARAM_DIRECTION = 1,
    PARAM_DIAMETER  = 2,
    PARAM_WIDTH     = 3,
    PARAM_COUNT     = 4,
} ParamIndex;

// ─────────────────────────────────────────
// Стан UI
// ─────────────────────────────────────────

static UiMode  uiMode         = UI_MODE_VIEW;
static uint8_t selectedParam  = PARAM_SPEED;
static bool    needFullRedraw  = true;

// Тимчасові значення під час редагування
static uint32_t editSpeed     = 0;
static float    editDiameter  = 0.0f;
static float    editWidth     = 0.0f;
static bool     editDirection = true;

// Кеш відображених значень
static uint32_t lastTurns    = UINT32_MAX;
static float    lastPosition = -9999.0f;

// ─────────────────────────────────────────
// Позиції курсора {col, row}
//
// Row 0: курсор SPEED=col0, DIRECTION=col11
// Row 3: курсор DIAMETER=col0, WIDTH=col11
// ─────────────────────────────────────────

static const uint8_t CURSOR_POS[PARAM_COUNT][2] = {
    {0,  0},   // PARAM_SPEED
    {11, 0},   // PARAM_DIRECTION
    {0,  3},   // PARAM_DIAMETER
    {11, 3},   // PARAM_WIDTH
};

// ─────────────────────────────────────────
// Малювання полів
// Кожне поле: курсор в col X, текст починається з col X+1
// ─────────────────────────────────────────

// Row 0, col 1-10: "SPD:300rpm"
static void draw_speed(uint32_t speed)
{
    char buf[11];
    snprintf(buf, sizeof(buf), "SPD:%3lurpm", (unsigned long)speed);
    lcd_set_cursor(1, 0);
    lcd_print(buf);
}

// Row 0, col 12-19: "FWD>>>" або "REV<<<"
static void draw_direction(bool forward)
{
    lcd_set_cursor(12, 0);
    lcd_print(forward ? "FWD>>>" : "REV<<<");
}

// Row 1, col 0-19: "TURNS: 0000"
static void draw_turns(uint32_t turns)
{
    char buf[21];
    snprintf(buf, sizeof(buf), "TURNS: %04lu         ", (unsigned long)turns);
    buf[20] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(buf);
    lastTurns = turns;
}

// Row 2, col 0-19: "POS:   0.00mm"
static void draw_position(float pos)
{
    char buf[21];
    snprintf(buf, sizeof(buf), "POS:  %6.2fmm      ", pos);
    buf[20] = '\0';
    lcd_set_cursor(0, 2);
    lcd_print(buf);
    lastPosition = pos;
}

// Row 3, col 1-9: "DIA:0.20"
static void draw_diameter(float dia)
{
    char buf[10];
    snprintf(buf, sizeof(buf), "DIA:%.2f ", dia);
    lcd_set_cursor(1, 3);
    lcd_print(buf);
}

// Row 3, col 12-19: "W: 30.0"
static void draw_width(float width)
{
    char buf[9];
    snprintf(buf, sizeof(buf), "W:%5.1f", width);
    lcd_set_cursor(12, 3);
    lcd_print(buf);
}

// Намалювати курсор.
// prevParam — попередній параметр (його курсор очищаємо).
static void draw_cursor(uint8_t prevParam)
{
    // Очистити попередній
    lcd_set_cursor(CURSOR_POS[prevParam][0], CURSOR_POS[prevParam][1]);
    lcd_write_char(' ');

    // Намалювати новий
    lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
    lcd_write_char(uiMode == UI_MODE_EDIT ? '*' : '>');
}

// Повне перемалювання
static void draw_full_screen()
{
    lcd_clear();

    // Скидаємо кеш
    lastTurns    = UINT32_MAX;
    lastPosition = -9999.0f;

    // Малюємо всі поля
    draw_speed(machineState.spindleSpeed);
    draw_direction(machineState.directionForward);
    draw_turns(machineState.turns);
    draw_position(machineState.carriagePosition);
    draw_diameter(machineState.wireDiameter);
    draw_width(machineState.windingWidth);

    // Курсор поточного параметра
    lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
    lcd_write_char('>');
}

// ─────────────────────────────────────────
// Редагування
// ─────────────────────────────────────────

static void edit_begin()
{
    editSpeed     = machineState.spindleSpeed;
    editDiameter  = machineState.wireDiameter;
    editWidth     = machineState.windingWidth;
    editDirection = machineState.directionForward;
}

static void edit_confirm()
{
    motion_set_spindle_speed(editSpeed);
    motion_set_wire_diameter(editDiameter);
    machineState.windingWidth     = editWidth;
    machineState.directionForward = editDirection;
}

static void edit_apply_delta(int8_t delta)
{
    switch (selectedParam)
    {
        case PARAM_SPEED:
            if (delta > 0 && editSpeed < SPINDLE_SPEED_MAX) editSpeed += SPINDLE_SPEED_STEP;
            if (delta < 0 && editSpeed > SPINDLE_SPEED_MIN) editSpeed -= SPINDLE_SPEED_STEP;
            draw_speed(editSpeed);
            break;

        case PARAM_DIRECTION:
            if (delta != 0) editDirection = !editDirection;
            draw_direction(editDirection);
            break;

        case PARAM_DIAMETER:
            if (delta > 0 && editDiameter < WIRE_DIAMETER_MAX) editDiameter += WIRE_DIAMETER_STEP;
            if (delta < 0 && editDiameter > WIRE_DIAMETER_MIN) editDiameter -= WIRE_DIAMETER_STEP;
            draw_diameter(editDiameter);
            break;

        case PARAM_WIDTH:
            if (delta > 0 && editWidth < WINDING_WIDTH_MAX) editWidth += WINDING_WIDTH_STEP;
            if (delta < 0 && editWidth > WINDING_WIDTH_MIN) editWidth -= WINDING_WIDTH_STEP;
            draw_width(editWidth);
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
    encoder_update();

    int8_t delta     = encoder_get_delta();
    bool   click     = encoder_get_click();
    bool   longPress = encoder_get_long_press();

    // Повне перемалювання
    if (needFullRedraw)
    {
        draw_full_screen();
        needFullRedraw = false;
        return;
    }

    // ── VIEW ────────────────────────────────────────────────────
    if (uiMode == UI_MODE_VIEW)
    {
        if (delta != 0)
        {
            uint8_t prev = selectedParam;
            if (delta > 0)
                selectedParam = (selectedParam + 1) % PARAM_COUNT;
            else
                selectedParam = (selectedParam + PARAM_COUNT - 1) % PARAM_COUNT;
            draw_cursor(prev);
        }

        if (longPress)
        {
            edit_begin();
            uiMode = UI_MODE_EDIT;
            lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
            lcd_write_char('*');
        }

        // Оновити динамічні поля тільки якщо змінились
        if (machineState.turns != lastTurns)
            draw_turns(machineState.turns);

        if (fabsf(machineState.carriagePosition - lastPosition) > 0.005f)
            draw_position(machineState.carriagePosition);

        (void)click;
    }

    // ── EDIT ────────────────────────────────────────────────────
    else if (uiMode == UI_MODE_EDIT)
    {
        if (delta != 0)
            edit_apply_delta(delta);

        // Короткий натиск — скасувати
        if (click)
        {
            uiMode = UI_MODE_VIEW;
            needFullRedraw = true;
        }

        // Довгий натиск — зберегти
        if (longPress)
        {
            edit_confirm();
            uiMode = UI_MODE_VIEW;
            needFullRedraw = true;
        }
    }
}