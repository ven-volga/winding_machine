// ui.cpp
//
// Макет екрану (4×20):
//
//   Col: 01234567890123456789
//   Row 0: >SPD:300rpm  >FWD>>>
//   Row 1: >TURNS:9999/0000
//   Row 2: >POS:  0.00mm AR:+
//   Row 3: >DIA:0.20  >W: 30.0
//
// Row 0: SPD (col 0), FWD/REV (col 12, курсор col 11)  ← зсунуто на 1
// Row 1: TURNS задані / намотані (курсор col 0)
// Row 2: POS + AR:+/- або >>> / <<< в сервісі
// Row 3: DIA (col 0), W (col 11, курсор col 10)        ← зсунуто на 1
//
// TURNS редагування:
//   Повільно (>200мс між кроками) → крок 1
//   Середньо (>100мс)             → крок 10
//   Швидко   (<100мс)             → крок 100
//
// Підменю POS:
//   Крутиш         → обираєш пункт
//   Довгий натиск  → входиш в редагування
//   Короткий натиск → вихід в головне меню
//   В редагуванні:
//   Крутиш          → змінюєш значення
//   Довгий натиск   → зберегти і вийти
//   Короткий натиск → скасувати

#include "ui.h"
#include "encoder.h"
#include "lcd.h"
#include "shared_state.h"
#include "config.h"
#include "motion.h"

#include <cstdio>
#include <cmath>

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_timer.h"
}

// ─────────────────────────────────────────
// Режими
// ─────────────────────────────────────────

typedef enum
{
    UI_VIEW,
    UI_EDIT,
    UI_SUBMENU_POS,
    UI_SUBMENU_EDIT,
} UiMode;

typedef enum
{
    PARAM_SPEED     = 0,
    PARAM_DIRECTION = 1,
    PARAM_TURNS     = 2,
    PARAM_POS       = 3,
    PARAM_DIAMETER  = 4,
    PARAM_WIDTH     = 5,
    PARAM_COUNT     = 6,
} Param;

typedef enum
{
    SUBMENU_SET_VALUE = 0,
    SUBMENU_SET_HOME  = 1,
    SUBMENU_AUTO_REV  = 2,
    SUBMENU_COUNT     = 3,
} PosSubmenu;

// ─────────────────────────────────────────
// Стан UI
// ─────────────────────────────────────────

static UiMode  uiMode         = UI_VIEW;
static uint8_t selectedParam  = PARAM_SPEED;
static bool    needFullRedraw  = true;

// Буфер редагування
static uint32_t editSpeed      = 0;
static float    editDia        = 0.0f;
static float    editWidth      = 0.0f;
static bool     editDir        = true;
static uint32_t editTargetTurns = 0;

// Підменю POS
static uint8_t  submenuItem  = SUBMENU_SET_VALUE;
static float    editPos      = 0.0f;
static bool     editAutoRev  = true;

// Кеш
static uint32_t lastTurns        = 0xFFFFFFFF;
static uint32_t lastTargetTurns  = 0xFFFFFFFF;
static float    lastPos          = -9999.0f;
static uint32_t lastSpeed        = 0xFFFFFFFF;
static bool     lastDir          = true;
static bool     lastSvcActive    = false;
static bool     lastSvcDir       = true;
static bool     lastAutoRev      = true;

// Для прискорення TURNS
static uint32_t lastTurnsEditMs  = 0;

// Позиції курсорів {col, row}
static const uint8_t CURSOR_POS[PARAM_COUNT][2] =
{
    {0,  0},   // SPEED
    {12, 0},   // DIRECTION (курсор col 12, текст col 13)
    {0,  1},   // TURNS
    {0,  2},   // POS
    {0,  3},   // DIAMETER
    {11, 3},   // WIDTH (курсор col 11, текст col 12)
};

// ─────────────────────────────────────────
// Таймер енкодера (1мс)
// ─────────────────────────────────────────

static esp_timer_handle_t encoderTimer;

static void IRAM_ATTR encoder_timer_cb(void* arg)
{
    encoder_update();
    if (encDelta == 0) return;

    shared_lock();
    bool running = shared.running;
    shared_unlock();
    if (!running) return;

    shared_lock();
    uint32_t spd = shared.spindleSpeed;
    if (encDelta > 0 && spd < SPINDLE_SPEED_MAX) spd += SPINDLE_SPEED_STEP;
    if (encDelta < 0 && spd > SPINDLE_SPEED_MIN) spd -= SPINDLE_SPEED_STEP;
    shared.spindleSpeed = spd;
    shared_unlock();

    encDelta = 0;
}

// ─────────────────────────────────────────
// Малювання
// ─────────────────────────────────────────

static void draw_speed(uint32_t spd)
{
    char buf[11];
    snprintf(buf, sizeof(buf), "SPD:%3lurpm", (unsigned long)spd);
    lcd_set_cursor(1, 0);
    lcd_print(buf);
    lastSpeed = spd;
}

// FWD/REV (курсор col 12, текст col 13)
static void draw_direction(bool fwd)
{
    lcd_set_cursor(13, 0);
    lcd_print(fwd ? "FWD>>>" : "REV<<<");
    lastDir = fwd;
}

// TURNS: задані / намотані
static void draw_turns(uint32_t turns, uint32_t target)
{
    char buf[21];
    if (target == 0)
    {
        uint32_t t = turns > 9999 ? 9999 : turns;
        snprintf(buf, sizeof(buf), " TURNS:----%04u    ", (unsigned)t);
    }
    else
    {
        uint32_t t = turns  > 9999 ? 9999 : turns;
        uint32_t g = target > 9999 ? 9999 : target;
        snprintf(buf, sizeof(buf), " TURNS:%04u/%04u   ", (unsigned)g, (unsigned)t);
    }
    buf[20] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(buf);
    lastTurns       = turns;
    lastTargetTurns = target;
}

// Row 2: POS + AR:+/- або >>> / <<< в сервісі
static void draw_position_row(float pos, bool svcActive, bool svcDir, bool autoRev)
{
    char info[5];
    if (svcActive)
        snprintf(info, sizeof(info), svcDir ? ">>> " : "<<< ");
    else
        snprintf(info, sizeof(info), autoRev ? "AR:+" : "AR:-");

    char buf[21];
    snprintf(buf, sizeof(buf), " POS:%6.2fmm %s", pos, info);
    buf[20] = '\0';
    lcd_set_cursor(0, 2);
    lcd_print(buf);
    lastPos       = pos;
    lastSvcActive = svcActive;
    lastSvcDir    = svcDir;
    lastAutoRev   = autoRev;
}

static void draw_diameter(float dia)
{
    char buf[10];
    snprintf(buf, sizeof(buf), "DIA:%.2f ", dia);
    lcd_set_cursor(1, 3);
    lcd_print(buf);
}

// W (курсор col 11, текст col 12)
static void draw_width(float w)
{
    char buf[9];
    snprintf(buf, sizeof(buf), "W:%5.1f", w);
    lcd_set_cursor(12, 3);
    lcd_print(buf);
}

static void draw_cursor(uint8_t prev)
{
    lcd_set_cursor(CURSOR_POS[prev][0], CURSOR_POS[prev][1]);
    lcd_print(" ");
    lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
    lcd_print(uiMode == UI_EDIT ? "*" : ">");
}

static void draw_full_screen(const SharedState& s)
{
    lcd_clear();
    lastTurns       = 0xFFFFFFFF;
    lastTargetTurns = 0xFFFFFFFF;
    lastPos         = -9999.0f;
    lastSpeed       = 0xFFFFFFFF;
    lastDir         = !s.dirForward;
    lastSvcActive   = false;
    lastAutoRev     = !s.autoReverse;

    draw_speed(s.spindleSpeed);
    draw_direction(s.dirForward);
    draw_turns(s.turns, s.targetTurns);
    draw_position_row(s.carriagePos, s.serviceCarriageActive,
                      s.serviceCarriageDir, s.autoReverse);
    draw_diameter(s.wireDiameter);
    draw_width(s.windingWidth);

    lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
    lcd_print(">");
}

// ─────────────────────────────────────────
// Підменю POS
// ─────────────────────────────────────────

static void draw_submenu_pos(const SharedState& s, bool editMode)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("-- POS SETTINGS --");

    char buf[21];

    // Set value
    snprintf(buf, sizeof(buf), "%cSet val: %6.1fmm",
             (submenuItem == SUBMENU_SET_VALUE) ? (editMode ? '*' : '>') : ' ',
             editPos);
    buf[20] = '\0';
    lcd_set_cursor(0, 1); lcd_print(buf);

    // Set home
    snprintf(buf, sizeof(buf), "%cSet home (0.00mm)",
             (submenuItem == SUBMENU_SET_HOME) ? (editMode ? '*' : '>') : ' ');
    buf[20] = '\0';
    lcd_set_cursor(0, 2); lcd_print(buf);

    // Auto rev
    snprintf(buf, sizeof(buf), "%cAuto rev:     %s",
             (submenuItem == SUBMENU_AUTO_REV) ? (editMode ? '*' : '>') : ' ',
             editAutoRev ? "ON " : "OFF");
    buf[20] = '\0';
    lcd_set_cursor(0, 3); lcd_print(buf);
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void ui_init()
{
    lcd_init();
    encoder_init();

    esp_timer_create_args_t timerArgs = {
        .callback              = encoder_timer_cb,
        .arg                   = nullptr,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                  = "enc_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&timerArgs, &encoderTimer);
    esp_timer_start_periodic(encoderTimer, 1000);

    needFullRedraw = true;
}

void ui_update()
{
    SharedState s;
    shared_lock();
    s = shared;
    shared_unlock();

    if (needFullRedraw)
    {
        if (uiMode == UI_SUBMENU_POS || uiMode == UI_SUBMENU_EDIT)
            draw_submenu_pos(s, uiMode == UI_SUBMENU_EDIT);
        else
            draw_full_screen(s);
        needFullRedraw = false;
        return;
    }

    // ════════════════════════════════════════════════════════════
    // VIEW
    // ════════════════════════════════════════════════════════════
    if (uiMode == UI_VIEW)
    {
        if (encDelta != 0 && !s.running)
        {
            uint8_t prev = selectedParam;
            if (encDelta > 0)
                selectedParam = (selectedParam + 1) % PARAM_COUNT;
            else
                selectedParam = (selectedParam + PARAM_COUNT - 1) % PARAM_COUNT;
            draw_cursor(prev);
            encDelta = 0;
        }

        if (evLong)
        {
            if (selectedParam == PARAM_POS)
            {
                shared_lock();
                editPos     = shared.carriagePos;
                editAutoRev = shared.autoReverse;
                shared_unlock();
                submenuItem    = SUBMENU_SET_VALUE;
                uiMode         = UI_SUBMENU_POS;
                needFullRedraw = true;
            }
            else
            {
                editSpeed       = s.spindleSpeed;
                editDia         = s.wireDiameter;
                editWidth       = s.windingWidth;
                editDir         = s.dirForward;
                editTargetTurns = s.targetTurns;
                uiMode = UI_EDIT;
                lcd_set_cursor(CURSOR_POS[selectedParam][0],
                               CURSOR_POS[selectedParam][1]);
                lcd_print("*");
            }
        }

        // Динамічні поля
        if (s.spindleSpeed != lastSpeed)
            draw_speed(s.spindleSpeed);
        if (s.dirForward != lastDir)
            draw_direction(s.dirForward);
        if (s.turns != lastTurns || s.targetTurns != lastTargetTurns)
            draw_turns(s.turns, s.targetTurns);
        if (fabsf(s.carriagePos - lastPos) > 0.005f    ||
            s.serviceCarriageActive != lastSvcActive    ||
            s.serviceCarriageDir    != lastSvcDir       ||
            s.autoReverse           != lastAutoRev)
        {
            draw_position_row(s.carriagePos, s.serviceCarriageActive,
                              s.serviceCarriageDir, s.autoReverse);
        }
    }

    // ════════════════════════════════════════════════════════════
    // EDIT
    // ════════════════════════════════════════════════════════════
    else if (uiMode == UI_EDIT)
    {
        if (encDelta != 0)
        {
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

            switch (selectedParam)
            {
                case PARAM_SPEED:
                    if (encDelta > 0 && editSpeed < SPINDLE_SPEED_MAX)
                        editSpeed += SPINDLE_SPEED_STEP;
                    if (encDelta < 0 && editSpeed > SPINDLE_SPEED_MIN)
                        editSpeed -= SPINDLE_SPEED_STEP;
                    draw_speed(editSpeed);
                    break;

                case PARAM_DIRECTION:
                    editDir = !editDir;
                    draw_direction(editDir);
                    break;

                case PARAM_TURNS:
                {
                    // Прискорення залежно від швидкості кручення
                    uint32_t elapsed = now - lastTurnsEditMs;
                    uint32_t step = 1;
                    if      (elapsed < 50)  step = 100;
                    else if (elapsed < 100) step = 10;
                    lastTurnsEditMs = now;

                    if (encDelta > 0)
                        editTargetTurns = (editTargetTurns + step > 9999) ?
                                           9999 : editTargetTurns + step;
                    else
                        editTargetTurns = (editTargetTurns < step) ?
                                           0 : editTargetTurns - step;

                    // Оновлюємо тільки поле TURNS
                    char buf[21];
                    if (editTargetTurns == 0)
                    {
                        uint32_t st = s.turns > 9999 ? 9999 : s.turns;
                        snprintf(buf, sizeof(buf), "*TURNS:----%04u    ", (unsigned)st);
                    }
                    else
                    {
                        uint32_t et = editTargetTurns > 9999 ? 9999 : editTargetTurns;
                        uint32_t st = s.turns > 9999 ? 9999 : s.turns;
                        snprintf(buf, sizeof(buf), "*TURNS:%04u/%04u   ",
                                 (unsigned)et, (unsigned)st);
                    }
                    buf[20] = '\0';
                    lcd_set_cursor(0, 1);
                    lcd_print(buf);
                    break;
                }

                case PARAM_DIAMETER:
                    if (encDelta > 0 && editDia < WIRE_DIAMETER_MAX)
                        editDia += WIRE_DIAMETER_STEP;
                    if (encDelta < 0 && editDia > WIRE_DIAMETER_MIN)
                        editDia -= WIRE_DIAMETER_STEP;
                    draw_diameter(editDia);
                    break;

                case PARAM_WIDTH:
                    if (encDelta > 0 && editWidth < WINDING_WIDTH_MAX)
                        editWidth += WINDING_WIDTH_STEP;
                    if (encDelta < 0 && editWidth > WINDING_WIDTH_MIN)
                        editWidth -= WINDING_WIDTH_STEP;
                    draw_width(editWidth);
                    break;
            }
            encDelta = 0;
        }

        if (evClick)
        {
            uiMode = UI_VIEW;
            needFullRedraw = true;
        }

        if (evLong)
        {
            shared_lock();
            shared.spindleSpeed  = editSpeed;
            shared.wireDiameter  = editDia;
            shared.windingWidth  = editWidth;
            shared.dirForward    = editDir;
            shared.targetTurns   = editTargetTurns;
            shared_unlock();
            uiMode = UI_VIEW;
            needFullRedraw = true;
        }
    }

    // ════════════════════════════════════════════════════════════
    // SUBMENU_POS — вибір пункту
    // ════════════════════════════════════════════════════════════
    else if (uiMode == UI_SUBMENU_POS)
    {
        if (encDelta != 0)
        {
            if (encDelta > 0)
                submenuItem = (submenuItem + 1) % SUBMENU_COUNT;
            else
                submenuItem = (submenuItem + SUBMENU_COUNT - 1) % SUBMENU_COUNT;
            needFullRedraw = true;
            encDelta = 0;
        }

        if (evClick)
        {
            uiMode = UI_VIEW;
            needFullRedraw = true;
        }

        if (evLong)
        {
            uiMode = UI_SUBMENU_EDIT;
            needFullRedraw = true;
        }
    }

    // ════════════════════════════════════════════════════════════
    // SUBMENU_EDIT — редагування пункту
    // ════════════════════════════════════════════════════════════
    else if (uiMode == UI_SUBMENU_EDIT)
    {
        if (encDelta != 0)
        {
            switch (submenuItem)
            {
                case SUBMENU_SET_VALUE:
                    if (encDelta > 0 && editPos < CARRIAGE_POS_MAX)
                        editPos += CARRIAGE_POS_STEP;
                    if (encDelta < 0 && editPos > CARRIAGE_POS_MIN)
                        editPos -= CARRIAGE_POS_STEP;
                    break;
                case SUBMENU_AUTO_REV:
                    editAutoRev = !editAutoRev;
                    break;
                case SUBMENU_SET_HOME:
                    break;
            }
            needFullRedraw = true;
            encDelta = 0;
        }

        // Короткий натиск → скасувати, повернутись в підменю
        if (evClick)
        {
            shared_lock();
            editPos     = shared.carriagePos;
            editAutoRev = shared.autoReverse;
            shared_unlock();
            uiMode = UI_SUBMENU_POS;
            needFullRedraw = true;
        }

        // Довгий натиск → зберегти
        if (evLong)
        {
            switch (submenuItem)
            {
                case SUBMENU_SET_VALUE:
                    shared_lock();
                    shared.autoReverse = editAutoRev;
                    shared_unlock();
                    motion_move_to(editPos);
                    uiMode = UI_VIEW;
                    break;

                case SUBMENU_SET_HOME:
                    shared_lock();
                    shared.carriagePos = 0.0f;
                    shared_unlock();
                    lcd_clear();
                    lcd_set_cursor(3, 1); lcd_print("HOME SET!");
                    lcd_set_cursor(2, 2); lcd_print("pos = 0.00mm");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    uiMode = UI_VIEW;
                    break;

                case SUBMENU_AUTO_REV:
                    shared_lock();
                    shared.autoReverse = editAutoRev;
                    shared_unlock();
                    uiMode = UI_SUBMENU_POS;
                    break;
            }
            needFullRedraw = true;
        }
    }
}