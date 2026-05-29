// ui.cpp
//
// Макет екрану (4×20):
//
//   Col: 01234567890123456789
//   Row 0: >SPD: 300rpm FWD>>>
//   Row 1:  TURNS: 0000
//   Row 2: >POS:   0.00mm >>>
//   Row 3: >DIA:0.20  >W: 30.0
//
// Логіка керування (однакова скрізь):
//   Крутиш         → обираєш параметр / змінюєш значення
//   Довгий натиск  → входиш в редагування / зберігаєш
//   Короткий натиск → виходиш без збереження / виходиш в попереднє меню
//
// Підменю POS:
//   Крутиш         → обираєш пункт (Set value / Set home / Auto rev)
//   Довгий натиск  → входиш в редагування пункту
//   Короткий натиск → вихід в головне меню
//
//   В редагуванні пункту:
//   Крутиш         → змінюєш значення
//   Довгий натиск  → зберегти і вийти з редагування
//   Короткий натиск → скасувати і вийти з редагування

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
    UI_VIEW,           // головний екран
    UI_EDIT,           // редагування параметра головного меню
    UI_SUBMENU_POS,    // підменю POS — вибір пункту
    UI_SUBMENU_EDIT,   // редагування пункту підменю
} UiMode;

typedef enum
{
    PARAM_SPEED     = 0,
    PARAM_DIRECTION = 1,
    PARAM_POS       = 2,
    PARAM_DIAMETER  = 3,
    PARAM_WIDTH     = 4,
    PARAM_COUNT     = 5,
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

// Буфер редагування головного меню
static uint32_t editSpeed = 0;
static float    editDia   = 0.0f;
static float    editWidth = 0.0f;
static bool     editDir   = true;

// Підменю POS
static uint8_t submenuItem  = SUBMENU_SET_VALUE;
static float   editPos      = 0.0f;
static bool    editAutoRev  = true;

// Кеш відображених значень
static uint32_t lastTurns    = 0xFFFFFFFF;
static float    lastPos      = -9999.0f;
static uint32_t lastSpeed    = 0xFFFFFFFF;
static bool     lastDir      = true;
static bool     lastSvcActive = false;
static bool     lastSvcDir   = true;

// Позиції курсорів {col, row}
static const uint8_t CURSOR_POS[PARAM_COUNT][2] =
{
    {0,  0},   // SPEED
    {11, 0},   // DIRECTION
    {0,  2},   // POS
    {0,  3},   // DIAMETER
    {11, 3},   // WIDTH
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
// Малювання головного екрану
// ─────────────────────────────────────────

static void draw_speed(uint32_t spd)
{
    char buf[11];
    snprintf(buf, sizeof(buf), "SPD:%3lurpm", (unsigned long)spd);
    lcd_set_cursor(1, 0);
    lcd_print(buf);
    lastSpeed = spd;
}

static void draw_direction(bool fwd)
{
    lcd_set_cursor(12, 0);
    lcd_print(fwd ? "FWD>>>" : "REV<<<");
    lastDir = fwd;
}

static void draw_turns(uint32_t t)
{
    char buf[21];
    snprintf(buf, sizeof(buf), " TURNS: %04lu       ", (unsigned long)t);
    buf[20] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(buf);
    lastTurns = t;
}

static void draw_position_row(float pos, bool svcActive, bool svcDir,
                               bool running, bool fwd)
{
    // Завжди показуємо напрямок:
    // сервіс активний → напрямок руху каретки зараз
    // інакше          → dirForward (куди піде при старті)
    bool showDir  = svcActive ? svcDir : fwd;
    const char* arrow = showDir ? ">>>" : "<<<";

    char buf[21];
    snprintf(buf, sizeof(buf), " POS:%6.2fmm %s", pos, arrow);
    buf[20] = '\0';
    lcd_set_cursor(0, 2);
    lcd_print(buf);
    lastPos       = pos;
    lastSvcActive = svcActive;
    lastSvcDir    = svcDir;
}

static void draw_diameter(float dia)
{
    char buf[10];
    snprintf(buf, sizeof(buf), "DIA:%.2f ", dia);
    lcd_set_cursor(1, 3);
    lcd_print(buf);
}

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
    lastTurns     = 0xFFFFFFFF;
    lastPos       = -9999.0f;
    lastSpeed     = 0xFFFFFFFF;
    lastDir       = !s.dirForward;
    lastSvcActive = false;

    draw_speed(s.spindleSpeed);
    draw_direction(s.dirForward);
    draw_turns(s.turns);
    draw_position_row(s.carriagePos, s.serviceCarriageActive,
                      s.serviceCarriageDir, s.running, s.dirForward);
    draw_diameter(s.wireDiameter);
    draw_width(s.windingWidth);

    lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
    lcd_print(">");
}

// ─────────────────────────────────────────
// Малювання підменю POS
// ─────────────────────────────────────────

static void draw_submenu_pos(const SharedState& s, bool editMode)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("-- POS SETTINGS --");

    // Рядок 1: Set value
    {
        char buf[21];
        char cursor = (submenuItem == SUBMENU_SET_VALUE) ?
                      (editMode ? '*' : '>') : ' ';
        snprintf(buf, sizeof(buf), "%cSet val: %6.1fmm", cursor, editPos);
        buf[20] = '\0';
        lcd_set_cursor(0, 1);
        lcd_print(buf);
    }

    // Рядок 2: Set home
    {
        char buf[21];
        char cursor = (submenuItem == SUBMENU_SET_HOME) ?
                      (editMode ? '*' : '>') : ' ';
        snprintf(buf, sizeof(buf), "%cSet home (0.00mm)", cursor);
        buf[20] = '\0';
        lcd_set_cursor(0, 2);
        lcd_print(buf);
    }

    // Рядок 3: Auto rev
    {
        char buf[21];
        char cursor = (submenuItem == SUBMENU_AUTO_REV) ?
                      (editMode ? '*' : '>') : ' ';
        snprintf(buf, sizeof(buf), "%cAuto rev:     %s",
                 cursor, editAutoRev ? "ON " : "OFF");
        buf[20] = '\0';
        lcd_set_cursor(0, 3);
        lcd_print(buf);
    }
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

    // ── Повне перемалювання ──────────────────────────────────────
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
    // VIEW — головний екран
    // ════════════════════════════════════════════════════════════
    if (uiMode == UI_VIEW)
    {
        // Крутіння → переміщення курсора (тільки коли стоїть)
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

        // Довгий натиск → редагування
        if (evLong)
        {
            if (selectedParam == PARAM_POS)
            {
                // Відкрити підменю POS
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
                editSpeed = s.spindleSpeed;
                editDia   = s.wireDiameter;
                editWidth = s.windingWidth;
                editDir   = s.dirForward;
                uiMode    = UI_EDIT;
                lcd_set_cursor(CURSOR_POS[selectedParam][0],
                               CURSOR_POS[selectedParam][1]);
                lcd_print("*");
            }
        }

        // Оновлення динамічних полів
        if (s.spindleSpeed != lastSpeed)
            draw_speed(s.spindleSpeed);
        if (s.dirForward != lastDir)
            draw_direction(s.dirForward);
        if (s.turns != lastTurns)
            draw_turns(s.turns);
        if (fabsf(s.carriagePos - lastPos) > 0.005f  ||
            s.serviceCarriageActive != lastSvcActive  ||
            s.serviceCarriageDir    != lastSvcDir)
        {
            draw_position_row(s.carriagePos, s.serviceCarriageActive,
                              s.serviceCarriageDir, s.running, s.dirForward);
        }
    }

    // ════════════════════════════════════════════════════════════
    // EDIT — редагування параметра головного меню
    // ════════════════════════════════════════════════════════════
    else if (uiMode == UI_EDIT)
    {
        if (encDelta != 0)
        {
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

        // Короткий натиск → скасувати
        if (evClick)
        {
            uiMode = UI_VIEW;
            needFullRedraw = true;
        }

        // Довгий натиск → зберегти
        if (evLong)
        {
            shared_lock();
            shared.spindleSpeed = editSpeed;
            shared.wireDiameter = editDia;
            shared.windingWidth = editWidth;
            shared.dirForward   = editDir;
            shared_unlock();
            uiMode = UI_VIEW;
            needFullRedraw = true;
        }
    }

    // ════════════════════════════════════════════════════════════
    // SUBMENU_POS — вибір пункту підменю
    // ════════════════════════════════════════════════════════════
    else if (uiMode == UI_SUBMENU_POS)
    {
        // Крутіння → навігація між пунктами
        if (encDelta != 0)
        {
            if (encDelta > 0)
                submenuItem = (submenuItem + 1) % SUBMENU_COUNT;
            else
                submenuItem = (submenuItem + SUBMENU_COUNT - 1) % SUBMENU_COUNT;
            needFullRedraw = true;
            encDelta = 0;
        }

        // Короткий натиск → вийти в головне меню
        if (evClick)
        {
            uiMode = UI_VIEW;
            needFullRedraw = true;
        }

        // Довгий натиск → увійти в редагування пункту
        if (evLong)
        {
            uiMode = UI_SUBMENU_EDIT;
            needFullRedraw = true;
        }
    }

    // ════════════════════════════════════════════════════════════
    // SUBMENU_EDIT — редагування пункту підменю
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
                    // Нічого — тут тільки підтвердження
                    break;
            }
            needFullRedraw = true;
            encDelta = 0;
        }

        // Короткий натиск → скасувати, вийти в підменю
        if (evClick)
        {
            // Відновлюємо оригінальні значення
            shared_lock();
            editPos     = shared.carriagePos;
            editAutoRev = shared.autoReverse;
            shared_unlock();
            uiMode = UI_SUBMENU_POS;
            needFullRedraw = true;
        }

        // Довгий натиск → зберегти і вийти в підменю
        if (evLong)
        {
            switch (submenuItem)
            {
                case SUBMENU_SET_VALUE:
                    // Зберегти autoReverse і запустити переміщення
                    shared_lock();
                    shared.autoReverse = editAutoRev;
                    shared_unlock();
                    motion_move_to(editPos);
                    // Повертаємось в головне меню
                    uiMode = UI_VIEW;
                    break;

                case SUBMENU_SET_HOME:
                    // Скинути позицію в 0
                    shared_lock();
                    shared.carriagePos = 0.0f;
                    shared_unlock();
                    // Підтвердження на екрані
                    lcd_clear();
                    lcd_set_cursor(3, 1);
                    lcd_print("HOME SET!");
                    lcd_set_cursor(2, 2);
                    lcd_print("pos = 0.00mm");
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